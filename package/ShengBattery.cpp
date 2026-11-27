// SPDX-License-Identifier: BSD-2-Clause-Patent
// The battery as iOS reads it (boot argument battery=1, started by BatteryProbe.cpp; needs adsp=2): an IOPMPowerSource whose
// values come from the battery manager in the audio DSP over the PMIC GLINK channel (ShengBattmgr.c). powerd finds every IOPMPowerSource,
// reads CurrentCapacity/MaxCapacity, IsCharging, ExternalConnected, voltage, amperage and AdapterDetails and listens for the status change
// message updateStatus sends; the status bar, the lock screen's charging state and Settings > Battery follow from that.
//
// Design: everything runs on the service's work loop. The DSP thread's callbacks only copy a message into a small ring and signal an
// IOInterruptEventSource (no provider: a software event), so the DSP thread never waits for this driver. One request is in flight at a
// time; a 2 s timer gives up on an unanswered one. Battery notifications start a refresh at once, and a 60 s timer refreshes in any case
// (the fields that hardly change, every tenth time). Charging itself is the DSP firmware's; writes synchronize keyboard attachment and the input current
// limit on a computer's USB port (ShengBattmgr.c; boot argument usb-sdp-ma, 0 leaves the firmware's limit).
#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOPlatformExpert.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSString.h>
#include <libkern/c++/OSSymbol.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <libkern/crypto/sha2.h>
#include <sys/random.h>
extern "C" {
#include "ShengAdsp.h"
#include "ShengBattmgr.h"
#include "ShengChargeAuth.h"
}

extern "C" void qcom_dwc3_vbus(int) __attribute__((weak_import));
// The USB-C owner (32779) shares this channel: QcomDwc3 asks what is on the cable over UCSI.
extern "C" uint32_t qcom_ucsi_start(uint8_t *) __attribute__((weak_import));
extern "C" uint32_t qcom_ucsi_receive(const uint8_t *, uint32_t, uint8_t *) __attribute__((weak_import));
extern "C" uint32_t qcom_ucsi_tick(uint8_t *) __attribute__((weak_import));
extern "C" bool qcom_ucsi_polling(void) __attribute__((weak_import));
extern "C" uint32_t qcom_ucsi_request_data_host(uint8_t *) __attribute__((weak_import));
extern "C" int qcom_ucsi_data_host(void) __attribute__((weak_import));
extern "C" bool qcom_ucsi_connection(uint32_t *, uint32_t *) __attribute__((weak_import));
extern "C" ShengAdspChannel *sheng_adsp_channel(const char *, uint32_t, uint32_t, int, const ShengAdspClient *, void *) __attribute__((weak_import));
extern "C" int sheng_adsp_send(ShengAdspChannel *, const void *, uint32_t) __attribute__((weak_import));
// [0] stage (1 class up, 2 channel registered, 3 channel up, 4 published; negative: the step that failed), [1] percent, [2] mV,
// [3] mA, [4] temperature (raw), [5] flags (bit 0 present, 1 external, 2 charging, 3 full, 4 notifications enabled), [6] refresh
// cycles, [7] answers, [8] timeouts | errors << 16, [9] notifications, [10] USB type (raw), [11] full mAh, [12] design mAh,
// [13] cycles, [14] messages dropped (ring full) | ignored << 16, [15] send refusals
extern "C" {extern volatile int32_t native_battery_stats[16];}
// The raw readings behind the values: [0] capacity property (hundredths of a percent), [1] gauge 1 remaining, [2] gauge 1 full, [3] gauge 2
// remaining, [4] gauge 2 full, [5] temperature property, [6] current property (uA, discharge positive), [7] valid field bits, [8] the field
// the last timeout skipped (-1 none)
extern "C" {volatile int32_t native_battery_raw[10];}
// The charger as the firmware reports it (-1 no reading): [0] USB online, [1] USB type, [2] adapter type, [3] the board's adapter type,
// [4] input current limit mA, [5] the most the port offers mA, [6] USB mV, [7] USB mA, [8] battery status, [9] charge type, [10] thermal
// level, [11] charge current mA, [12] adapter watts, [13] bits: 0 cell 1 authenticated, 1 cell 2, 2 PD adapter authenticated, 3 input
// suspended (each with bit +8 set when read), [14] limit sets | tries << 8 | port type << 16, [15] last limit result
extern "C" {volatile int32_t native_charger_stats[16];}
extern "C" {volatile int32_t native_charger_auth_stats[32];}

namespace {
// The keyboard thread only writes atomics. This service and its event source live for the platform lifetime.
volatile int gKeyboardAttached = -1;
IOInterruptEventSource *volatile gKeyboardInbox;
constexpr uint32_t kRequestMs = 2000, kRefreshMs = 60000, kSlowEvery = 10, kRing = 16, kMessage = 560;
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
constexpr const char *kLogicalBatterySerial = "Sheng-DualCell-A+B";
constexpr uint64_t kPackNominalHealthMetric = 1ULL << 10;
#endif
uint64_t chargeNowMs() {uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}
void chargeRandom(void *bytes, uint32_t count) {read_random(bytes, count);}
void chargeHmac(const uint8_t *key, uint32_t keyBytes, const uint8_t *message, uint32_t bytes, uint8_t digest[32]) {
    uint8_t pad[64], inner[32];
    SHA256_CTX sha;
    for (unsigned i = 0; i < sizeof pad; ++i) pad[i] = (i < keyBytes ? key[i] : 0) ^ 0x36;
    SHA256_Init(&sha); SHA256_Update(&sha, pad, sizeof pad); SHA256_Update(&sha, message, bytes); SHA256_Final(inner, &sha);
    for (unsigned i = 0; i < sizeof pad; ++i) pad[i] ^= 0x36 ^ 0x5c;
    SHA256_Init(&sha); SHA256_Update(&sha, pad, sizeof pad); SHA256_Update(&sha, inner, sizeof inner); SHA256_Final(digest, &sha);
    bzero(pad, sizeof pad); bzero(inner, sizeof inner); bzero(&sha, sizeof sha);
}
const ShengChargeCrypto kChargeCrypto = {chargeRandom, chargeHmac};
}

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
// powerd 27.3 matches AppleSmartBatteryPack (ID=0) and AppleSmartBattery for macOS battery health.
class AppleSmartBatteryPack : public IOService {
    OSDeclareDefaultStructors(AppleSmartBatteryPack)
};
OSDefineMetaClassAndStructors(AppleSmartBatteryPack, IOService)

class AppleSmartBattery : public IOPMPowerSource {
    OSDeclareDefaultStructors(AppleSmartBattery)
    using Self = AppleSmartBattery;
    AppleSmartBatteryPack *healthPack = nullptr;
#else
class ShengBattery : public IOPMPowerSource {
    OSDeclareDefaultStructors(ShengBattery)
    using Self = ShengBattery;
#endif
    IOWorkLoop *loop = nullptr;
    IOTimerEventSource *request = nullptr, *refresh = nullptr;
    IOInterruptEventSource *inbox = nullptr;
    IOSimpleLock *ringLock = nullptr;
    struct {uint32_t length; uint8_t data[kMessage];} ring[kRing];
    uint32_t ringHead = 0, ringCount = 0;
    volatile uint32_t channelUp = 0, channelChanged = 0, channelLost = 0;
    bool wasUp = false, published = false;
    uint32_t refreshes = 0;
    ShengBattmgr bm;
    ShengChargeAuth auth;
    bool chargeAuthEnabled = true;
    bool cableKnown = false, usbDisconnected = false;
    uint32_t cableGeneration = 0;
    IOTimerEventSource *authTimer = nullptr;
    uint32_t sdpLimit = 0;
    ShengAdspChannel *channel = nullptr;
    IOTimerEventSource *ucsiTimer = nullptr;
    uint8_t ucsiOut[kMessage];
    // Diagnosis (boot argument ucsi-delay, seconds): the first UCSI command (PPM_RESET) waits this long after the channel came up, so the
    // charger rows before it show what the port was doing without the reset.
    uint32_t ucsiDelayMs = 0;
    bool ucsiPending = false;

    bool observeCable() {
        uint32_t generation = 0, status = 0;
        if (!qcom_ucsi_connection || !qcom_ucsi_connection(&generation, &status)) return false;
        const uint64_t now = chargeNowMs();
        if (!cableKnown || generation != cableGeneration) {
            cableKnown = true; cableGeneration = generation; usbDisconnected = false;
            sheng_charge_auth_reconnect(&auth, now);
            authTimer->cancelTimeout();
            sheng_battmgr_refresh(&bm, 1);
        }
        const int connected = ((status >> 19) & 1) && !usbDisconnected;
        const int pd_sink = ((status >> 16) & 7) == 3 && !((status >> 20) & 1);
        sheng_charge_auth_observe(&auth, connected, pd_sink ? 6 : 5, now);
        return true;
    }

    void ucsiSend(uint32_t n) {
        if (n) sheng_adsp_send(channel, ucsiOut, n);
        if (qcom_ucsi_polling && qcom_ucsi_polling()) ucsiTimer->setTimeoutMS(20);
    }
    static void ucsiTick(OSObject *owner, IOTimerEventSource *) {
        Self *self = OSDynamicCast(Self, owner);
        if (self && self->wasUp && self->ucsiPending) {
            self->ucsiPending = false;
            if (qcom_ucsi_start) self->ucsiSend(qcom_ucsi_start(self->ucsiOut));
            return;
        }
        if (self && self->wasUp && qcom_ucsi_tick) self->ucsiSend(qcom_ucsi_tick(self->ucsiOut));
    }

    static void dspState(void *context, int up) {
        Self *self = static_cast<Self *>(context);
        if (!up) __atomic_store_n(&self->channelLost, 1u, __ATOMIC_RELEASE);
        __atomic_store_n(&self->channelUp, up ? 1u : 0u, __ATOMIC_RELEASE);
        __atomic_store_n(&self->channelChanged, 1u, __ATOMIC_RELEASE);
        self->inbox->interruptOccurred(nullptr, nullptr, 0);
    }
    static void dspReceive(void *context, const uint8_t *data, uint32_t length) {
        Self *self = static_cast<Self *>(context);
        IOSimpleLockLock(self->ringLock);
        if (self->ringCount < kRing && length <= kMessage) {
            auto &slot = self->ring[(self->ringHead + self->ringCount) % kRing];
            memcpy(slot.data, data, length);
            slot.length = length;
            self->ringCount++;
        } else {
            native_battery_stats[14] = native_battery_stats[14] + 1;
        }
        IOSimpleLockUnlock(self->ringLock);
        self->inbox->interruptOccurred(nullptr, nullptr, 0);
    }

    void pump() {
        if (!wasUp) return;
        if (auth.pending || bm.in_flight || bm.limit_in_flight || bm.keyboard_in_flight >= 0) return;
        uint8_t out[SHENG_CHARGE_REQUEST_BYTES];
        uint32_t n = sheng_battmgr_next(&bm, out);
        if (!n) {
            if (auth.state == ChargeHostRole) {
                auth.host_ready = qcom_ucsi_data_host ? qcom_ucsi_data_host() : -95;
                if (!auth.host_ready && qcom_ucsi_request_data_host) ucsiSend(qcom_ucsi_request_data_host(ucsiOut));
            }
            n = sheng_charge_auth_next(&auth, chargeNowMs(), out);
        }
        if (!n) {
            const uint32_t delay = sheng_charge_auth_wait_ms(&auth, chargeNowMs());
            if (delay) authTimer->setTimeoutMS(delay);
            return;
        }
        if (sheng_adsp_send(channel, out, n)) native_battery_stats[15] = native_battery_stats[15] + 1;  // the timer skips the field
        request->setTimeoutMS(kRequestMs);
    }
    void number(const char *key, int64_t value) {
        const OSSymbol *symbol = OSSymbol::withCString(key);
        OSNumber *n = OSNumber::withNumber((unsigned long long)value, 32);
        if (symbol && n) setPSProperty(symbol, n);
        OSSafeReleaseNULL(n); OSSafeReleaseNULL(symbol);
    }
    void flag(const char *key, bool value) {
        const OSSymbol *symbol = OSSymbol::withCString(key);
        if (symbol) setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
        OSSafeReleaseNULL(symbol);
    }
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
    void text(const char *key, const char *value) {
        const OSSymbol *symbol = OSSymbol::withCString(key);
        OSString *s = OSString::withCString(value);
        if (symbol && s) setPSProperty(symbol, s);
        OSSafeReleaseNULL(s); OSSafeReleaseNULL(symbol);
    }
    static bool dictNumber(OSDictionary *dict, const char *key, uint64_t value, unsigned bits) {
        OSNumber *n = OSNumber::withNumber(value, bits);
        const bool ok = n && dict->setObject(key, n);
        OSSafeReleaseNULL(n);
        return ok;
    }
    void updateHealthPack(const ShengBatterySnapshot &s) {
        if (!s.present || !s.charge_known || s.design_mah <= 0 || s.design_mah > 20000 ||
            s.full_mah < 2000 || s.full_mah > 20000 || (uint64_t)s.full_mah * 100 > (uint64_t)s.design_mah * 150 ||
            s.raw_mah <= 0 || s.raw_mah > s.full_mah + s.full_mah / 20 || s.cycles < 0 || s.cycles >= 65535)
            return;
        OSDictionary *data = OSDictionary::withCapacity(11);
        OSString *serial = OSString::withCString(kLogicalBatterySerial);
        bool ok = data && serial;
        if (ok) {
            ok = data->setObject("Serial", serial);
            ok = dictNumber(data, "RemainingCapacity", (uint64_t)(uint32_t)s.raw_mah, 32) && ok;
            ok = dictNumber(data, "MaxCapacity", (uint64_t)(uint32_t)s.full_mah, 32) && ok;
            ok = dictNumber(data, "AppleRawMaxCapacity", (uint64_t)(uint32_t)s.full_mah, 32) && ok;
            ok = dictNumber(data, "DesignCapacity", (uint64_t)(uint32_t)s.design_mah, 32) && ok;
            ok = dictNumber(data, "NominalChargeCapacity", (uint64_t)(uint32_t)s.full_mah, 32) && ok;
            ok = dictNumber(data, "CycleCount", (uint64_t)(uint32_t)s.cycles, 32) && ok;
            ok = dictNumber(data, "Voltage", (uint64_t)(uint32_t)(s.voltage_mv > 0 ? s.voltage_mv : 0), 32) && ok;
            ok = dictNumber(data, "AverageCurrent", (uint64_t)(int64_t)s.current_ma, 32) && ok;
            ok = dictNumber(data, "FullyCharged", s.fully_charged ? 1ULL : 0ULL, 32) && ok;
            if (s.temperature_valid) ok = dictNumber(data, "Temperature", (uint64_t)(int64_t)s.temperature, 32) && ok;
        }
        OSSafeReleaseNULL(serial);
        if (!ok) {
            OSSafeReleaseNULL(data);
            return;
        }
        const bool isNew = healthPack == nullptr;
        AppleSmartBatteryPack *pack = healthPack;
        if (!pack) {
            pack = OSTypeAlloc(AppleSmartBatteryPack);
            if (!pack || !pack->init() || !pack->attach(this)) {
                OSSafeReleaseNULL(pack);
                data->release();
                return;
            }
            OSNumber *idNum = OSNumber::withNumber(0ULL, 32);
            OSNumber *flags = OSNumber::withNumber(kPackNominalHealthMetric, 64);
            ok = idNum && flags && pack->setProperty("ID", idNum) && pack->setProperty("FeatureFlags", flags);
            OSSafeReleaseNULL(idNum);
            OSSafeReleaseNULL(flags);
        }
        ok = ok && pack->setProperty("BatteryData", data);
        data->release();
        if (!ok) {
            if (isNew) {
                pack->detach(this);
                pack->release();
            }
            return;
        }
        if (isNew) {
            healthPack = pack;
            pack->registerService();
        }
    }
#endif
    void recordRaw() {
        static const int fields[8] = {BmCapacity, BmGauge1Remaining, BmGauge1Full, BmGauge2Remaining, BmGauge2Full, BmTemperature, BmCurrent, -1};
        for (unsigned i = 0; i < 7; ++i) native_battery_raw[i] = (bm.valid >> fields[i]) & 1 ? bm.value[fields[i]] : -999999;
        native_battery_raw[7] = (int32_t)bm.valid;
        native_battery_raw[8] = bm.last_timeout;
        static const int charger[13] = {BmUsbOnline, BmUsbType, BmUsbAdapter, BmRealType, BmUsbLimit, BmUsbMax, BmUsbVoltage, BmUsbCurrent,
                                        BmStatus, BmChargeType, BmThermalLevel, BmChargeCurrent, BmPowerMax};
        static const int milli[13] = {0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0};
        if (qcom_dwc3_vbus && (bm.valid >> BmUsbOnline) & 1) qcom_dwc3_vbus(bm.value[BmUsbOnline]);
        for (unsigned i = 0; i < 13; ++i)
            native_charger_stats[i] = (bm.valid >> charger[i]) & 1 ? (milli[i] ? bm.value[charger[i]] / 1000 : bm.value[charger[i]]) : -1;
        static const int bits[4] = {BmAuthentic, BmSlaveAuthentic, BmPdVerified, BmInputSuspend};
        int32_t flags = 0;
        for (unsigned i = 0; i < 4; ++i)
            if ((bm.valid >> bits[i]) & 1) flags |= (bm.value[bits[i]] != 0) << i | 1 << (i + 8);
        native_charger_stats[13] = flags;
        native_charger_stats[14] = (int32_t)(bm.limit_sets | bm.limit_tries << 8 | (uint32_t)(bm.limit_port & 0xff) << 16);
        native_charger_stats[15] = bm.limit_result;
    }
    void publish() {
        ShengBatterySnapshot s;
        sheng_battmgr_snapshot(&bm, &s);
        setBatteryInstalled(s.present);
        if (s.external_known) {
            setExternalConnected(s.external);
            setExternalChargeCapable(s.external);
        }
        if (s.external_known && s.status_known) setIsCharging(s.charging);
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
        setAtWarnLevel(false);
        setAtCriticalLevel(false);
#else
        setAtCriticalLevel(s.critical);
#endif
        setCurrentCapacity((unsigned)s.percent);
        setMaxCapacity(100);
        setVoltage((unsigned)(s.voltage_mv > 0 ? s.voltage_mv : 0));
        setAmperage(s.current_ma);
        if (s.cycles > 0) setCycleCount((unsigned)s.cycles);
        setTimeRemaining(s.charging ? (s.time_to_full_min >= 0 ? s.time_to_full_min : 0xffff)
                                    : (s.time_to_empty_min >= 0 ? s.time_to_empty_min : 0xffff));
        if (s.external_known && s.status_known) flag("FullyCharged", s.fully_charged);
        number("InstantAmperage", s.current_ma);
        if (s.temperature_valid) number("Temperature", s.temperature);
        if (s.raw_mah > 0) number("AppleRawCurrentCapacity", s.raw_mah);
        if (s.full_mah > 0) {number("AppleRawMaxCapacity", s.full_mah); number("NominalChargeCapacity", s.full_mah);}
        if (s.design_mah > 0) number("DesignCapacity", s.design_mah);
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
        text(kIOPMPSSerialKey, kLogicalBatterySerial);
        if (s.design_mah > 0 && s.full_mah > 0) {
            number(kIOPMPSBatteryHealthKey, ((uint64_t)s.full_mah * 100 < (uint64_t)s.design_mah * 80) ? kIOPMFairValue : kIOPMGoodValue);
        }
        number("AvgTimeToEmpty", s.time_to_empty_min >= 0 ? s.time_to_empty_min : 0xffff);
        number("AvgTimeToFull", s.time_to_full_min >= 0 ? s.time_to_full_min : 0xffff);
        {
            clock_sec_t sec = 0; clock_usec_t usec = 0;
            clock_get_calendar_microtime(&sec, &usec);
            if (sec) number("UpdateTime", (int64_t)sec);
        }
        flag("built-in", true);
        if (s.external_known) flag("AppleRawExternalConnected", s.external);
        text("DeviceName", "Xiaomi Pad 6S Pro Dual-Cell");
        text("Manufacturer", "Xiaomi");
#endif
        OSDictionary *adapter = OSDictionary::withCapacity(7);
        if (adapter) {
            OSNumber *family = OSNumber::withNumber(s.adapter_family, 32);
            if (family) {adapter->setObject(kIOPMPSAdapterDetailsFamilyKey, family); family->release();}
            if (s.external) {
                OSNumber *w = OSNumber::withNumber((unsigned long long)s.adapter_watts, 32), *mv = OSNumber::withNumber((unsigned long long)s.adapter_mv, 32),
                         *ma = OSNumber::withNumber((unsigned long long)s.adapter_ma, 32);
                if (w) {adapter->setObject(kIOPMPSAdapterDetailsWattsKey, w); w->release();}
                if (mv) {adapter->setObject(kIOPMPSAdapterDetailsVoltage, mv); mv->release();}
                if (ma) {adapter->setObject(kIOPMPSAdapterDetailsAmperageKey, ma); ma->release();}
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
                OSNumber *idNum = OSNumber::withNumber(0ULL, 32);
                if (idNum) {adapter->setObject(kIOPMPSAdapterDetailsIDKey, idNum); idNum->release();}
                adapter->setObject("IsWireless", kOSBooleanFalse);
#endif
            }
            const OSSymbol *key = OSSymbol::withCString(kIOPMPSAdapterDetailsKey);
            if (key) setPSProperty(key, adapter);
            OSSafeReleaseNULL(key);
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
            const OSSymbol *rawKey = OSSymbol::withCString("AppleRawAdapterDetails");
            if (rawKey) setPSProperty(rawKey, adapter);
            OSSafeReleaseNULL(rawKey);
#endif
            adapter->release();
        }
        native_battery_stats[1] = s.percent; native_battery_stats[2] = s.voltage_mv; native_battery_stats[3] = s.current_ma;
        native_battery_stats[4] = s.temperature;
        native_battery_stats[5] = s.present | s.external << 1 | s.charging << 2 | s.fully_charged << 3 | bm.enabled << 4;
        native_battery_stats[10] = (bm.valid >> BmUsbType) & 1 ? bm.value[BmUsbType] : -1;
        native_battery_stats[11] = s.full_mah; native_battery_stats[12] = s.design_mah; native_battery_stats[13] = s.cycles;
        recordRaw();
        if (!published && !s.charge_known) return;  // no charge reading yet: 0 % would read as an empty battery
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
        updateHealthPack(s);
#endif
        if (!published) {
            // The properties are in place before anyone finds the service.
            published = true;
            registerService();
            native_battery_stats[0] = 4;
        }
        updateStatus();
    }
    void counters() {
        native_battery_stats[6] = (int32_t)bm.cycles_done;
        native_battery_stats[7] = (int32_t)bm.responses;
        native_battery_stats[8] = (int32_t)(bm.timeouts | bm.errors << 16);
        native_battery_stats[9] = (int32_t)bm.notifications;
        native_battery_stats[14] = (native_battery_stats[14] & 0xffff) | (int32_t)(bm.ignored << 16);
        native_charger_auth_stats[0] = (int32_t)auth.state;
        native_charger_auth_stats[1] = (int32_t)auth.generation;
        native_charger_auth_stats[2] = auth.result; native_charger_auth_stats[3] = auth.last_result;
        native_charger_auth_stats[4] = (int32_t)auth.gauge_verified; native_charger_auth_stats[5] = (int32_t)auth.adapter_verified;
        native_charger_auth_stats[6] = auth.pending ? (int32_t)(auth.opcode | auth.property << 8) : 0;
        native_charger_auth_stats[7] = (int32_t)auth.reads; native_charger_auth_stats[8] = (int32_t)auth.writes;
        native_charger_auth_stats[9] = (int32_t)auth.errors; native_charger_auth_stats[10] = (int32_t)auth.timeouts;
        native_charger_auth_stats[11] = (int32_t)auth.retries; native_charger_auth_stats[12] = auth.host_ready;
        native_charger_auth_stats[13] = (int32_t)auth.diagnostic_valid;
        for (unsigned i = 0; i < ChargeDiagnosticCount; ++i)
            native_charger_auth_stats[14+i] = (auth.diagnostic_valid >> i) & 1 ? auth.diagnostic[i] : -1;
    }
    void handle(int result) {
        if (result == BmCycleDone) {
            publish();
            if (!observeCable() && (bm.valid >> BmUsbOnline) & 1) {
                int type = (bm.valid >> BmUsbAdapter) & 1 ? bm.value[BmUsbAdapter] : 0;
                if (!type && (bm.valid >> BmUsbType) & 1) type = bm.value[BmUsbType];
                if (type < 6 && (bm.valid >> BmRealType) & 1 && bm.value[BmRealType] >= 6 && bm.value[BmRealType] <= 8)
                    type = bm.value[BmRealType];
                sheng_charge_auth_observe(&auth, bm.value[BmUsbOnline], type, chargeNowMs());
            }
        }
        pump();
        counters();
    }

    static void inboxAction(OSObject *owner, IOInterruptEventSource *, int) {
        Self *self = OSDynamicCast(Self, owner);
        if (!self || !self->channel) return;
        if (__atomic_exchange_n(&self->channelChanged, 0u, __ATOMIC_ACQ_REL)) {
            const bool up = __atomic_load_n(&self->channelUp, __ATOMIC_ACQUIRE) != 0;
            // A down/up pair may coalesce into one event. Remember the loss so the latest cover state is always replayed.
            if (__atomic_exchange_n(&self->channelLost, 0u, __ATOMIC_ACQ_REL)) {
                self->request->cancelTimeout();
                self->authTimer->cancelTimeout();
                sheng_battmgr_init(&self->bm);
                sheng_charge_auth_init(&self->auth, self->chargeAuthEnabled ? &kChargeCrypto : nullptr);
                self->cableKnown = self->usbDisconnected = false;
                self->bm.sdp_limit_ua = self->sdpLimit;
                self->wasUp = false;
            }
            const bool before = self->wasUp;
            self->wasUp = up;
            if (up && !before) {
                const int attached = __atomic_load_n(&gKeyboardAttached, __ATOMIC_ACQUIRE);
                if (attached >= 0) sheng_battmgr_keyboard(&self->bm, attached);
                native_battery_stats[0] = self->published ? 4 : 3;
                uint8_t enable[SHENG_BATTMGR_ENABLE_BYTES];
                sheng_adsp_send(self->channel, enable, sheng_battmgr_enable_request(enable));
                if (qcom_ucsi_start && self->ucsiDelayMs) {self->ucsiPending = true; self->ucsiTimer->setTimeoutMS(self->ucsiDelayMs);}
                else if (qcom_ucsi_start) self->ucsiSend(qcom_ucsi_start(self->ucsiOut));
                sheng_battmgr_refresh(&self->bm, 1);
                self->pump();
                self->refresh->setTimeoutMS(kRefreshMs);
            }
            if (!up && before) {
                self->request->cancelTimeout(); self->authTimer->cancelTimeout();
                sheng_battmgr_init(&self->bm); self->bm.sdp_limit_ua = self->sdpLimit;
                sheng_charge_auth_init(&self->auth, self->chargeAuthEnabled ? &kChargeCrypto : nullptr);
                self->cableKnown = self->usbDisconnected = false;
            }
        }
        const int attached = __atomic_load_n(&gKeyboardAttached, __ATOMIC_ACQUIRE);
        if (self->wasUp && attached >= 0) {
            sheng_battmgr_keyboard(&self->bm, attached);
            self->pump();
        }
        for (;;) {
            uint8_t data[kMessage];
            uint32_t length = 0;
            IOSimpleLockLock(self->ringLock);
            if (self->ringCount) {
                auto &slot = self->ring[self->ringHead];
                length = slot.length;
                memcpy(data, slot.data, length);
                self->ringHead = (self->ringHead + 1) % kRing;
                self->ringCount--;
            }
            IOSimpleLockUnlock(self->ringLock);
            if (!length) break;
            if (qcom_ucsi_receive && length >= 4 && (data[0] | data[1] << 8 | data[2] << 16 | (uint32_t)data[3] << 24) == 32779u) {
                self->ucsiSend(qcom_ucsi_receive(data, length, self->ucsiOut));
                self->observeCable();
                self->pump();
                continue;
            }
            if (sheng_charge_auth_receive(&self->auth, data, length, chargeNowMs())) {
                self->request->cancelTimeout();
                if (self->auth.state == ChargeDiagnostics && !self->auth.diagnostic_cursor)
                    sheng_battmgr_refresh(&self->bm, 1);
                self->handle(BmAnswer);
                continue;
            }
            const bool onlineReply = self->bm.in_flight && self->bm.cursor == BmUsbOnline;
            const int result = sheng_battmgr_receive(&self->bm, data, length);
            if (onlineReply && (result == BmAnswer || result == BmCycleDone) && (self->bm.valid >> BmUsbOnline) & 1) {
                self->usbDisconnected = self->bm.value[BmUsbOnline] == 0;
                if (self->usbDisconnected) {
                    sheng_charge_auth_observe(&self->auth, 0, 0, chargeNowMs());
                    self->authTimer->cancelTimeout();
                }
            }
            if (result == BmAnswer || result == BmCycleDone) self->request->cancelTimeout();
            self->handle(result);
        }
    }
    static void requestAction(OSObject *owner, IOTimerEventSource *) {
        Self *self = OSDynamicCast(Self, owner);
        if (!self || !self->wasUp) return;
        if (self->auth.pending) {
            sheng_charge_auth_timeout(&self->auth, chargeNowMs()); self->handle(BmAnswer); return;
        }
        const uint32_t before = self->bm.cycles_done;
        sheng_battmgr_timeout(&self->bm);
        self->handle(self->bm.cycles_done != before ? BmCycleDone : BmAnswer);
    }
    static void refreshAction(OSObject *owner, IOTimerEventSource *sender) {
        Self *self = OSDynamicCast(Self, owner);
        if (!self || !self->wasUp) return;
        sheng_battmgr_refresh(&self->bm, ++self->refreshes % kSlowEvery == 0);
        self->handle(BmAnswer);
        sender->setTimeoutMS(kRefreshMs);
    }
    static void authAction(OSObject *owner, IOTimerEventSource *) {
        Self *self = OSDynamicCast(Self, owner);
        if (self && self->wasUp) self->handle(BmAnswer);
    }

public:
    bool startBattery() {
        sheng_battmgr_init(&bm);
        uint32_t authentication = 1;
        PE_parse_boot_argn("charge-auth", &authentication, sizeof authentication);
        chargeAuthEnabled = authentication != 0;
        sheng_charge_auth_init(&auth, chargeAuthEnabled ? &kChargeCrypto : nullptr);
        uint32_t ma = 0;
        if (PE_parse_boot_argn("usb-sdp-ma", &ma, sizeof ma)) bm.sdp_limit_ua = ma > 3000 ? 3000000u : ma * 1000u;
        sdpLimit = bm.sdp_limit_ua;
        uint32_t delay = 0;
        if (PE_parse_boot_argn("ucsi-delay", &delay, sizeof delay)) ucsiDelayMs = delay > 600 ? 600000u : delay * 1000u;
        loop = IOWorkLoop::workLoop();
        ringLock = IOSimpleLockAlloc();
        inbox = IOInterruptEventSource::interruptEventSource(this, &Self::inboxAction);
        request = IOTimerEventSource::timerEventSource(this, &Self::requestAction);
        refresh = IOTimerEventSource::timerEventSource(this, &Self::refreshAction);
        ucsiTimer = IOTimerEventSource::timerEventSource(this, &Self::ucsiTick);
        authTimer = IOTimerEventSource::timerEventSource(this, &Self::authAction);
        if (!loop || !ringLock || !inbox || !request || !refresh || !ucsiTimer || !authTimer || loop->addEventSource(inbox) != kIOReturnSuccess ||
            loop->addEventSource(request) != kIOReturnSuccess || loop->addEventSource(refresh) != kIOReturnSuccess ||
            loop->addEventSource(ucsiTimer) != kIOReturnSuccess || loop->addEventSource(authTimer) != kIOReturnSuccess)
            return false;
        native_battery_stats[0] = 1;
        const ShengAdspClient client = {&Self::dspState, &Self::dspReceive};
        channel = sheng_adsp_channel ? sheng_adsp_channel("PMIC_RTR_ADSP_APPS", 1024, 4, 0, &client, this) : nullptr;
        if (!channel) return false;
        native_battery_stats[0] = 2;
        __atomic_store_n(&gKeyboardInbox, inbox, __ATOMIC_RELEASE);
        inbox->interruptOccurred(nullptr, nullptr, 0);  // replay a cover state received before this service started
        return true;
    }
};
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
OSDefineMetaClassAndStructors(AppleSmartBattery, IOPMPowerSource)
#else
OSDefineMetaClassAndStructors(ShengBattery, IOPMPowerSource)
#endif

// Called by BatteryProbe.cpp once this file's classes are registered.
extern "C" void sheng_battery_start(void) {
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
    AppleSmartBattery *battery = OSTypeAlloc(AppleSmartBattery);
#else
    ShengBattery *battery = OSTypeAlloc(ShengBattery);
#endif
    if (!battery || !battery->init() || !battery->attach(IOService::getPlatform())) {
        if (battery) battery->release();
        native_battery_stats[0] = -1;
        return;
    }
    if (!battery->startBattery()) {
        native_battery_stats[0] = native_battery_stats[0] > 0 ? -native_battery_stats[0] - 1 : -1;
        battery->detach(IOService::getPlatform());
        battery->release();
        return;
    }
    battery->release();  // the platform's attachment keeps it
}

// Nonblocking handoff; PMIC requests stay on the battery work loop and share its one-request queue.
extern "C" void sheng_battery_keyboard_attached(int attached) {
    __atomic_store_n(&gKeyboardAttached, attached ? 1 : 0, __ATOMIC_RELEASE);
    IOInterruptEventSource *event = __atomic_load_n(&gKeyboardInbox, __ATOMIC_ACQUIRE);
    if (event) event->interruptOccurred(nullptr, nullptr, 0);
}
