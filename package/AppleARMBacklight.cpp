// SPDX-License-Identifier: BSD-2-Clause-Patent
// Panel brightness for the sheng board (boot argument backlight=1, started by BacklightProbe.cpp). The panel's backlight is two Kinetic KTZ8866 drivers at I2C
// address 0x11, one on the QUP wrap1 serial engine 1 (i2c@a84000, "ktz8866a") and one on the I2C hub engine 2 (i2c@988000, "ktz8866b");
// both get the same 11-bit level in BL_BRT_LSB (0x04, bits 2:0) and BL_BRT_MSB (0x05), and blank via BL_EN (0x08) bit 6.
// Exponential/linear mapping, calibration and steps follow Pad 8 Pro (ShengBacklightCurve.h / AndroidBrightness / Ktz8866).
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/IOPlatformExpert.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSData.h>
#include <libkern/c++/OSSymbol.h>
#include <pexpert/pexpert.h>
#include "GeniI2c.h"
#include "LidClamshell.h"
#include "ShengBacklightCurve.h"
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#endif

// [0] stage (1 mapped, 2 engines ready, 3 published, 4 published as AppleARMBacklight, negative: the step that failed), [1] brightness requests, [2] last level written,
// [3] level read back from engine a at start, [4] from engine b, [5] transfer errors, [6] last M_IRQ status of a failed transfer,
// [7] engine a protocol and clock configuration (proto << 24 | M_CLK_CFG), [8] the same for engine b, [9] last error (engine << 8 | the
// GeniI2c.h error, or a setup code), [10] interrupts from engine a, [11] from engine b, [12] interrupt mode (0 polling, 1 attached, else
// the attach result), [13] transfer timeouts, [14] attach tries, [15] transfers done
extern "C" {extern volatile int32_t native_backlight_stats[16];}
// The last brightness asked for: [0] level, [1] fade in milliseconds; [2] request timer runs, [3] fades begun from a request; [4] interrupts
// with no status bit set, [5] the last status an interrupt saw.
extern "C" {volatile int32_t native_backlight_request[6];}
extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
// The frames the display shows (GopFramebuffer NativeLegacyDisplay.cpp): [7] copies, [9] flips, [10] latches.
extern "C" volatile uint32_t native_legacy_swapstat[16] __attribute__((weak_import));

class ShengBacklight;

namespace {
constexpr uint64_t kGcc = 0x100000, kSeA = 0xa84000, kSeB = 0x988000;
constexpr uint32_t kSpi[2] = {354, 466};
constexpr uint32_t kGccVote = 0x52008;
// gcc-sm8550.c: wrap1 core_2x 18, core 19, m_ahb 20, s_ahb 21, s1 23; i2c hub s_ahb 7, core 8, s2 12.
constexpr uint32_t kVoteBits = (1u << 18) | (1u << 19) | (1u << 20) | (1u << 21) | (1u << 23) | (1u << 7) | (1u << 8) | (1u << 12);
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#define BACKLIGHT_CONST __attribute__((section("__DATA_CONST,__const")))
#else
#define BACKLIGHT_CONST
#endif
static const uint32_t kHalt[] BACKLIGHT_CONST = {0x23294, 0x23284, 0x2327c, 0x23280, 0x1813c, 0x23140, 0x23144, 0x1703c};
static const char kLogSurvey[] BACKLIGHT_CONST = "sheng backlight: survey a(id=0x%02x cfg1=0x%02x en=0x%02x code=%d) b(id=0x%02x cfg1=0x%02x en=0x%02x code=%d) mode=%s initial=%u\n";
static const char kLogSetProps[] BACKLIGHT_CONST = "sheng backlight: %s -> level %u (%llu ms)\n";
static const char kLogWakeDone[] BACKLIGHT_CONST = "sheng backlight: panel back to its level %llu ms after the wake, %u frames shown%s\n";
static const char kLogLid[] BACKLIGHT_CONST = "sheng backlight: lid %s, panel %s\n";
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
static const char kLogSleep[] BACKLIGHT_CONST = "sheng backlight: system going to sleep, panel dark\n";
static const char kLogAwake[] BACKLIGHT_CONST = "sheng backlight: system awake, panel waits for the display\n";
static const char kLogNoSleep[] BACKLIGHT_CONST = "sheng backlight: no sleep/wake interest\n";
#endif
constexpr uint8_t kAddress = shengbacklight::kAddress;
constexpr uint32_t kMinNits = shengbacklight::kMinNits, kMidNits = shengbacklight::kMidNits, kMaxNits = shengbacklight::kMaxNits;
constexpr uint32_t kFloorCode = shengbacklight::kFloorCode, kMaxLevel = shengbacklight::kFullScale;
constexpr uint32_t kStepMs = 16, kPollMs = 1, kStepDeadlineMs = 25, kAttachMs = 250, kAttachTries = 240;

struct Window {IOMemoryMap *map; volatile uint32_t *va;};
Window gGcc, gSe[2];
volatile uint32_t gIrqStatus[2];
IOInterruptEventSource *volatile gInbox;
ShengBacklight *volatile gBacklight = nullptr;
volatile uint32_t gLastLevel = 0;
volatile uint32_t gExponential = 1;
volatile uint32_t gDirectSetProps = 0;
volatile uint32_t gSwapNitsActive = 0;
volatile uint32_t gSetPropsLogCount = 0;
// The newest brightness asked for: goal << 16 | fade milliseconds, with bit 63 set while the work loop has not taken it.
volatile uint64_t gRequest;
// 1 while the keyboard cover's lid is closed (sheng_backlight_lid): panel is blanked via BL_EN (LidClamshell.h).
volatile uint32_t gLidClosed;
IOTimerEventSource *volatile gLidTimer;
// 1 from the system's will-sleep notification (macOS platform sleep) until the display shows frames again after the wake.
volatile uint32_t gSystemAsleep;
volatile uint32_t gWaking, gWakeFrames;
volatile uint64_t gWakeAt;
IOTimerEventSource *volatile gWakeTimer;

uint32_t shownFrames(bool &counted) {
    counted = &native_legacy_swapstat != nullptr;
    return counted ? native_legacy_swapstat[7] + native_legacy_swapstat[9] + native_legacy_swapstat[10] : 0;
}

bool mapWindow(Window &w, uint64_t base, uint64_t size) {
    IODeviceMemory *memory = IODeviceMemory::withRange(base, size);
    w.map = memory ? memory->map(kIOMapAnywhere | kIOMapInhibitCache) : nullptr;
    if (memory) memory->release();
    w.va = w.map ? reinterpret_cast<volatile uint32_t *>(w.map->getVirtualAddress()) : nullptr;
    return w.va != nullptr;
}
uint32_t rd(volatile uint32_t *base, uint32_t reg) {return base[reg / 4];}
void wr(volatile uint32_t *base, uint32_t reg, uint32_t value) {base[reg / 4] = value; __asm__ volatile("dsb sy" ::: "memory");}
uint32_t ioRead(void *ctx, uint32_t reg) {return rd(static_cast<volatile uint32_t *>(ctx), reg);}
void ioWrite(void *ctx, uint32_t reg, uint32_t value) {wr(static_cast<volatile uint32_t *>(ctx), reg, value);}
void fail(int step, int error) {native_backlight_stats[9] = error; native_backlight_stats[0] = -step;}
void bump(int index) {native_backlight_stats[index] = native_backlight_stats[index] + 1;}

// Interrupt context: read and clear the engine's status, keep it for the work loop and signal it. Nothing else.
void seIrq(void *context) {
    const unsigned index = (unsigned)(uintptr_t)context;
    volatile uint32_t *se = gSe[index].va;
    const uint32_t status = rd(se, GENI_M_IRQ_STATUS);
    if (status & GENI_M_TX_WATERMARK) wr(se, GENI_TX_WATERMARK, 0);
    if (status) wr(se, GENI_M_IRQ_CLEAR, status);
    else native_backlight_request[4] = native_backlight_request[4] + 1;
    native_backlight_request[5] = (int32_t)status;
    __atomic_fetch_or(&gIrqStatus[index], status, __ATOMIC_RELEASE);
    bump(10 + (int)index);
    IOInterruptEventSource *inbox = gInbox;
    if (inbox && status) inbox->interruptOccurred(nullptr, nullptr, 0);
}
}  // namespace

class ShengBacklight : public IOService {
    OSDeclareDefaultStructors(ShengBacklight)
    enum Step : uint8_t {StepNone, StepReadId, StepReadCfg1, StepReadEn, StepReadLsb, StepReadMsb, StepWriteLsb, StepWriteMsb, StepWriteEn};
    struct Engine {
        GeniI2c bus;
        Step step;
        bool started, usable, cfg1Valid, enValid, lit, targetLit;
        uint8_t id, cfg1, foundEn, lsb, enRetries;
        int32_t written;
        uint32_t sending;
        uint64_t deadline;
    };
    Engine engine[2] = {};
    IOWorkLoop *loop = nullptr;
    IOTimerEventSource *fade = nullptr, *tick = nullptr, *attachTimer = nullptr, *request = nullptr, *lid = nullptr, *wake = nullptr;
    IOInterruptEventSource *inbox = nullptr;
    uint32_t level = 0, from = 0, to = 0, targetCode = 0, attachTries = 0;
    bool targetDark = false, exponential = true;
    uint64_t start = 0, period = 0;
    bool irqMode = false, published = false;

    static uint64_t now() {return mach_absolute_time();}
    static uint64_t after(uint32_t ms) {uint64_t delta = 0; nanoseconds_to_absolutetime((uint64_t)ms * NSEC_PER_MSEC, &delta); return now() + delta;}

    static uint32_t clampLit(uint32_t code) {
        return code < kFloorCode ? kFloorCode : code > kMaxLevel ? kMaxLevel : code;
    }

    // Starts the engine's next transfer when it is free.
    void kick(unsigned i) {
        Engine &e = engine[i];
        if (!e.usable || geni_i2c_busy(&e.bus)) return;
        int rc = 0;
        if (!e.started) {
            e.step = StepReadId;
            rc = geni_i2c_start_read(&e.bus, shengbacklight::kRegId);
            if (rc) {e.started = true; e.written = -1; native_backlight_stats[3 + i] = -1;}
        } else if (targetDark) {
            if (e.enValid && e.lit) {
                e.targetLit = false;
                e.step = StepWriteEn;
                rc = geni_i2c_start_write(&e.bus, shengbacklight::kRegEn, shengbacklight::enableValue(e.foundEn, false));
                if (rc) {e.lit = false; e.step = StepNone; return;}
            } else if (!e.enValid && e.written != 0) {
                e.sending = 0;
                e.step = StepWriteLsb;
                rc = geni_i2c_start_write(&e.bus, shengbacklight::kRegBrtLsb, 0);
                if (rc) {e.written = 0; e.step = StepNone; return;}
            } else {
                e.step = StepNone;
                return;
            }
        } else if (e.written != (int32_t)targetCode) {
            e.sending = targetCode;
            e.step = StepWriteLsb;
            rc = geni_i2c_start_write(&e.bus, shengbacklight::kRegBrtLsb, (uint8_t)(targetCode & 7));
            if (rc) {e.written = (int32_t)targetCode; e.step = StepNone; return;}
        } else if (e.enValid && !e.lit) {
            // Write BL_BRT_LSB/MSB before turning BL_EN bit 6 on (Pad 8 Pro Ktz8866.h contract).
            e.targetLit = true;
            e.step = StepWriteEn;
            rc = geni_i2c_start_write(&e.bus, shengbacklight::kRegEn, shengbacklight::enableValue(e.foundEn, true));
            if (rc) {e.lit = true; e.step = StepNone; return;}
        } else {
            e.step = StepNone;
            return;
        }
        if (rc) {e.step = StepNone; return;}
        e.deadline = after(kStepDeadlineMs);
    }

    void noteFailure(unsigned i) {
        const Engine &e = engine[i];
        bump(5);
        native_backlight_stats[6] = (int32_t)e.bus.error_status;
        native_backlight_stats[9] = (int32_t)(i << 8 | e.bus.error);
        native_backlight_stats[13] = (int32_t)(engine[0].bus.timeouts + engine[1].bus.timeouts);
    }

    // A transfer ended: the next step of the engine's sequence.
    void finished(unsigned i, int result) {
        Engine &e = engine[i];
        const bool ok = result == GeniDone;
        if (!ok) noteFailure(i);
        else bump(15);
        int rc = 0;
        switch (e.step) {
        case StepReadId:
            if (ok) e.id = e.bus.data;
            rc = geni_i2c_start_read(&e.bus, shengbacklight::kRegCfg1);
            e.step = StepReadCfg1;
            break;
        case StepReadCfg1:
            if (ok) {e.cfg1 = e.bus.data; e.cfg1Valid = true;}
            rc = geni_i2c_start_read(&e.bus, shengbacklight::kRegEn);
            e.step = StepReadEn;
            break;
        case StepReadEn:
            if (ok) {
                e.foundEn = e.bus.data;
                e.enValid = true;
                e.lit = (e.bus.data & shengbacklight::kEnBacklight) != 0;
            }
            rc = geni_i2c_start_read(&e.bus, shengbacklight::kRegBrtLsb);
            e.step = StepReadLsb;
            break;
        case StepReadLsb:
            if (ok) {
                e.lsb = e.bus.data;
                rc = geni_i2c_start_read(&e.bus, shengbacklight::kRegBrtMsb);
                e.step = StepReadMsb;
                break;
            }
            e.started = true; e.written = -1; native_backlight_stats[3 + i] = -1;
            break;
        case StepReadMsb:
            e.started = true;
            e.written = ok ? (int32_t)(((uint32_t)e.bus.data << 3) | (e.lsb & 7)) : -1;
            native_backlight_stats[3 + i] = e.written;
            break;
        case StepWriteLsb:
            if (ok) {
                rc = geni_i2c_start_write(&e.bus, shengbacklight::kRegBrtMsb, (uint8_t)((e.sending >> 3) & 0xff));
                e.step = StepWriteMsb;
                break;
            }
            e.written = (int32_t)e.sending;
            break;
        case StepWriteMsb:
            e.written = (int32_t)e.sending;
            if (ok) native_backlight_stats[2] = (int32_t)e.sending;
            break;
        case StepWriteEn:
            if (ok) {
                e.lit = e.targetLit;
                e.foundEn = shengbacklight::enableValue(e.foundEn, e.targetLit);
                e.enRetries = 0;
                if (!e.targetLit) native_backlight_stats[2] = 0;
                else native_backlight_stats[2] = (int32_t)targetCode;
            } else if (++e.enRetries >= 2) {
                e.lit = e.targetLit;
                e.enRetries = 0;
            }
            break;
        default:
            break;
        }
        if (rc) {
            noteFailure(i);
            if (!e.started) {e.started = true; e.written = -1; native_backlight_stats[3 + i] = -1;}
        }
        if (geni_i2c_busy(&e.bus)) {e.deadline = after(kStepDeadlineMs); return;}
        e.step = StepNone;
        if (!published && engine[0].started && engine[1].started) publishFirst();
        kick(i);
    }

    void advance(unsigned i, uint32_t status) {
        Engine &e = engine[i];
        if (!geni_i2c_busy(&e.bus)) return;
        const int result = geni_i2c_event(&e.bus, status);
        if (result != GeniPending) finished(i, result);
    }

    void armTick() {
        bool busy = false;
        uint64_t soonest = UINT64_MAX;
        for (const Engine &e : engine) if (geni_i2c_busy(&e.bus)) {busy = true; if (e.deadline < soonest) soonest = e.deadline;}
        if (!busy) {tick->cancelTimeout(); return;}
        if (!irqMode) {tick->setTimeoutMS(kPollMs); return;}
        tick->wakeAtTime((AbsoluteTime)soonest);
    }

    void service() {
        for (unsigned i = 0; i < 2; ++i) {
            uint32_t status = __atomic_exchange_n(&gIrqStatus[i], 0u, __ATOMIC_ACQUIRE);
            if (!irqMode) {
                const uint32_t direct = rd(gSe[i].va, GENI_M_IRQ_STATUS);
                if (direct) wr(gSe[i].va, GENI_M_IRQ_CLEAR, direct);
                status |= direct;
            }
            if (status) advance(i, status);
            Engine &e = engine[i];
            if (geni_i2c_busy(&e.bus) && now() >= e.deadline) {
                const int result = geni_i2c_timeout(&e.bus);
                native_backlight_stats[13] = (int32_t)(engine[0].bus.timeouts + engine[1].bus.timeouts);
                if (result != GeniPending) finished(i, result);
                else e.deadline = after(kStepDeadlineMs);
            }
        }
        armTick();
    }

    static void inboxFired(OSObject *owner, IOInterruptEventSource *, int) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self) return;
        self->service();
    }
    static void requestFired(OSObject *owner, IOTimerEventSource *) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self) return;
        native_backlight_request[2] = native_backlight_request[2] + 1;
        const uint64_t req = __atomic_exchange_n(&gRequest, 0ull, __ATOMIC_ACQUIRE);
        if (req >> 63) {
            native_backlight_request[3] = native_backlight_request[3] + 1;
            self->beginFade((uint32_t)(req >> 16) & 0xffff, (uint32_t)req & 0xffff);
        }
    }
    static void tickFired(OSObject *owner, IOTimerEventSource *) {
        if (ShengBacklight *self = static_cast<ShengBacklight *>(owner)) self->service();
    }
    static void lidFired(OSObject *owner, IOTimerEventSource *) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self || !self->published) return;
        self->setLevel(self->level);
    }
    static void wakeFired(OSObject *owner, IOTimerEventSource *sender) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self || !__atomic_load_n(&gWaking, __ATOMIC_ACQUIRE)) return;
        bool counted = false;
        const uint32_t frames = shownFrames(counted);
        uint64_t ns = 0;
        absolutetime_to_nanoseconds(now() - gWakeAt, &ns);
        const uint64_t ms = ns / NSEC_PER_MSEC;
        if (!lid_backlight_wake_ready(counted, gWakeFrames, frames, ms)) {sender->setTimeoutMS(20); return;}
        __atomic_store_n(&gWaking, 0u, __ATOMIC_RELEASE);
        __atomic_store_n(&gSystemAsleep, 0u, __ATOMIC_RELEASE);
        IOLog(kLogWakeDone, ms, frames - gWakeFrames,
              counted ? "" : " (no frame count)");
        if (self->published) self->setLevel(self->level);
    }
    static void attachFired(OSObject *owner, IOTimerEventSource *sender) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self || self->irqMode) return;
        if (!armcpu_spi_attach) {native_backlight_stats[12] = -1; return;}
        native_backlight_stats[14] = (int32_t)++self->attachTries;
        int rc = armcpu_spi_attach(32 + kSpi[0], false, seIrq, (void *)(uintptr_t)0, nullptr, nullptr);
        if (rc == kIOReturnNotReady && self->attachTries < kAttachTries) {sender->setTimeoutMS(kAttachMs); return;}
        if (!rc) rc = armcpu_spi_attach(32 + kSpi[1], false, seIrq, (void *)(uintptr_t)1, nullptr, nullptr);
        if (rc) {native_backlight_stats[12] = rc; return;}
        self->irqMode = true;
        native_backlight_stats[12] = 1;
        self->service();
    }

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
    static IOReturn sleepWake(void *, void *, UInt32 message, IOService *, void *, vm_size_t) {
        if (message != kIOMessageSystemWillSleep && message != kIOMessageSystemHasPoweredOn) return kIOReturnSuccess;
        if (message == kIOMessageSystemWillSleep) {
            __atomic_store_n(&gWaking, 0u, __ATOMIC_RELEASE);
            __atomic_store_n(&gSystemAsleep, 1u, __ATOMIC_RELEASE);
            IOLog(kLogSleep);
            if (IOTimerEventSource *timer = gLidTimer) timer->setTimeoutUS(1);
            return kIOReturnSuccess;
        }
        bool counted = false;
        gWakeFrames = shownFrames(counted);
        gWakeAt = now();
        __atomic_store_n(&gWaking, 1u, __ATOMIC_RELEASE);
        IOLog(kLogAwake);
        if (IOTimerEventSource *timer = gWakeTimer) timer->setTimeoutMS(20);
        else {__atomic_store_n(&gWaking, 0u, __ATOMIC_RELEASE); __atomic_store_n(&gSystemAsleep, 0u, __ATOMIC_RELEASE);
              if (IOTimerEventSource *lidTimer = gLidTimer) lidTimer->setTimeoutUS(1);}
        return kIOReturnSuccess;
    }
#endif

    void setLevel(uint32_t value) {
        const bool hwDark = lid_backlight_dark(__atomic_load_n(&gLidClosed, __ATOMIC_ACQUIRE) != 0,
                                               __atomic_load_n(&gSystemAsleep, __ATOMIC_ACQUIRE) != 0);
        targetDark = hwDark || value == 0;
        if (value > 0) targetCode = clampLit(value);
        engine[0].enRetries = engine[1].enRetries = 0;
        kick(0); kick(1);
        armTick();
    }

    static void fadeStep(OSObject *owner, IOTimerEventSource *sender) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self) return;
        const uint64_t elapsed = now() - self->start;
        uint32_t next = self->to;
        if (elapsed < self->period) {
            const uint32_t interp = (uint32_t)((int64_t)self->from + ((int64_t)self->to - (int64_t)self->from) * (int64_t)elapsed / (int64_t)self->period);
            next = clampLit(interp);
        } else if (self->to > 0) {
            next = clampLit(self->to);
        }
        if (next != self->level) {self->setLevel(next); self->level = next;}
        if (next != self->to) sender->setTimeoutMS(kStepMs);
    }

    void beginFade(uint32_t goal, uint32_t milliseconds) {
        fade->cancelTimeout();
        from = level;
        to = goal;
        start = now();
        nanoseconds_to_absolutetime((uint64_t)milliseconds * NSEC_PER_MSEC, &period);
        fadeStep(this, fade);
    }

    static IOReturn begin(OSObject *owner, void *, void *, void *, void *) {
        ShengBacklight *self = static_cast<ShengBacklight *>(owner);
        if (!self) return kIOReturnBadArgument;
        self->kick(0); self->kick(1);
        if (!self->published && self->engine[0].started && self->engine[1].started) self->publishFirst();
        self->armTick();
        self->attachTimer->setTimeoutMS(1);
        return kIOReturnSuccess;
    }

    void publish(uint32_t value) {
        const uint32_t reportedCode = value == 0 ? 0 : clampLit(value);
        const uint32_t currentNitsMilli = shengbacklight::milliNitsForCode((uint16_t)reportedCode, exponential);
        OSDictionary *brightness = OSDictionary::withCapacity(3), *milliNits = OSDictionary::withCapacity(3), *parameters = OSDictionary::withCapacity(2);
        OSNumber *min = OSNumber::withNumber((uint64_t)kFloorCode, 32), *max = OSNumber::withNumber(kMaxLevel, 32), *current = OSNumber::withNumber(reportedCode, 32);
        OSNumber *minMilli = OSNumber::withNumber(kMinNits * 1000ULL, 32), *maxMilli = OSNumber::withNumber(kMaxNits * 1000ULL, 32);
        OSNumber *currentMilli = OSNumber::withNumber(currentNitsMilli, 32);
        if (brightness && milliNits && parameters && min && max && current && minMilli && maxMilli && currentMilli) {
            brightness->setObject("min", min); brightness->setObject("max", max); brightness->setObject("value", current);
            milliNits->setObject("min", minMilli); milliNits->setObject("max", maxMilli); milliNits->setObject("value", currentMilli);
            parameters->setObject("brightness", brightness);
            parameters->setObject("BrightnessMilliNits", milliNits);
            setProperty("IODisplayParameters", parameters);
        }
        OSSafeReleaseNULL(min); OSSafeReleaseNULL(max); OSSafeReleaseNULL(current); OSSafeReleaseNULL(minMilli); OSSafeReleaseNULL(maxMilli);
        OSSafeReleaseNULL(currentMilli); OSSafeReleaseNULL(brightness); OSSafeReleaseNULL(milliNits); OSSafeReleaseNULL(parameters);
    }

    void publishCalibration() {
        const int32_t table[] = {(int32_t)(kMinNits << 16), (int32_t)(kMidNits << 16), (int32_t)(kMaxNits << 16)};
        if (OSData *marketing = OSData::withBytes(table, sizeof table)) {
            setProperty("backlight-marketing-table", marketing);
            marketing->release();
        }
        if (OSData *value = OSData::withBytes(&table[2], sizeof table[2])) {
            setProperty("LmaxProduct", value);
            value->release();
        }
        const struct {const char *key; uint32_t nits;} products[] = {{"LminProduct", kMinNits}, {"LmidProduct", kMidNits}};
        for (const auto &product : products) {
            if (OSNumber *value = OSNumber::withNumber(product.nits, 32)) {
                setProperty(product.key, value);
                value->release();
            }
        }
        OSDictionary *calibration = OSDictionary::withCapacity(6);
        if (!calibration) return;
        const struct {const char *key; uint32_t nits;} fields[] = {
            {"lMinProduct", kMinNits}, {"lMidProduct", kMidNits}, {"lMaxProduct", kMaxNits},
            {"lMinPanel", kMinNits}, {"lMidPanel", kMidNits}, {"lMaxPanel", kMaxNits}
        };
        for (const auto &field : fields) {
            if (OSNumber *number = OSNumber::withNumber((uint64_t)field.nits << 16, 32)) {
                calibration->setObject(field.key, number);
                number->release();
            }
        }
        setProperty("backlight-calibration-parameters", calibration);
        calibration->release();
    }

    static inline const OSMetaClass *gAppleClass = nullptr;
    static inline bool gImpersonating = false;
    static const OSMetaClass *appleMetaClass(const OSMetaClassBase *) {return gAppleClass;}
    bool impersonate() {
        enum {kSlots = 320, kGetMetaClass = 7};
        const OSSymbol *name = OSSymbol::withCString("AppleARMBacklight");
        gAppleClass = name ? OSMetaClass::getMetaClassWithName(name) : nullptr;
        OSSafeReleaseNULL(name);
        if (!gAppleClass) return false;
#if __has_feature(ptrauth_calls)
        __atomic_store_n(&gImpersonating, true, __ATOMIC_RELEASE);
        return true;
#endif
        uintptr_t *copy = static_cast<uintptr_t *>(IOMalloc(kSlots * sizeof(uintptr_t)));
        if (!copy) return false;
        memcpy(copy, *reinterpret_cast<uintptr_t **>(this), kSlots * sizeof(uintptr_t));
        copy[kGetMetaClass] = reinterpret_cast<uintptr_t>(&appleMetaClass);
        __asm__ volatile("dmb ish" ::: "memory");
        *reinterpret_cast<uintptr_t **>(this) = copy;
        return true;
    }

    void publishFirst() {
        published = true;
        const Engine &a = engine[0], &b = engine[1];
        if (a.cfg1Valid) exponential = shengbacklight::isExponential(a.cfg1);
        else if (b.cfg1Valid) exponential = shengbacklight::isExponential(b.cfg1);
        else exponential = true;
        __atomic_store_n(&gExponential, exponential ? 1u : 0u, __ATOMIC_RELEASE);
        const int32_t read = native_backlight_stats[3] > 0 ? native_backlight_stats[3] : native_backlight_stats[4];
        const uint32_t initial = read > 0 ? clampLit((uint32_t)read) : shengbacklight::codeForNits(shengbacklight::kMidFixed, exponential);
        level = to = targetCode = initial;
        targetDark = false;
        IOLog(kLogSurvey,
              a.id, a.cfg1, a.foundEn, native_backlight_stats[3],
              b.id, b.cfg1, b.foundEn, native_backlight_stats[4],
              exponential ? "exponential" : "linear", initial);
        uint32_t control = 1;
        PE_parse_boot_argn("backlight-control", &control, sizeof control);
        if (control == 1 && impersonate()) {
            static const uint8_t kForceOverride = 1;
            if (OSData *force = OSData::withBytes(&kForceOverride, sizeof kForceOverride)) {
                setProperty("force-brightness-control", force);
                force->release();
            }
            setProperty("NativeSandboxGrantProcess", "backboardd,WindowServer,corebrightnessd");
            setProperty("backlight-control", true);
        }
        const bool impersonated = control == 1 && gAppleClass;
        publishCalibration();
        publish(level);
        __atomic_store_n(&gLastLevel, level, __ATOMIC_RELEASE);
        __atomic_store_n(&gBacklight, this, __ATOMIC_RELEASE);
        registerService();
        native_backlight_stats[0] = impersonated ? 4 : 3;
        if (__atomic_load_n(&gLidClosed, __ATOMIC_ACQUIRE)) setLevel(level);
    }

public:
    bool startBacklight(const bool usable[2]) {
        loop = IOWorkLoop::workLoop();
        fade = IOTimerEventSource::timerEventSource(this, &ShengBacklight::fadeStep);
        tick = IOTimerEventSource::timerEventSource(this, &ShengBacklight::tickFired);
        attachTimer = IOTimerEventSource::timerEventSource(this, &ShengBacklight::attachFired);
        inbox = IOInterruptEventSource::interruptEventSource(this, &ShengBacklight::inboxFired);
        request = IOTimerEventSource::timerEventSource(this, &ShengBacklight::requestFired);
        lid = IOTimerEventSource::timerEventSource(this, &ShengBacklight::lidFired);
        wake = IOTimerEventSource::timerEventSource(this, &ShengBacklight::wakeFired);
        if (!loop || !fade || !tick || !attachTimer || !inbox || !request || !lid || !wake) return false;
        if (loop->addEventSource(fade) || loop->addEventSource(tick) || loop->addEventSource(attachTimer) || loop->addEventSource(inbox) ||
            loop->addEventSource(request) || loop->addEventSource(lid) || loop->addEventSource(wake)) return false;
        gLidTimer = lid;
        gWakeTimer = wake;
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
        if (!registerSleepWakeInterest(&sleepWake, nullptr, nullptr)) IOLog(kLogNoSleep);
#endif
        for (unsigned i = 0; i < 2; ++i) {
            const GeniIo io = {ioRead, ioWrite, (void *)gSe[i].va};
            uint32_t config = 0;
            engine[i].usable = usable[i] && geni_i2c_prepare(&engine[i].bus, &io, kAddress, &config) == 0;
            native_backlight_stats[7 + i] = (int32_t)config;
            engine[i].started = !engine[i].usable;
            engine[i].written = -1;
            engine[i].lit = true;
            engine[i].foundEn = 0x7f;
            if (!engine[i].usable) native_backlight_stats[3 + i] = -1;
        }
        if (!engine[0].usable && !engine[1].usable) return false;
        gInbox = inbox;
        native_backlight_stats[0] = 2;
        loop->runAction(&ShengBacklight::begin, this);
        return true;
    }

    static uint32_t levelForNits(int32_t fixed) {
        return shengbacklight::codeForNits(fixed, __atomic_load_n(&gExponential, __ATOMIC_ACQUIRE) != 0);
    }

    static uint32_t steppedLevel(uint32_t current, int delta) {
        shengbacklight::Steps steps;
        steps.build(kFloorCode, __atomic_load_n(&gExponential, __ATOMIC_ACQUIRE) != 0, kMaxLevel);
        uint16_t code = (uint16_t)clampLit(current);
        const int limited = delta < -32 ? -32 : delta > 32 ? 32 : delta;
        for (int i = 0; i < limited; ++i) code = steps.up(code);
        for (int i = 0; i > limited; --i) code = steps.down(code);
        return code;
    }

    void queueLevel(uint32_t goal, uint32_t milliseconds) {
        if (goal > kMaxLevel) goal = kMaxLevel;
        if (goal > 0 && goal < kFloorCode) goal = kFloorCode;
        if (milliseconds > 5000) milliseconds = 5000;
        __atomic_store_n(&gLastLevel, goal, __ATOMIC_RELEASE);
        bump(1);
        native_backlight_request[0] = (int32_t)goal;
        native_backlight_request[1] = (int32_t)milliseconds;
        __atomic_store_n(&gRequest, 1ull << 63 | (uint64_t)goal << 16 | milliseconds, __ATOMIC_RELEASE);
        request->setTimeoutUS(1);
        publish(goal);
    }

    IOReturn setProperties(OSObject *properties) override {
        OSDictionary *dictionary = OSDynamicCast(OSDictionary, properties);
        if (!dictionary) return kIOReturnBadArgument;
        uint32_t goal;
        uint64_t milliseconds = 0;
        const char *source = nullptr;
        if (OSNumber *nits = OSDynamicCast(OSNumber, dictionary->getObject("brightness-nits"))) {
            goal = levelForNits((int32_t)nits->unsigned32BitValue());
            source = "brightness-nits";
            __atomic_store_n(&gDirectSetProps, 1u, __ATOMIC_RELEASE);
        } else if (OSNumber *brightness = OSDynamicCast(OSNumber, dictionary->getObject("brightness"))) {
            OSNumber *fadePeriod = OSDynamicCast(OSNumber, dictionary->getObject("FadePeriod"));
            const int32_t requested = (int32_t)brightness->unsigned32BitValue();
            goal = requested <= 0 ? 0 : clampLit((uint32_t)requested);
            milliseconds = fadePeriod ? ((uint64_t)fadePeriod->unsigned32BitValue() * 1000) >> 16 : 0;
            source = "brightness";
            __atomic_store_n(&gDirectSetProps, 1u, __ATOMIC_RELEASE);
        } else if (OSNumber *step = OSDynamicCast(OSNumber, dictionary->getObject("brightness-step"))) {
            const int32_t delta = (int32_t)step->unsigned32BitValue();
            if (!delta) return kIOReturnSuccess;
            goal = steppedLevel(__atomic_load_n(&gLastLevel, __ATOMIC_ACQUIRE), delta);
            milliseconds = 60;
            source = "brightness-step";
        } else {
            return kIOReturnSuccess;
        }
        const uint32_t logged = __atomic_fetch_add(&gSetPropsLogCount, 1u, __ATOMIC_RELAXED);
        if (logged < 8) IOLog(kLogSetProps, source, goal, (unsigned long long)milliseconds);
        queueLevel(goal, (uint32_t)milliseconds);
        return kIOReturnSuccess;
    }
};
#if __has_feature(ptrauth_calls)
ShengBacklight::MetaClass ShengBacklight::gMetaClass;
const OSMetaClass * const ShengBacklight::metaClass = &ShengBacklight::gMetaClass;
const OSMetaClass * const ShengBacklight::superClass = &IOService::gMetaClass;
ShengBacklight::ShengBacklight(const OSMetaClass *meta) : IOService(meta) {}
ShengBacklight::~ShengBacklight() {}
const OSMetaClass *ShengBacklight::getMetaClass() const
{
    return __atomic_load_n(&gImpersonating, __ATOMIC_ACQUIRE) && gAppleClass ? gAppleClass : &gMetaClass;
}
OSMetaClassConstructorInit(ShengBacklight, IOService, )
OSDefineDefaultStructors(ShengBacklight, IOService)
#else
OSDefineMetaClassAndStructors(ShengBacklight, IOService)
#endif

extern "C" void sheng_backlight_lid(bool closed) {
    __atomic_store_n(&gLidClosed, closed ? 1u : 0u, __ATOMIC_RELEASE);
    IOLog(kLogLid, closed ? "closed" : "open", closed ? "dark" : "back to its level");
    if (IOTimerEventSource *timer = gLidTimer) timer->setTimeoutUS(1);
}

extern "C" void sheng_backlight_set_nits(int32_t fixed) {
    if (fixed <= 0 || __atomic_load_n(&gDirectSetProps, __ATOMIC_ACQUIRE)) return;
    ShengBacklight *bl = __atomic_load_n(&gBacklight, __ATOMIC_ACQUIRE);
    if (!bl) return;
    __atomic_store_n(&gSwapNitsActive, 1u, __ATOMIC_RELEASE);
    const uint32_t goal = ShengBacklight::levelForNits(fixed);
    if (goal == __atomic_load_n(&gLastLevel, __ATOMIC_ACQUIRE)) return;
    bl->queueLevel(goal, 0);
}

extern "C" void sheng_backlight_step(int delta) {
    if (!delta || __atomic_load_n(&gDirectSetProps, __ATOMIC_ACQUIRE) || __atomic_load_n(&gSwapNitsActive, __ATOMIC_ACQUIRE)) return;
    ShengBacklight *bl = __atomic_load_n(&gBacklight, __ATOMIC_ACQUIRE);
    if (!bl) return;
    const uint32_t cur = __atomic_load_n(&gLastLevel, __ATOMIC_ACQUIRE);
    bl->queueLevel(ShengBacklight::steppedLevel(cur, delta), 60);
}

extern "C" void sheng_backlight_start(void) {
    if (!mapWindow(gGcc, kGcc, 0x60000) || !mapWindow(gSe[0], kSeA, 0x4000) || !mapWindow(gSe[1], kSeB, 0x4000)) {fail(1, -12); return;}
    native_backlight_stats[0] = 1;
    wr(gGcc.va, kGccVote, rd(gGcc.va, kGccVote) | kVoteBits);
    for (uint32_t halt : kHalt) {
        if (rd(gGcc.va, halt) & (1u << 1)) continue;
        unsigned us = 0;
        while ((rd(gGcc.va, halt) & (1u << 31)) && us < 200) {IODelay(1); ++us;}
        if (us == 200) {fail(2, (int)halt); return;}
    }
    const bool usable[2] = {true, true};
    ShengBacklight *backlight = OSTypeAlloc(ShengBacklight);
    if (!backlight || !backlight->init() || !backlight->attach(IOService::getPlatform()) || !backlight->startBacklight(usable)) {
        if (backlight) backlight->release();
        fail(3, -19); return;
    }
    backlight->release();
}
