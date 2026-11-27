// SPDX-License-Identifier: BSD-2-Clause-Patent
// The sheng keyboard cover (boot argument keyboard=1): keyboard and touchpad behind the Nanosic WN8030 controller on I2C9 (QUP2 SE1,
// 0x884000, client 0x4c), using Pad 8 Pro (PianoWn8030.h) serving/poll/keys/backlight/gesture policy over sheng's GeniI2c bus.
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/IODeviceMemory.h>
#include <IOKit/IOService.h>
#include <libkern/c++/OSSharedPtr.h>
#include <libkern/c++/OSMetaClass.h>
#include <kern/thread.h>
#include <kern/clock.h>
#include <pexpert/pexpert.h>
#include <mach/kmod.h>
#include <string.h>
#include "AppsSmmuLock.h"
#include "KeyboardService.h"
#include "PianoWn8030.h"
extern "C" {
#include "GeniI2c.h"
#include "Wn8030.h"
#include "QcomRpmh.h"
#include "TouchBoard.h"
#include "KeyboardTrace.h"
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#include <IOKit/pwr_mgt/RootDomain.h>
#include "LidClamshell.h"
#endif
}
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#include <IOKit/IOMessage.h>
#endif

extern kmod_info_t g_kernel_kmod_info;
extern "C" kern_return_t OSKextLoadKextWithIdentifier(const char *);
extern "C" void sheng_keyboard_cover_static_init(void) __asm__("__GLOBAL__sub_I_KeyboardCover.cpp");
extern "C" IOService *sheng_keyboard_cover_allocate(void);
extern "C" void sheng_keyboard_cover_input(IOService *, const uint8_t *, uint32_t);
extern "C" void sheng_keyboard_cover_enable(IOService *, bool);
extern "C" void sheng_keyboard_cover_configure(IOService *, bool gestures, bool keepLone);
extern "C" void sheng_keyboard_cover_key_typed(IOService *, uint64_t ms);
extern "C" uint8_t sheng_keyboard_cover_take_swipe(IOService *);
extern "C" void sheng_keyboard_cover_output_sink(void (*)(void *, uint8_t), void *);
extern "C" void sheng_battery_keyboard_attached(int) __attribute__((weak_import));
extern "C" void sheng_backlight_step(int) __attribute__((weak_import));
extern "C" int armcpu_spi_attach(uint32_t, bool, void (*)(void *), void *, uint32_t *, uint64_t *) __attribute__((weak_import));
extern "C" int armcpu_spi_share(uint32_t, void (*)(void *), void *) __attribute__((weak_import));
extern "C" bool sm8550_gcc_update32(volatile uint32_t *, uint32_t, uint32_t) __attribute__((weak_import));
extern "C" const uint8_t native_keyboard_firmware[];
extern "C" const uint32_t native_keyboard_firmware_size;

extern "C" {volatile int32_t native_keyboard_stats[17];}
extern "C" {volatile uint32_t native_keyboard_lid_closed;}
extern "C" {volatile int32_t native_keyboard_accel[4];}
extern "C" {volatile int32_t native_keyboard_touch_range[4] = {-1, -1, -1, -1};}

namespace {
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
#define KEYBOARD_CONST __attribute__((section("__DATA_CONST,__const")))
#else
#define KEYBOARD_CONST
#endif
static const char kLidLog[] KEYBOARD_CONST = "sheng keyboard: lid %s\n";
static const char kClamshellLog[] KEYBOARD_CONST = "sheng keyboard: clamshell %s sent to the power manager\n";
static const char kCoverLog[] KEYBOARD_CONST = "sheng keyboard: cover %s; frame %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x; before it %s, ms since a frame %d, "
          "input %d, accelerometer %d; frames %d, controller interrupts %d, i2c failures %d, last i2c error 0x%x, auth %d, attaches %d, "
          "reloads %u\n";
static const char kKeepAwakeFailureLog[] KEYBOARD_CONST = "sheng keyboard: keep-awake command failed, waking the controller (i2c failures %d, last i2c error 0x%x)\n";
static const char kControllerReloadLog[] KEYBOARD_CONST = "sheng keyboard: controller reload %u (read failures %u, command failures %u, i2c failures %d, last i2c error 0x%x, "
              "frames %d)\n";
static const char kControllerReloadResultLog[] KEYBOARD_CONST = "sheng keyboard: controller reload %u %s (stage %d, detail %d)\n";
#undef KEYBOARD_CONST

constexpr uint64_t kGcc = 0x00100000ULL, kTlmm = 0x0f100000ULL, kSe = 0x00884000ULL, kRsc = 0x17a20000ULL, kCmdDb = 0x81c60000ULL;
constexpr uint32_t kCmdDbSize = 0x20000, kRscTcs = 0xd00, kTcs = 2;
constexpr uint32_t kVote = 0x52010, kVoteBits = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 3) | (1u << 5);
constexpr uint32_t kHalts[] = {0x233d4, 0x233d0, 0x233cc, 0x23004, 0x1e13c};
constexpr uint32_t kRcg = 0x1e148;
constexpr uint32_t kSeIntid = 32 + 583, kTlmmIntid = 32 + 208;
constexpr unsigned kStatusPin = 56, kSleepPin = 57, kResetPin = 58, kSdaPin = 60, kSclPin = 61, kIrqPin = 62, kLidPin = 171, kTabletPin = 172;
constexpr uint32_t kDebounceMs = 15, kHallPollMs = 1000;
constexpr uint32_t kCtl = 0x0, kIo = 0x4, kIntrCfg = 0x8, kIntrStatus = 0xc;

struct Region {OSSharedPtr<IOMemoryMap> keep; uintptr_t va = 0;};
bool mapRegion(uint64_t phys, uint64_t length, Region &region) {
    IODeviceMemory *memory = IODeviceMemory::withRange(phys, length);
    if (!memory) return false;
    region.keep = OSSharedPtr<IOMemoryMap>(memory->map(kIOMapAnywhere | kIOMapInhibitCache), OSNoRetain);
    memory->release();
    region.va = region.keep ? static_cast<uintptr_t>(region.keep->getVirtualAddress()) : 0;
    return region.va != 0;
}
uint32_t rd32(uintptr_t base, uint32_t reg) {return *reinterpret_cast<volatile uint32_t *>(base + reg);}
void wr32(uintptr_t base, uint32_t reg, uint32_t value) {
    *reinterpret_cast<volatile uint32_t *>(base + reg) = value;
    __asm__ volatile("dsb sy" ::: "memory");
}
uint32_t ioRd(void *ctx, uint32_t reg) {return rd32(reinterpret_cast<uintptr_t>(ctx), reg);}
void ioWr(void *ctx, uint32_t reg, uint32_t value) {wr32(reinterpret_cast<uintptr_t>(ctx), reg, value);}
uint64_t nowMs() {uint64_t ns; absolutetime_to_nanoseconds(mach_absolute_time(), &ns); return ns / 1000000;}
uint64_t deadlineMs(uint32_t ms) {uint64_t d; clock_interval_to_deadline(ms, kMillisecondScale, &d); return d;}
void stage(int step) {native_keyboard_stats[0] = step;}
bool fail(int step, int32_t detail) {native_keyboard_stats[1] = detail; native_keyboard_stats[0] = -step; return false;}
void bump(unsigned index, int32_t by = 1) {native_keyboard_stats[index] = native_keyboard_stats[index] + by;}

struct Keyboard {
    Region gccLow, gcc1e, gcc23, gccVote, se;
    Region pin[8];
    GeniI2c bus;
    IOSimpleLock *busLock;
    uint32_t khz;
    uint32_t backlightLevel, backlightIdleMs;
    bool gesturesEnabled, fnRowEnabled;
    volatile int busResult;
    volatile uint32_t mcuIrqs, hallIrqs;
    volatile uint32_t capsWanted, capsRequests;
    volatile uint32_t wakeRequested;
    uint32_t hallSeen, capsSeen;
    uint32_t hidRetries;
    bool attached, inputEnabled, lid, tablet, caps, hallPending, detachPending, sleepRegistered;
    uint8_t rctrlBrightness, rctrlKbdBl;
    uint64_t hallDue, hallPoll, detachDueMs, hidRetryDue;
    KeyboardService service;
    pianokeyboard::Keys keys;
    pianokeyboard::Backlight kbdBacklight;
    pianokeyboard::PollPlan pollPlan;
    IOService *hid;
    bool classRegistered;
    uint8_t frame[WN8030_FRAME_BYTES];
    uint8_t command[1 + WN8030_DATA_COMMAND_BYTES];
    kb_trace trace;
    uint32_t reloads;
};
Keyboard *gKb;
volatile uint32_t gWake;
volatile uint32_t gBusWake;

enum PinIndex {PinStatus, PinSleep, PinReset, PinSda, PinScl, PinIrq, PinLid, PinTablet};
constexpr unsigned kPins[] = {kStatusPin, kSleepPin, kResetPin, kSdaPin, kSclPin, kIrqPin, kLidPin, kTabletPin};
constexpr PinIndex kInterruptPins[] = {PinIrq, PinLid, PinTablet}, kHallPins[] = {PinLid, PinTablet}, kBusPins[] = {PinSda, PinScl};
uintptr_t pin(PinIndex i) {return gKb->pin[i].va;}
void setOutput(PinIndex i, bool high) {wr32(pin(i), kIo, high ? 2u : 0u);}
bool level(PinIndex i) {return rd32(pin(i), kIo) & 1u;}

void busIrq(void *) {
    Keyboard *kb = gKb;
    IOSimpleLockLock(kb->busLock);
    const int result = geni_i2c_poll(&kb->bus);
    if (result != GeniPending) __atomic_store_n(&kb->busResult, result, __ATOMIC_RELEASE);
    IOSimpleLockUnlock(kb->busLock);
    bump(12);
    if (result != GeniPending) thread_wakeup((event_t)&gBusWake);
}

void tlmmIrq(void *) {
    Keyboard *kb = gKb;
    bool wake = false;
    if (rd32(pin(PinIrq), kIntrStatus) & 1u) {
        wr32(pin(PinIrq), kIntrStatus, 0);
        __atomic_fetch_add(&kb->mcuIrqs, 1, __ATOMIC_RELEASE);
        wake = true;
    }
    for (PinIndex i : kHallPins) {
        if (!(rd32(pin(i), kIntrStatus) & 1u)) continue;
        wr32(pin(i), kIntrStatus, 0);
        __atomic_fetch_add(&kb->hallIrqs, 1, __ATOMIC_RELEASE);
        wake = true;
    }
    if (wake) thread_wakeup((event_t)&gWake);
}

template <typename Start> int transfer(Start start, uint32_t bytes) {
    Keyboard *kb = gKb;
    const uint32_t ms = 10 + bytes * 9 * 2 / kb->khz;
    IOInterruptState state = IOSimpleLockLockDisableInterrupt(kb->busLock);
    kb->busResult = GeniPending;
    const int started = start(&kb->bus);
    IOSimpleLockUnlockEnableInterrupt(kb->busLock, state);
    if (started) {bump(11); return GeniFailed;}
    int result = GeniPending;
    for (unsigned step = 0; step < 4 && result == GeniPending; ++step) {
        const uint64_t deadline = deadlineMs(ms);
        wait_result_t waited = THREAD_AWAKENED;
        while ((result = __atomic_load_n(&kb->busResult, __ATOMIC_ACQUIRE)) == GeniPending && waited != THREAD_TIMED_OUT) {
            assert_wait_deadline((event_t)&gBusWake, THREAD_UNINT, deadline);
            if ((result = __atomic_load_n(&kb->busResult, __ATOMIC_ACQUIRE)) != GeniPending) {clear_wait(current_thread(), THREAD_AWAKENED); break;}
            waited = thread_block(THREAD_CONTINUE_NULL);
        }
        if (result != GeniPending) break;
        state = IOSimpleLockLockDisableInterrupt(kb->busLock);
        result = __atomic_load_n(&kb->busResult, __ATOMIC_ACQUIRE);
        if (result == GeniPending) result = geni_i2c_timeout(&kb->bus);
        IOSimpleLockUnlockEnableInterrupt(kb->busLock, state);
    }
    if (result != GeniDone) {
        bump(11);
        native_keyboard_stats[13] = (int32_t)(kb->bus.error | (kb->bus.error_status << 8));
    }
    return result == GeniPending ? GeniFailed : result;
}

bool romWrite(uint8_t value) {
    return transfer([value](GeniI2c *g) {return geni_i2c_start_write(g, WN8030_REG_WRITE, value);}, 2) == GeniDone;
}
int romRead() {
    if (transfer([](GeniI2c *g) {return geni_i2c_start_read(g, WN8030_REG_WRITE);}, 2) != GeniDone) return -1;
    return gKb->bus.data;
}
int romState() {return romWrite(WN8030_ROM_STATE) ? romRead() : -1;}
bool writeBuffer(const uint8_t *bytes, uint32_t length) {
    return transfer([bytes, length](GeniI2c *g) {return geni_i2c_start_write_buffer(g, bytes, length);}, length) == GeniDone;
}
bool readFrame() {
    static const uint8_t reg = WN8030_REG_READ;
    uint8_t *frame = gKb->frame;
    return transfer([frame](GeniI2c *g) {return geni_i2c_start_read_buffer(g, &reg, 1, frame, WN8030_FRAME_BYTES);},
                    1 + WN8030_FRAME_BYTES) == GeniDone;
}
bool control(uint8_t command, uint8_t value) {
    uint8_t out[1 + WN8030_COMMAND_BYTES];
    const uint32_t n = wn8030_control(out, command, &value, 1);
    const bool ok = n && writeBuffer(out, n);
    if (ok) bump(14);
    return ok;
}

void applyBacklight(int level) {
    Keyboard *kb = gKb;
    if (level < 0 || !kb->attached || !kb->inputEnabled) return;
    if (control(WN8030_CMD_BACKLIGHT, static_cast<uint8_t>(level))) {
        IOLog("sheng keyboard: backlight -> %d\n", level);
    }
}

bool setupPins() {
    for (unsigned i = 0; i < sizeof kPins / sizeof *kPins; ++i)
        if (!mapRegion(kTlmm + tb_tlmm_ctl(kPins[i]), 0x1000, gKb->pin[i])) return fail(1, (int32_t)kPins[i]);
    for (PinIndex i : kInterruptPins) wr32(pin(i), kIntrCfg, rd32(pin(i), kIntrCfg) & ~1u);
    setOutput(PinReset, false); wr32(pin(PinReset), kCtl, 1u << 9);
    setOutput(PinSleep, false); wr32(pin(PinSleep), kCtl, 1u << 9 | 1u);
    wr32(pin(PinStatus), kCtl, 0);
    wr32(pin(PinIrq), kCtl, rd32(pin(PinIrq), kCtl) & 3u);
    for (PinIndex i : kBusPins) wr32(pin(i), kCtl, 1u << 2);
    for (PinIndex i : kHallPins) wr32(pin(i), kCtl, 3u);
    return true;
}

bool setupRails() {
    struct CmdDbCopy {uint8_t *bytes = static_cast<uint8_t *>(IOMallocData(kCmdDbSize)); ~CmdDbCopy() {if (bytes) IOFreeData(bytes, kCmdDbSize);}} copy;
    if (!copy.bytes) return fail(2, -12);
    Region db, rsc;
    if (!mapRegion(kCmdDb, kCmdDbSize, db) || !mapRegion(kRsc, 0x10000, rsc)) return fail(2, -12);
    for (uint32_t i = 0; i < kCmdDbSize; i += 4) {const uint32_t w = rd32(db.va, i); memcpy(copy.bytes + i, &w, 4);}
    const struct {const char *name; uint32_t millivolts;} rails[] = {{"ldob12", 1800}, {"ldob13", 3296}};
    QcomRpmhResource res[2];
    for (unsigned i = 0; i < 2; ++i) if (qcom_cmd_db_find(copy.bytes, kCmdDbSize, rails[i].name, &res[i])) return fail(2, 0x100 + (int32_t)i);
    const QcomRscLayout layout = qcom_rsc_layout(rd32(rsc.va, 0));
    AppsSmmuLock tcsOwner;
    if (!tcsOwner.ready()) return fail(2, -16);
    auto send = [&](uint32_t address, uint32_t value) {
        QcomRpmhWrite writes[16];
        const unsigned n = qcom_rpmh_active_request(&layout, kTcs, &address, &value, 1, writes, 16);
        for (unsigned i = 0; i < n; ++i) wr32(rsc.va, kRscTcs + writes[i].offset, writes[i].value);
        IOSleep(2);
        const unsigned m = qcom_rpmh_finish(&layout, kTcs, writes, 16);
        for (unsigned i = 0; i < m; ++i) wr32(rsc.va, kRscTcs + writes[i].offset, writes[i].value);
    };
    for (unsigned i = 0; i < 2; ++i) {
        send(res[i].address + 8, 7);
        send(res[i].address, rails[i].millivolts);
        send(res[i].address + 4, 1);
    }
    return true;
}

bool setupClocks() {
    Keyboard *kb = gKb;
    if (!mapRegion(kGcc, 0x1000, kb->gccLow) || !mapRegion(kGcc + 0x1e000, 0x1000, kb->gcc1e) ||
        !mapRegion(kGcc + 0x23000, 0x1000, kb->gcc23) || !mapRegion(kGcc + 0x52000, 0x1000, kb->gccVote)) return fail(3, -12);
    wr32(kb->gccVote.va, TB_GCC_GPLL0_ENABLE & 0xfff, rd32(kb->gccVote.va, TB_GCC_GPLL0_ENABLE & 0xfff) | 1u);
    for (unsigned i = 0; i < 10000 && !(rd32(kb->gccLow.va, TB_GCC_GPLL0_MODE) & (1u << 31)); ++i) IODelay(10);
    volatile uint32_t *vote = reinterpret_cast<volatile uint32_t *>(kb->gccVote.va + (kVote & 0xfff));
    if (sm8550_gcc_update32) {
        if (!sm8550_gcc_update32(vote, 0, kVoteBits)) return fail(3, -5);
    } else wr32(kb->gccVote.va, kVote & 0xfff, rd32(kb->gccVote.va, kVote & 0xfff) | kVoteBits);
    for (unsigned i = 0; i < 2000; ++i) {
        bool running = true;
        for (uint32_t reg : kHalts) {
            const uint32_t v = rd32((reg >> 12) == 0x1e ? kb->gcc1e.va : kb->gcc23.va, reg & 0xfff);
            if ((v & TB_CBCR_CLK_OFF) && !(v & 2u)) running = false;
        }
        if (running) return true;
        IODelay(10);
    }
    return fail(3, -60);
}

bool setupBus() {
    Keyboard *kb = gKb;
    if (!mapRegion(kSe, TB_SE_SIZE, kb->se)) return fail(3, -12);
    const uintptr_t se = kb->se.va, rcg = kb->gcc1e.va + (kRcg & 0xfff);
    unsigned level = 0xff;
    for (unsigned l = 0; l < TB_DFS_LEVELS && level == 0xff; ++l)
        if (tb_dfs_level_hz(rd32(rcg, TB_RCG_PERF_CFG(l)), rd32(rcg, TB_RCG_PERF_M(l)), rd32(rcg, TB_RCG_PERF_N(l))) == 19200000ULL) level = l;
    const uint32_t proto = (rd32(se, GENI_FW_REVISION) >> 8) & 0xff;
    native_keyboard_stats[2] = (int32_t)(proto << 24 | (rd32(se, GENI_CLK_CFG) & 0xffffff));
    if (proto != GENI_PROTO_I2C) return fail(3, (int32_t)(0x200 | proto));
    if (level != 0xff) {
        const uint32_t divider = kb->khz == 1000 ? 1 : kb->khz == 400 ? 2 : 7;
        const uint32_t counters = kb->khz == 1000 ? (2u << 20 | 8u << 10 | 18u) : kb->khz == 400 ? (5u << 20 | 11u << 10 | 22u)
                                                                                                 : (10u << 20 | 12u << 10 | 26u);
        wr32(se, 0x28, rd32(se, 0x28) | 0x7fu);
        wr32(se, 0xe30, rd32(se, 0xe30) | 0xfu);
        wr32(se, 0x24, 0x7fu);
        wr32(se, 0x20, 1u);
        wr32(se, 0xe18, 0);
        wr32(se, 0x7c, level);
        wr32(se, GENI_CLK_CFG, divider << 4 | 1u);
        wr32(se, GENI_SCL_COUNTERS, counters);
    } else if (!(rd32(se, GENI_CLK_CFG) & 1u) || !rd32(se, GENI_SCL_COUNTERS)) return fail(3, 0x300);
    const GeniIo io = {ioRd, ioWr, reinterpret_cast<void *>(se)};
    uint32_t config = 0;
    if (geni_i2c_prepare(&kb->bus, &io, WN8030_ADDRESS, &config)) return fail(3, (int32_t)config);
    native_keyboard_stats[2] = (int32_t)config;
    return true;
}

bool setupInterrupts() {
    if (!armcpu_spi_attach || !armcpu_spi_share) return fail(4, -78);
    int rc = 0;
    for (unsigned tries = 0;; ++tries) {
        rc = armcpu_spi_attach(kSeIntid, false, busIrq, nullptr, nullptr, nullptr);
        if (rc != kIOReturnNotReady || tries == 240) break;
        IOSleep(250);
    }
    if (rc) return fail(4, rc);
    for (PinIndex i : kInterruptPins) {
        uint32_t cfg = rd32(pin(i), kIntrCfg) & ~((7u << 5) | (3u << 2) | (1u << 1) | 1u);
        cfg |= (3u << 5) | ((i == PinIrq ? 1u : 3u) << 2) | (1u << 1) | (1u << 4);
        wr32(pin(i), kIntrCfg, cfg);
        wr32(pin(i), kIntrStatus, 0);
    }
    rc = armcpu_spi_share(kTlmmIntid, tlmmIrq, nullptr);
    if (rc) return fail(4, 0x1000 | rc);
    for (PinIndex i : kInterruptPins) wr32(pin(i), kIntrCfg, rd32(pin(i), kIntrCfg) | 1u);
    return true;
}

bool powerOn() {
    setOutput(PinSleep, false);
    setOutput(PinReset, false);
    IOSleep(10);
    setOutput(PinReset, true);
    IOSleep(20);
    setOutput(PinSleep, true);
    IOSleep(5);
    if (!romWrite(WN8030_ROM_ID)) return fail(5, 1);
    const int id = romRead();
    const int state = romState();
    native_keyboard_stats[3] = (id & 0xff) | (state & 0xff) << 8;
    if (id != (int)WN8030_BOOT_ID) return fail(5, id);
    if (state != Wn8030RomIdle) return fail(5, 0x100 | state);
    return true;
}

bool loadImage() {
    Keyboard *kb = gKb;
    const uint64_t start = nowMs();
    uint32_t code = 0;
    const int checked = wn8030_image_check(native_keyboard_firmware, native_keyboard_firmware_size, &code);
    if (checked) return fail(6, checked);
    uint32_t n = wn8030_header_command(kb->command, native_keyboard_firmware, code);
    if (!writeBuffer(kb->command, n)) return fail(6, -10);
    const int header = romState();
    native_keyboard_stats[3] |= (header & 0xff) << 16;
    if (header != Wn8030RomHeader) return fail(6, 0x100 | (header & 0xff));
    for (uint32_t offset = 0, next = 0; offset < code; offset = next) {
        n = wn8030_data_command(kb->command, native_keyboard_firmware, code, offset, &next);
        if (!n || !writeBuffer(kb->command, n)) return fail(6, (int32_t)(0x10000 | offset));
    }
    if (!romWrite(WN8030_ROM_VERIFY)) return fail(6, -11);
    IOSleep(10);
    const int verified = romState();
    native_keyboard_stats[3] |= (verified & 0xff) << 24;
    if (verified != Wn8030RomVerified) return fail(6, 0x200 | (verified & 0xff));
    kb->service.seen = __atomic_load_n(&kb->mcuIrqs, __ATOMIC_ACQUIRE);
    if (!romWrite(WN8030_ROM_BOOT)) return fail(6, -12);
    IOSleep(50);
    native_keyboard_stats[4] = (int32_t)(nowMs() - start);
    return true;
}

bool registerClass() {
    Keyboard *kb = gKb;
    if (kb->classRegistered) return true;
    if (OSKextLoadKextWithIdentifier("com.apple.iokit.IOHIDFamily")) return false;
    void *handle = OSMetaClass::preModLoad(g_kernel_kmod_info.name);
    if (!handle) return false;
    sheng_keyboard_cover_static_init();
    if (OSMetaClass::postModLoad(handle)) return false;
    kb->classRegistered = true;
    return true;
}

void ledRequest(void *, uint8_t caps) {
    Keyboard *kb = gKb;
    __atomic_store_n(&kb->capsWanted, caps ? 1u : 0u, __ATOMIC_RELEASE);
    __atomic_fetch_add(&kb->capsRequests, 1, __ATOMIC_RELEASE);
    thread_wakeup((event_t)&gWake);
}

IOService *createHid() {
    if (!registerClass()) return nullptr;
    OSDictionary *match = IOService::serviceMatching("IOHIDResource");
    IOService *resource = match ? IOService::waitForMatchingService(match, 5000000000ULL) : nullptr;
    IOService *device = resource ? sheng_keyboard_cover_allocate() : nullptr;
    bool started = false, attached = false;
    if (device && device->init()) {
        attached = device->attach(resource);
        started = attached && device->start(resource);
    }
    if (!started && device) {
        if (attached) device->detach(resource);
        OSSafeReleaseNULL(device);
    }
    OSSafeReleaseNULL(resource);
    OSSafeReleaseNULL(match);
    return device;
}

void releaseInputs() {
    uint8_t keyboard[WN8030_KEYBOARD_REPORT], consumer[WN8030_CONSUMER_REPORT], touchpad[WN8030_TOUCHPAD_REPORT];
    wn8030_release_reports(keyboard, consumer, touchpad);
    gKb->rctrlBrightness = 0;
    gKb->rctrlKbdBl = 0;
    gKb->keys.reset();
    IOService *hid = gKb->hid;
    if (!hid) return;
    sheng_keyboard_cover_input(hid, keyboard, sizeof keyboard);
    sheng_keyboard_cover_input(hid, consumer, sizeof consumer);
    sheng_keyboard_cover_input(hid, touchpad, sizeof touchpad);
}

bool applyInputState() {
    Keyboard *kb = gKb;
    if (!kb->attached) return true;
    const bool on = kb->inputEnabled;
    bool ok = control(WN8030_CMD_TOUCHPAD, on ? 1 : 0);
    ok = control(WN8030_CMD_INDICATOR, on && kb->caps ? WN8030_LED_CAPS_ON : WN8030_LED_CAPS_OFF) && ok;
    ok = control(WN8030_CMD_INDICATOR, WN8030_LED_MIC_OFF) && ok;
    const uint32_t bl = kb->kbdBacklight.level();
    if (bl <= WN8030_BACKLIGHT_MAX) {
        const uint8_t target = on ? static_cast<uint8_t>(bl) : 0;
        const bool blOk = control(WN8030_CMD_BACKLIGHT, target);
        if (blOk) IOLog("sheng keyboard: backlight -> %u\n", (unsigned)target);
        ok = blOk && ok;
    }
    return ok;
}

void attachCover() {
    Keyboard *kb = gKb;
    kb->attached = true;
    kb->detachPending = false;
    kb->caps = false;
    kb->service.statePending = true;
    kb->kbdBacklight.detach();
    (void)kb->kbdBacklight.attach(nowMs());
    if (!kb->hid) kb->hid = createHid();
    if (kb->hid) {
        kb->hidRetries = 0;
        kb->hidRetryDue = 0;
        sheng_keyboard_cover_configure(kb->hid, kb->gesturesEnabled, wn8030_keep_lone_contact != 0);
        sheng_keyboard_cover_enable(kb->hid, kb->inputEnabled);
    } else {
        bump(8, 1 << 24);
        kb->hidRetryDue = nowMs() + 2000;
    }
    kb->service.keepalive = nowMs();
    if (sheng_battery_keyboard_attached) sheng_battery_keyboard_attached(1);
    native_keyboard_stats[8] = (native_keyboard_stats[8] & ~0xff) + 0x100 + 1;
}

void detachCover() {
    Keyboard *kb = gKb;
    kb->attached = false;
    kb->detachPending = false;
    kb->service.statePending = false;
    kb->kbdBacklight.detach();
    releaseInputs();
    if (kb->hid) sheng_keyboard_cover_enable(kb->hid, false);
    if (sheng_battery_keyboard_attached) sheng_battery_keyboard_attached(0);
    native_keyboard_stats[8] = native_keyboard_stats[8] & ~0xff;
}

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
lid_clamshell gClamshell = LID_CLAMSHELL_INIT;
#endif
}  // namespace
extern "C" void sheng_backlight_lid(bool closed) __attribute__((weak_import));
namespace {

void publishLid(bool closed, bool changed) {
    __atomic_store_n(&native_keyboard_lid_closed, closed ? 1u : 0u, __ATOMIC_RELEASE);
    if (changed) IOLog(kLidLog, closed ? "closed" : "open");
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
    IOPMrootDomain *root = IOService::getPMRootDomain();
    if (!root) return;
    const lid_clamshell_message message = lid_clamshell_step(&gClamshell, closed);
    if (message == LidClamshellNone) return;
    constexpr uint32_t kSetValue = 1u << 16;
    root->receivePowerNotification(message == LidClamshellClosed ? kIOPMClamshellClosed : kIOPMClamshellOpened | kSetValue);
    IOLog(kClamshellLog, message == LidClamshellClosed ? "closed" : "open");
    if (sheng_backlight_lid) sheng_backlight_lid(message == LidClamshellClosed);
#endif
}

void readHall() {
    Keyboard *kb = gKb;
    const bool wasLid = kb->lid;
    kb->lid = !level(PinLid);
    kb->tablet = !level(PinTablet);
    native_keyboard_stats[9] = (native_keyboard_stats[9] & ~3) | (kb->lid ? 1 : 0) | (kb->tablet ? 2 : 0);
    publishLid(kb->lid, kb->lid != wasLid);
    const bool enabled = !kb->lid && !kb->tablet;
    if (enabled == kb->inputEnabled) return;
    kb->inputEnabled = enabled;
    native_keyboard_stats[10] = enabled;
    if (!enabled) releaseInputs();
    if (kb->hid) sheng_keyboard_cover_enable(kb->hid, enabled);
    kb->service.statePending = true;
}

kb_trace_kind traceKind(int kind) {
    switch (kind) {
    case Wn8030Keyboard: return KbTraceKeyboard;
    case Wn8030Consumer: return KbTraceConsumer;
    case Wn8030Touchpad: return KbTraceTouchpad;
    case Wn8030Attach: return KbTraceAttach;
    case Wn8030Accel: return KbTraceAccel;
    case Wn8030AuthRequest: case Wn8030AuthInit: case Wn8030AuthChallenge: return KbTraceAuth;
    case Wn8030Invalid: return KbTraceInvalid;
    default: return KbTraceOther;
    }
}

void logAttach(const char *what, uint32_t now) {
    Keyboard *kb = gKb;
    kb_trace_summary s;
    kb_trace_summarize(&kb->trace, now, &s);
    const uint8_t *b = kb->frame;
    const int32_t *k = const_cast<const int32_t *>(native_keyboard_stats);
    IOLog(kCoverLog, what, b[5], b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13], b[14], b[15], b[16], s.kinds,
          (int32_t)s.since_any, (int32_t)s.since_input, (int32_t)s.since_accel, k[6], k[5], k[11], k[13], k[15] & 0xffff,
          (k[8] >> 8) & 0xffff, kb->reloads);
}

KeyboardRead handleFrame() {
    Keyboard *kb = gKb;
    if (!readFrame()) return KeyboardRead::Error;
    bump(6);
    Wn8030Frame f;
    const int kind = wn8030_parse(kb->frame, &f);
    const uint64_t now64 = nowMs();
    const uint32_t now = static_cast<uint32_t>(now64);
    if (kind == Wn8030Attach) {
        if (f.attached == 1 && !kb->attached) logAttach("attached", now);
        else if (f.attached == 0 && kb->attached) logAttach("detached", now);
        else if (f.attached < 0) logAttach("changing", now);
    }
    kb_trace_push(&kb->trace, now, traceKind(kind));
    if (kind == Wn8030Invalid) return KeyboardRead::Empty;
    switch (kind) {
    case Wn8030Keyboard: case Wn8030Consumer: case Wn8030Touchpad:
        if (kb->detachPending || !kb->attached) {
            const bool wasDetachPending = kb->detachPending;
            kb->detachPending = false;
            if (!kb->attached) attachCover();
            else if (wasDetachPending) kb->service.statePending = true;
        }
        if (kb->hid && kb->inputEnabled) {
            uint8_t touchpad[WN8030_TOUCHPAD_REPORT];
            uint8_t filtered[WN8030_KEYBOARD_REPORT];
            uint8_t kbdCopy[WN8030_KEYBOARD_REPORT];
            uint8_t synthConsumer[WN8030_CONSUMER_REPORT] = {WN8030_CONSUMER_ID, 0, 0};
            const uint8_t *reportOut = f.report;
            bool sendSynth = false;
            int stepBrightness = 0;
            uint16_t kbdBlUsage = 0;
            if (f.kind == Wn8030Touchpad) {
                for (uint32_t i = 0; i < WN8030_TOUCHPAD_FINGERS; ++i) {
                    const uint8_t *finger = f.report + 2 + 6 * i;
                    if (!(finger[0] & 1u)) continue;
                    const int32_t x = finger[2] | finger[3] << 8, y = finger[4] | finger[5] << 8;
                    volatile int32_t *r = native_keyboard_touch_range;
                    if (r[0] < 0 || x < r[0]) r[0] = x;
                    if (x > r[1]) r[1] = x;
                    if (r[2] < 0 || y < r[2]) r[2] = y;
                    if (y > r[3]) r[3] = y;
                }
                bump(16, (int32_t)wn8030_touchpad_report(f.report, touchpad));
                reportOut = touchpad;
                applyBacklight(kb->kbdBacklight.input(now64));
            } else if (f.kind == Wn8030Consumer && f.length >= WN8030_CONSUMER_REPORT) {
                const uint16_t usage = (uint16_t)f.report[1] | ((uint16_t)f.report[2] << 8);
                if (usage == 0x006f) stepBrightness = 1;
                else if (usage == 0x0070) stepBrightness = -1;
                else applyBacklight(kb->kbdBacklight.consumer(usage, now64));
            } else if (f.kind == Wn8030Keyboard && f.length >= WN8030_KEYBOARD_REPORT) {
                applyBacklight(kb->kbdBacklight.input(now64));
                uint8_t prevSent[6];
                memcpy(prevSent, kb->keys.sent(), sizeof prevSent);
                const bool rollover = kb->keys.filter(f.report, filtered);
                if (!rollover && pianokeyboard::Keys::newKey(prevSent, filtered)) {
                    sheng_keyboard_cover_key_typed(kb->hid, now64);
                }
                reportOut = filtered;
                if (!kb->fnRowEnabled) {
                    memcpy(kbdCopy, filtered, WN8030_KEYBOARD_REPORT);
                    const bool rctrl = (kbdCopy[1] & 0x10u) != 0;
                    uint8_t curBr = 0, curBl = 0;
                    for (unsigned i = 3; i < WN8030_KEYBOARD_REPORT; ++i) {
                        if (!rctrl) continue;
                        if (kbdCopy[i] == 0x1e) {curBr |= 1u; kbdCopy[i] = 0;}
                        else if (kbdCopy[i] == 0x1f) {curBr |= 2u; kbdCopy[i] = 0;}
                        else if (kbdCopy[i] == 0x20) {curBl |= 1u; kbdCopy[i] = 0;}
                        else if (kbdCopy[i] == 0x21) {curBl |= 2u; kbdCopy[i] = 0;}
                        else if (kbdCopy[i] == 0x27) {curBl |= 4u; kbdCopy[i] = 0;}
                    }
                    if (curBr || curBl) reportOut = kbdCopy;
                    if ((curBr & 1u) && !(kb->rctrlBrightness & 1u)) stepBrightness = -1;
                    else if ((curBr & 2u) && !(kb->rctrlBrightness & 2u)) stepBrightness = 1;
                    if (curBr != kb->rctrlBrightness) {
                        if ((curBr & 1u) && (stepBrightness < 0 || !(curBr & 2u))) {sendSynth = true; synthConsumer[1] = 0x70;}
                        else if ((curBr & 2u) && (stepBrightness > 0 || !(curBr & 1u))) {sendSynth = true; synthConsumer[1] = 0x6f;}
                        else if (!curBr && kb->rctrlBrightness) sendSynth = true;
                    }
                    kb->rctrlBrightness = curBr;
                    if ((curBl & 1u) && !(kb->rctrlKbdBl & 1u)) kbdBlUsage = 0x7a;
                    else if ((curBl & 2u) && !(kb->rctrlKbdBl & 2u)) kbdBlUsage = 0x79;
                    else if ((curBl & 4u) && !(kb->rctrlKbdBl & 4u)) kbdBlUsage = 0x7c;
                    kb->rctrlKbdBl = curBl;
                }
            }
            sheng_keyboard_cover_input(kb->hid, reportOut, f.length);
            if (f.kind == Wn8030Touchpad) {
                if (const uint8_t swipeKey = sheng_keyboard_cover_take_swipe(kb->hid)) {
                    uint8_t seq[4][WN8030_KEYBOARD_REPORT];
                    const unsigned count = kb->keys.shortcut(swipeKey, seq);
                    for (unsigned s = 0; s < count; ++s) {
                        sheng_keyboard_cover_input(kb->hid, seq[s], WN8030_KEYBOARD_REPORT);
                    }
                }
            }
            if (sendSynth) sheng_keyboard_cover_input(kb->hid, synthConsumer, WN8030_CONSUMER_REPORT);
            if (stepBrightness && sheng_backlight_step) sheng_backlight_step(stepBrightness);
            if (kbdBlUsage) applyBacklight(kb->kbdBacklight.consumer(kbdBlUsage, now64));
            bump(7, f.kind == Wn8030Keyboard ? 1 : f.kind == Wn8030Consumer ? 0x100 : 0x10000);
        }
        break;
    case Wn8030Attach:
        if (f.attached == 1) {
            kb->detachPending = false;
            if (!kb->attached) {
                attachCover();
            } else {
                kb->service.statePending = true;
                kb->service.keepalive = nowMs();
                kb->kbdBacklight.detach();
                (void)kb->kbdBacklight.attach(nowMs());
                if (sheng_battery_keyboard_attached) sheng_battery_keyboard_attached(1);
            }
        } else if (f.attached == 0 && kb->attached) {
            releaseInputs();
            if (!kb->detachPending) {
                kb->detachPending = true;
                kb->detachDueMs = nowMs() + 500;
            }
        } else if (f.attached >= 0 && sheng_battery_keyboard_attached) {
            sheng_battery_keyboard_attached(f.attached);
        }
        break;
    case Wn8030Accel:
        native_keyboard_accel[0] = f.x; native_keyboard_accel[1] = f.y; native_keyboard_accel[2] = f.z;
        native_keyboard_accel[3] = native_keyboard_accel[3] + 1;
        break;
    case Wn8030AuthRequest: case Wn8030AuthInit: case Wn8030AuthChallenge:
        bump(15);
        break;
    default: bump(15, 0x10000); break;
    }
    return KeyboardRead::Valid;
}

bool recoverController() {
    Keyboard *kb = gKb;
    releaseInputs();
    setOutput(PinSleep, false);
    setOutput(PinReset, false);
    IOInterruptState state = IOSimpleLockLockDisableInterrupt(kb->busLock);
    wr32(kb->se.va, GENI_M_IRQ_EN, 0);
    wr32(kb->se.va, GENI_TX_WATERMARK, 0);
    if (rd32(kb->se.va, 0x40) & 1u) wr32(kb->se.va, GENI_M_CMD_CTRL, GENI_CMD_CTRL_ABORT);
    IOSimpleLockUnlockEnableInterrupt(kb->busLock, state);
    for (unsigned i = 0; i < 20 && (rd32(kb->se.va, 0x40) & 1u); ++i) IOSleep(1);
    if (rd32(kb->se.va, 0x40) & 1u) return false;
    const GeniIo io = {ioRd, ioWr, reinterpret_cast<void *>(kb->se.va)};
    uint32_t config = 0;
    state = IOSimpleLockLockDisableInterrupt(kb->busLock);
    const int prepared = geni_i2c_prepare(&kb->bus, &io, WN8030_ADDRESS, &config);
    IOSimpleLockUnlockEnableInterrupt(kb->busLock, state);
    if (prepared || !powerOn() || !loadImage()) return false;
    kb->kbdBacklight.detach();
    stage(7);
    return true;
}

struct KeyboardIO {
    uint64_t now() const {return nowMs();}
    uint32_t irqs() const {return __atomic_load_n(&gKb->mcuIrqs, __ATOMIC_ACQUIRE);}
    bool irqHigh() const {return level(PinIrq);}
    bool attached() const {return gKb->attached;}
    KeyboardRead read() const {return handleFrame();}
    void consumed(uint32_t seen) const {native_keyboard_stats[5] = (int32_t)seen;}
    void activity(uint64_t t) const {gKb->pollPlan.activity(t);}
    uint32_t pollIntervalMs() const {
        Keyboard *kb = gKb;
        const auto m = kb->pollPlan.mode(nowMs(), kb->attached, kb->lid || kb->tablet);
        return kb->pollPlan.readEveryMs(m);
    }
    bool power() const {setOutput(PinSleep, true); return control(WN8030_CMD_KEYBOARD_POWER, 1);}
    bool apply() const {return applyInputState();}
    void release() const {releaseInputs();}
    void wake() const {
        IOLog(kKeepAwakeFailureLog,
              native_keyboard_stats[11], native_keyboard_stats[13]);
        setOutput(PinSleep, false); IOSleep(10);
        setOutput(PinSleep, true); IOSleep(25);
    }
    bool recover() const {
        Keyboard *kb = gKb;
        ++kb->reloads;
        IOLog(kControllerReloadLog, kb->reloads, kb->service.readFailures, kb->service.controlFailures, native_keyboard_stats[11],
              native_keyboard_stats[13], native_keyboard_stats[6]);
        const bool ok = recoverController();
        IOLog(kControllerReloadResultLog, kb->reloads, ok ? "done" : "failed",
              native_keyboard_stats[0], native_keyboard_stats[1]);
        return ok;
    }
};

#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
IOReturn keyboardSleepWake(void *, void *, natural_t message, IOService *, void *, vm_size_t) {
    if ((message == kIOMessageSystemWillPowerOn || message == kIOMessageSystemHasPoweredOn) && gKb) {
        __atomic_store_n(&gKb->wakeRequested, 1u, __ATOMIC_RELEASE);
        thread_wakeup((event_t)&gWake);
    }
    return kIOReturnSuccess;
}
#endif

void serve() {
    Keyboard *kb = gKb;
    KeyboardIO io;
    kb->hallPoll = nowMs() + kHallPollMs;
    kb->service.keepalive = nowMs() + 500;
    kb->service.pollDue = nowMs() + 500;
    for (;;) {
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
        if (!kb->sleepRegistered && IOService::getPMRootDomain()) {
            kb->sleepRegistered = registerSleepWakeInterest(keyboardSleepWake, nullptr, nullptr) != nullptr;
        }
        if (__atomic_exchange_n(&kb->wakeRequested, 0u, __ATOMIC_ACQ_REL)) {
            if (kb->attached) {
                setOutput(PinSleep, false); IOSleep(10);
                setOutput(PinSleep, true); IOSleep(25);
                kb->service.statePending = true;
            }
            kb->service.highBackoffDue = 0;
            kb->service.keepalive = nowMs();
            kb->service.pollDue = nowMs();
        }
#endif
        const uint32_t hall = __atomic_load_n(&kb->hallIrqs, __ATOMIC_ACQUIRE);
        if (hall != kb->hallSeen) {
            kb->hallSeen = hall;
            kb->hallPending = true;
            kb->hallDue = nowMs() + kDebounceMs;
            native_keyboard_stats[9] = (native_keyboard_stats[9] & 3) | (int32_t)(hall << 8);
        }
        const uint64_t now = nowMs();
        if (kb->detachPending && now >= kb->detachDueMs) {
            kb->detachPending = false;
            if (kb->attached) detachCover();
        }
        if (kb->attached && !kb->hid && kb->hidRetries < 30 && now >= kb->hidRetryDue) {
            ++kb->hidRetries;
            kb->hid = createHid();
            if (kb->hid) {
                kb->hidRetryDue = 0;
                sheng_keyboard_cover_configure(kb->hid, kb->gesturesEnabled, wn8030_keep_lone_contact != 0);
                sheng_keyboard_cover_enable(kb->hid, kb->inputEnabled);
                kb->service.statePending = true;
            } else if (kb->hidRetries < 30) {
                kb->hidRetryDue = nowMs() + 2000;
            } else {
                kb->hidRetryDue = 0;
            }
        }
        if ((kb->hallPending && now >= kb->hallDue) || (!kb->hallPending && now >= kb->hallPoll)) {
            kb->hallPending = false;
            readHall();
            kb->hallPoll = nowMs() + kHallPollMs;
        }
        const uint32_t requests = __atomic_load_n(&kb->capsRequests, __ATOMIC_ACQUIRE);
        if (requests != kb->capsSeen) {
            kb->capsSeen = requests;
            kb->caps = __atomic_load_n(&kb->capsWanted, __ATOMIC_ACQUIRE) != 0;
            kb->service.statePending = kb->attached;
        }
        applyBacklight(kb->kbdBacklight.tick(nowMs()));
        kb->service.step(io);
        const uint64_t next = kb->service.deadline(nowMs(), io.irqs(), io.irqHigh());
        const uint64_t hallDeadline = kb->hallPending ? kb->hallDue : kb->hallPoll;
        uint64_t due = next < hallDeadline ? next : hallDeadline;
        if (kb->detachPending && kb->detachDueMs < due) due = kb->detachDueMs;
        if (kb->attached && !kb->hid && kb->hidRetries < 30 && kb->hidRetryDue < due) due = kb->hidRetryDue;
        const uint64_t current = nowMs();
        assert_wait_deadline((event_t)&gWake, THREAD_UNINT, deadlineMs(due > current ? (uint32_t)(due - current) : 1));
        if (kb->service.deadline(nowMs(), io.irqs(), io.irqHigh()) <= nowMs() ||
            kb->hallSeen != __atomic_load_n(&kb->hallIrqs, __ATOMIC_ACQUIRE) ||
            kb->capsSeen != __atomic_load_n(&kb->capsRequests, __ATOMIC_ACQUIRE) ||
            __atomic_load_n(&kb->wakeRequested, __ATOMIC_ACQUIRE) != 0 ||
            (kb->detachPending && nowMs() >= kb->detachDueMs) ||
            (kb->attached && !kb->hid && kb->hidRetries < 30 && nowMs() >= kb->hidRetryDue))
            clear_wait(current_thread(), THREAD_AWAKENED);
        else thread_block(THREAD_CONTINUE_NULL);
    }
}

void keyboardThread(void *, wait_result_t) {
    thread_set_thread_name(current_thread(), "keyboard");
    Keyboard *kb = gKb;
    stage(1);
    if (!setupPins()) return;
    stage(2);
    if (!setupRails()) return;
    stage(3);
    if (!setupClocks() || !setupBus()) return;
    stage(4);
    if (!setupInterrupts()) return;
    kb->lid = !level(PinLid);
    kb->tablet = !level(PinTablet);
    kb->inputEnabled = !kb->lid && !kb->tablet;
    native_keyboard_stats[9] = (kb->lid ? 1 : 0) | (kb->tablet ? 2 : 0);
    native_keyboard_stats[10] = kb->inputEnabled;
    stage(5);
    if (!powerOn()) return;
    stage(6);
    if (!loadImage()) return;
    sheng_keyboard_cover_output_sink(ledRequest, nullptr);
    stage(7);
    (void)control(WN8030_CMD_KEYBOARD_POWER, 1);
    publishLid(kb->lid, kb->lid);
    serve();
}
}  // namespace

extern "C" void sheng_keyboard_probe(void) {
    unsigned enabled = 0;
    if (!PE_parse_boot_argn("keyboard", &enabled, sizeof enabled) || enabled != 1 || gKb) return;
    Keyboard *kb = static_cast<Keyboard *>(IOMallocType(Keyboard));
    if (!kb) {fail(8, -12); return;}
    bzero(kb, sizeof *kb);
    kb->busLock = IOSimpleLockAlloc();
    if (!kb->busLock) {IOFreeType(kb, Keyboard); fail(8, -12); return;}
    unsigned khz = 1000, backlight = 50, idleSec = 0, gestures = 1, fnrow = 0;
    PE_parse_boot_argn("keyboard-khz", &khz, sizeof khz);
    uint32_t lone = 0;
    if (PE_parse_boot_argn("keyboard-lone", &lone, sizeof lone)) wn8030_keep_lone_contact = lone == 1;
    PE_parse_boot_argn("keyboard-backlight", &backlight, sizeof backlight);
    PE_parse_boot_argn("keyboard-backlight-idle", &idleSec, sizeof idleSec);
    PE_parse_boot_argn("keyboard-gestures", &gestures, sizeof gestures);
    PE_parse_boot_argn("keyboard-fnrow", &fnrow, sizeof fnrow);
    kb->khz = khz == 100 || khz == 400 ? khz : 1000;
    kb->backlightLevel = backlight;
    kb->backlightIdleMs = (idleSec > 3600 ? 3600 : idleSec) * 1000u;
    kb->gesturesEnabled = gestures != 0;
    kb->fnRowEnabled = fnrow == 1;
    kb->keys.fnRow = kb->fnRowEnabled;
    kb->pollPlan.configure(pianokeyboard::PollConfig());
    kb->kbdBacklight.configure(kb->backlightLevel, kb->backlightIdleMs);
    gKb = kb;
    thread_t thread = nullptr;
    if (kernel_thread_start(keyboardThread, nullptr, &thread) == KERN_SUCCESS) thread_deallocate(thread);
    else fail(8, -12);
}
