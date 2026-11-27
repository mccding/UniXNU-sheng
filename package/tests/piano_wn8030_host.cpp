// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test for PianoWn8030.h (Pad 8 Pro Keys, Backlight, PollPlan, and Pointer over sheng's 21-byte touchpad report).
extern "C" {
#include "../Wn8030.h"
}
#include "../PianoWn8030.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>

using namespace pianokeyboard;

static unsigned checks;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } ++checks; } while (0)

struct Finger {
    uint8_t flags, id;
    uint16_t x, y;
};

static void buildReport(uint8_t raw[WN8030_TOUCHPAD_REPORT], uint8_t flipped[WN8030_TOUCHPAD_REPORT],
                        uint8_t buttons, std::initializer_list<Finger> fingers, uint8_t countOverride = 0xff)
{
    std::memset(raw, 0, WN8030_TOUCHPAD_REPORT);
    raw[0] = WN8030_TOUCHPAD_ID;
    raw[1] = buttons;
    unsigned i = 0;
    for (const Finger &f : fingers) {
        if (i >= 3) break;
        uint8_t *p = raw + 2 + 6 * i++;
        p[0] = f.flags;
        p[1] = f.id;
        putLe16(p + 2, f.x);
        putLe16(p + 4, f.y);
    }
    raw[20] = countOverride == 0xff ? uint8_t(i) : countOverride;
    wn8030_touchpad_report(raw, flipped);
}

int main()
{
    // 1. Keys: rollover protection, shortcut, newKey, optional fnRow.
    Keys keys;
    uint8_t in[kKeyboardBytes] = {5, 0x02, 0, 0x04, 0x05, 0, 0, 0, 0}, out[kKeyboardBytes] = {};
    CHECK(!keys.filter(in, out) && out[3] == 0x04 && out[4] == 0x05);
    const uint8_t err[kKeyboardBytes] = {5, 0x02, 0, 0x01, 0, 0, 0, 0, 0};
    CHECK(keys.filter(err, out) && out[3] == 0x04 && out[4] == 0x05 && keys.rollovers() == 1);
    const uint8_t emptySent[6] = {};
    CHECK(Keys::newKey(emptySent, in));
    CHECK(!Keys::newKey(keys.sent(), in));
    uint8_t seq[4][kKeyboardBytes] = {};
    CHECK(keys.shortcut(0x52, seq) == 4 && (seq[0][1] & kModLeftCtrl) && seq[1][5] == 0x52 && seq[2][5] == 0 && !(seq[3][1] & kModLeftCtrl));
    keys.fnRow = true;
    const uint8_t fnIn[kKeyboardBytes] = {5, kModRightCtrl, 0, 0x1e, 0, 0, 0, 0, 0};
    CHECK(!keys.filter(fnIn, out) && out[3] == 0x3a && !(out[1] & kModRightCtrl));

    // 2. Backlight: default 50, consumer 0x79/0x7a/0x7c, idle timeout + wake, >100 disabled.
    Backlight bl;
    bl.configure(50, 30000);
    CHECK(bl.attach(100) == 50 && bl.attach(110) == -1);
    CHECK(bl.consumer(0x79, 200) == 60);
    CHECK(bl.consumer(0x7a, 300) == 50);
    CHECK(bl.consumer(0x7c, 400) == 0 && bl.consumer(0x7c, 500) == 50);
    CHECK(bl.tick(10000) == -1 && bl.tick(30500) == 0);
    CHECK(bl.input(31000) == 50);
    bl.configure(101, 0);
    CHECK(bl.attach(40000) == -1);

    // 3. PollPlan intervals: 1s active / 5s idle / 10s slow.
    PollPlan plan;
    plan.activity(1000);
    CHECK(plan.mode(1500, true, false) == PollMode::active && plan.readEveryMs(PollMode::active) == 1000);
    CHECK(plan.mode(4000, true, false) == PollMode::idle && plan.readEveryMs(PollMode::idle) == 5000);
    CHECK(plan.mode(4000, false, false) == PollMode::slow && plan.readEveryMs(PollMode::slow) == 10000);

    // 4. Pointer on sheng 21-byte format (after wn8030_touchpad_report flip): Y movement up = negative pointer Y, positive wheel.
    Pointer ptr;
    uint8_t raw[WN8030_TOUCHPAD_REPORT], tp[WN8030_TOUCHPAD_REPORT], outPtr[2][kPointerBytes];
    buildReport(raw, tp, 0, {{7, 1, 1000, 1000}});
    CHECK(ptr.touch(tp, 100, outPtr) == 0);
    // Finger moves UP on pad (in desktop coords Y decreases: 1000 -> 820 -> flipped increases).
    buildReport(raw, tp, 0, {{7, 1, 1090, 820}});
    CHECK(ptr.touch(tp, 113, outPtr) == 1);
    const int16_t mx = int16_t(le16(outPtr[0] + 1)), my = int16_t(le16(outPtr[0] + 3));
    CHECK(mx == 90 && my < 0);   // moving up -> negative pointer Y (cursor up!)

    // Lift and tap (1 finger -> left click; 2 fingers -> right click; 3 fingers -> middle click).
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 400, outPtr) == 0);   // too long (>220ms), not a tap
    buildReport(raw, tp, 0, {{7, 2, 1200, 900}});
    CHECK(ptr.touch(tp, 1000, outPtr) == 0);
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 1050, outPtr) == 2 && outPtr[0][0] == 1 && outPtr[1][0] == 0);
    buildReport(raw, tp, 0, {{7, 1, 1200, 900}, {7, 2, 1300, 900}});
    CHECK(ptr.touch(tp, 2000, outPtr) == 0);
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 2050, outPtr) == 2 && outPtr[0][0] == 2 && outPtr[1][0] == 0);
    buildReport(raw, tp, 0, {{7, 1, 1200, 900}, {7, 2, 1300, 900}, {7, 3, 1400, 900}});
    CHECK(ptr.touch(tp, 3000, outPtr) == 0);
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 3050, outPtr) == 2 && outPtr[0][0] == 4 && outPtr[1][0] == 0);

    // 2-finger smooth vertical scroll UP (finger Y 1000 -> 800): wheel > 0.
    buildReport(raw, tp, 0, {{7, 1, 1000, 1000}, {7, 2, 1200, 1000}});
    CHECK(ptr.touch(tp, 4000, outPtr) == 0);
    buildReport(raw, tp, 0, {{7, 1, 1000, 800}, {7, 2, 1200, 800}});
    CHECK(ptr.touch(tp, 4015, outPtr) == 1 && int8_t(outPtr[0][5]) > 0 && outPtr[0][6] == 0);
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 4400, outPtr) == 0);

    // Resting thumb in bottom fifth (y >= 1624) while index finger moves in top half: treated as 1 finger move, not 2-finger scroll!
    buildReport(raw, tp, 0, {{7, 1, 1200, 800}, {7, 2, 800, 1850}});
    CHECK(ptr.touch(tp, 5000, outPtr) == 0);
    buildReport(raw, tp, 0, {{7, 1, 1240, 800}, {7, 2, 800, 1850}});
    CHECK(ptr.touch(tp, 5015, outPtr) == 1 && int16_t(le16(outPtr[0] + 1)) == 40 && outPtr[0][5] == 0);
    buildReport(raw, tp, 0, {});
    ptr.touch(tp, 5400, outPtr);

    // 3-finger swipe UP -> Swipe::up (0x52).
    buildReport(raw, tp, 0, {{7, 1, 1000, 1200}, {7, 2, 1200, 1200}, {7, 3, 1400, 1200}});
    CHECK(ptr.touch(tp, 6000, outPtr) == 0);
    buildReport(raw, tp, 0, {{7, 1, 1000, 600}, {7, 2, 1200, 600}, {7, 3, 1400, 600}});
    CHECK(ptr.touch(tp, 6030, outPtr) == 0 && ptr.takeSwipe() == Swipe::up);
    buildReport(raw, tp, 0, {});
    ptr.touch(tp, 6400, outPtr);

    // Typing palm guard (250 ms after keyTyped).
    ptr.keyTyped(7000);
    buildReport(raw, tp, 0, {{7, 1, 1000, 1000}});
    CHECK(ptr.touch(tp, 7100, outPtr) == 0);
    buildReport(raw, tp, 0, {{7, 1, 1100, 1000}});
    CHECK(ptr.touch(tp, 7120, outPtr) == 0);
    buildReport(raw, tp, 0, {});
    CHECK(ptr.touch(tp, 7150, outPtr) == 0);   // tap suppressed while typing!

    // Lone unconfident contact when keepLoneContact=true + wn8030_keep_lone_contact=1.
    wn8030_keep_lone_contact = 1;
    PointerConfig cfg = ptr.config();
    cfg.keepLoneContact = true;
    ptr.configure(cfg);
    buildReport(raw, tp, 0, {{3, 1, 1000, 1000}});   // confidence bit 2 = 0, tip=1, in_range=2
    CHECK(ptr.touch(tp, 8000, outPtr) == 0);
    buildReport(raw, tp, 0, {{3, 1, 1020, 1000}});
    CHECK(ptr.touch(tp, 8015, outPtr) == 1 && int16_t(le16(outPtr[0] + 1)) == 20);

    std::printf("PASS PianoWn8030: %u checks\n", checks);
    return 0;
}
