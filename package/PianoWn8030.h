// SPDX-License-Identifier: BSD-2-Clause-Patent
// Portable keyboard, backlight, poll plan and touchpad pointer policy ported directly from Pad 8 Pro
// (lenzhang/UniXNU-xiaomipad8pro Wn8030Piano.h), adapted only to sheng's 21-byte 6-byte/finger touchpad report.
#ifndef SHENG_PIANO_WN8030_H
#define SHENG_PIANO_WN8030_H

#include <stddef.h>
#include <stdint.h>

namespace pianokeyboard {

constexpr uint8_t kModLeftCtrl = 1u << 0, kModRightCtrl = 1u << 4;
constexpr unsigned kKeyboardBytes = 9, kFingers = 3, kPointerBytes = 7;
constexpr unsigned kTouchXMax = 3047, kTouchYMax = 2031;
constexpr int32_t kYNum = 8, kYDen = 9;   // 93.5 x 55.5 mm pad: ~32.6 X counts/mm vs ~36.6 Y counts/mm
constexpr int32_t kRestY = int32_t(kTouchYMax) * 4 / 5;   // bottom fifth nearest the user (after desktop orientation)
constexpr unsigned kGapBuckets = 8;

inline uint16_t le16(const uint8_t *p) { return uint16_t(p[0] | (p[1] << 8)); }
inline void putLe16(uint8_t *p, uint32_t v) { p[0] = uint8_t(v); p[1] = uint8_t(v >> 8); }

inline unsigned gapBucket(uint32_t ms)
{
#if defined(XNU_TARGET_OS_OSX) && XNU_TARGET_OS_OSX
    static const uint32_t below[kGapBuckets - 1] __attribute__((section("__DATA_CONST,__const"))) = {10, 20, 30, 45, 60, 100, 250};
#else
    static const uint32_t below[kGapBuckets - 1] = {10, 20, 30, 45, 60, 100, 250};
#endif
    unsigned bucket = 0;
    while (bucket < kGapBuckets - 1 && ms >= below[bucket]) ++bucket;
    return bucket;
}

// ---------------------------------------------------------------------------------------------------------- the keyboard report
// Verbatim from Pad 8 Pro Wn8030Piano.h:
//   - ErrorRollOver / POST Fail / Undefined (usages 1..3) keeps the previous keys so held keys are not dropped;
//   - optional fnRow turns Right Ctrl + 1..0 - = into F1..F12 (default false);
//   - shortcut() emits a 4-report Left Ctrl + usage chord for 3-finger swipes.
class Keys {
  public:
    bool fnRow = false;

    bool filter(const uint8_t *in, uint8_t out[kKeyboardBytes])
    {
        for (unsigned i = 0; i < kKeyboardBytes; ++i) out[i] = in[i];
        bool rollover = false;
        for (unsigned i = 3; i < kKeyboardBytes; ++i) rollover = rollover || (in[i] >= 1 && in[i] <= 3);
        if (rollover) {
            ++rollovers_;
            for (unsigned i = 0; i < 6; ++i) out[3 + i] = sent_[i];
        } else if (fnRow) {
            uint16_t down = 0;
            for (unsigned i = 3; i < kKeyboardBytes; ++i) {
                const int slot = rowSlot(in[i]);
                if (slot < 0) continue;
                const uint16_t bit = uint16_t(1u << slot);
                down |= bit;
                if ((in[1] & kModRightCtrl) || (fn_ & bit)) {
                    fn_ |= bit;
                    out[i] = uint8_t(0x3a + slot);
                }
            }
            fn_ &= down;
        }
        if (fn_) out[1] &= uint8_t(~kModRightCtrl);
        for (unsigned i = 0; i < 6; ++i) sent_[i] = out[3 + i];
        mods_ = out[1];
        return rollover;
    }

    // Left Control + usage on top of currently held keys: Ctrl down, key down, key up, Ctrl up. Returns 4.
    unsigned shortcut(uint8_t usage, uint8_t out[4][kKeyboardBytes]) const
    {
        for (unsigned r = 0; r < 4; ++r) {
            out[r][0] = 0x05;
            out[r][1] = r < 3 ? uint8_t(mods_ | kModLeftCtrl) : mods_;
            out[r][2] = 0;
            for (unsigned i = 0; i < 6; ++i) out[r][3 + i] = sent_[i];
        }
        unsigned slot = 5;
        for (unsigned i = 0; i < 6; ++i) {
            if (!sent_[i]) { slot = i; break; }
        }
        out[1][3 + slot] = usage;
        return 4;
    }

    static bool newKey(const uint8_t beforeSent[6], const uint8_t *nowReport)
    {
        for (unsigned i = 3; i < kKeyboardBytes; ++i) {
            if (nowReport[i] <= 3) continue;
            bool held = false;
            for (unsigned j = 0; j < 6 && !held; ++j) held = beforeSent[j] == nowReport[i];
            if (!held) return true;
        }
        return false;
    }

    void reset()
    {
        for (uint8_t &k : sent_) k = 0;
        fn_ = 0;
        mods_ = 0;
    }
    const uint8_t *sent() const { return sent_; }
    uint32_t rollovers() const { return rollovers_; }
    uint16_t fnHeld() const { return fn_; }

  private:
    static int rowSlot(uint8_t usage)
    {
        if (usage >= 0x1e && usage <= 0x27) return usage - 0x1e;
        if (usage == 0x2d) return 10;
        if (usage == 0x2e) return 11;
        return -1;
    }
    uint8_t sent_[6] = {}, mods_ = 0;
    uint16_t fn_ = 0;
    uint32_t rollovers_ = 0;
};

// ------------------------------------------------------------------------------------------------------- the pointer for macOS
// Pad 8 Pro Pointer state machine adapted for sheng's 21-byte 6-byte/finger report (wn8030_touchpad_report flips Y toward hinge,
// so invertY=true converts back to desktop Y with 0 at hinge and 2031 near user). Keeps the 60 ms pause-gap restart telemetry.
struct PointerConfig {
    int32_t divide = 1;          // touchpad X counts per pointer count
    int32_t scrollCounts = 5;    // X counts of finger travel per wheel / AC Pan count
    bool tap = true;
    bool invertX = false;
    bool invertY = true;         // sheng's wn8030_touchpad_report flips Y away from user; true restores top=0/bottom=2031
    bool scrollInvert = false;
    bool axisLock = true;
    bool gestures = true;
    bool restingThumb = true;    // contact in bottom fifth while another contact is above it is ignored
    bool keepLoneContact = false;// 1: lone unconfident touching contact kept (keyboard-lone=1)
    uint32_t typingMs = 250, pressSettleMs = 50, pauseGapMs = 60;
    uint32_t tapMs = 220;
    uint32_t tapTravel = 150;    // max travel in X counts
    uint32_t secondaryMm = 45;   // two fingers pressing farther apart than this -> left click
    bool (*tapVeto)(void *context, uint32_t tapMs) = nullptr;
    void *tapVetoContext = nullptr;
};

enum class Swipe : uint8_t { none, up, down, left, right };
#if !defined(KERNEL)
inline const char *swipeName(Swipe s)
{
    switch (s) {
    case Swipe::none: return "none";
    case Swipe::up: return "up (Mission Control, Control+Up)";
    case Swipe::down: return "down (App Expose, Control+Down)";
    case Swipe::left: return "left (the Space to the right, Control+Right)";
    case Swipe::right: return "right (the Space to the left, Control+Left)";
    }
    return "?";
}
#endif
inline uint8_t swipeUsage(Swipe s)
{
    switch (s) {
    case Swipe::up: return 0x52;
    case Swipe::down: return 0x51;
    case Swipe::left: return 0x4f;
    case Swipe::right: return 0x50;
    default: return 0;
    }
}

struct Gesture {
    uint32_t ms = 0, travel = 0, moves = 0, scrolls = 0;
    int32_t x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    uint8_t most = 0, tap = 0;
    bool pressed = false, typing = false;
    uint8_t pressButton = 0;
    bool vetoed = false, rested = false;
    Swipe swipe = Swipe::none;
};

class Pointer {
  public:
    explicit Pointer(const PointerConfig &c = PointerConfig()) : cfg_(c) {}
    void configure(const PointerConfig &c) { cfg_ = c; }
    const PointerConfig &config() const { return cfg_; }
    void keyTyped(uint64_t ms) { lastKeyMs_ = ms; typed_ = true; }

    unsigned touch(const uint8_t *tp, uint64_t ms, uint8_t out[2][kPointerBytes])
    {
        const bool paused = cfg_.pauseGapMs && (ms - atMs_ > cfg_.pauseGapMs);
        ++reports_;
        if (fingers_) ++gaps_[gapBucket(uint32_t(ms - atMs_))];
        atMs_ = ms;

        const uint8_t count = tp[20];
        if (!count && continuation_) {
            continuation_ = false;
            ++continuations_;
            return 0;
        }
        continuation_ = count > kFingers;
        if (count) countSeen_ = true;
        const unsigned slots = count ? (count < kFingers ? count : kFingers) : (countSeen_ ? 0 : kFingers);

        unsigned touching = 0;
        for (unsigned f = 0; f < slots; ++f) {
            if (tp[2 + 6 * f] & 1u) ++touching;
        }

        unsigned seen = 0;
        uint8_t ids[kFingers] = {};
        int32_t xs[kFingers] = {}, ys[kFingers] = {};
        bool palmFollowed = false;
        for (unsigned f = 0; f < slots; ++f) {
            const uint8_t *p = tp + 2 + 6 * f;
            if ((p[0] & 1u) == 0) continue;
            const bool confident = (p[0] & 4u) != 0 || (cfg_.keepLoneContact && touching == 1);
            if (!confident) {
                for (unsigned i = 0; i < fingers_; ++i) palmFollowed = palmFollowed || contacts_[i] == p[1];
                continue;
            }
            const uint16_t rawX = le16(p + 2), rawY = le16(p + 4);
            const uint16_t cx = rawX > kTouchXMax ? kTouchXMax : rawX;
            const uint16_t cy = rawY > kTouchYMax ? kTouchYMax : rawY;
            // Keep contacts sorted by ascending ID so slot permutation does not cause a false delta or reset.
            unsigned at = seen;
            for (; at && ids[at - 1] > p[1]; --at) {
                ids[at] = ids[at - 1];
                xs[at] = xs[at - 1];
                ys[at] = ys[at - 1];
            }
            ids[at] = p[1];
            xs[at] = cfg_.invertX ? int32_t(kTouchXMax) - int32_t(cx) : int32_t(cx);
            ys[at] = cfg_.invertY ? int32_t(kTouchYMax) - int32_t(cy) : int32_t(cy);
            ++seen;
        }

        // Resting thumb rejection: bottom fifth (ys >= kRestY) while another contact is above it.
        bool keep[kFingers] = {true, true, true};
        bool above = false;
        for (unsigned f = 0; f < seen; ++f) above = above || ys[f] < kRestY;
        bool rested = false;
        if (cfg_.restingThumb && above) {
            for (unsigned f = 0; f < seen; ++f) {
                if (ys[f] >= kRestY) { keep[f] = false; rested = true; }
            }
        }
        unsigned fingers = 0;
        int32_t x = 0, y = 0, fx[2] = {}, fy[2] = {};
        for (unsigned f = 0; f < seen; ++f) {
            if (!keep[f]) continue;
            ids[fingers] = ids[f];
            if (fingers < 2) { fx[fingers] = xs[f]; fy[fingers] = ys[f]; }
            x += xs[f];
            y += ys[f];
            ++fingers;
        }
        if (fingers) { x /= int32_t(fingers); y /= int32_t(fingers); }

        if (fingers && !fingers_ && !touching_) {
            touching_ = true;
            downMs_ = ms;
            travel_ = 0;
            most_ = 0;
            pressed_ = false;
            palm_ = false;
            ignored_ = cfg_.typingMs && typed_ && (ms - lastKeyMs_ < cfg_.typingMs);
            g_ = Gesture{};
            g_.x0 = x;
            g_.y0 = y;
            g_.typing = ignored_;
        }
        if (palmFollowed) palm_ = true;
        if (fingers) { g_.x1 = x; g_.y1 = y; }
        if (fingers > most_) most_ = uint8_t(fingers);
        if (rested && touching_) g_.rested = true;
        if (tp[1] & 3u) pressed_ = true;

        const bool press = (tp[1] & 1u) != 0;
        if (press && !pressing_) {
            const int32_t dx = fx[1] - fx[0], dy = (fy[1] - fy[0]) * kYNum / kYDen;
            const int64_t far = int64_t(cfg_.secondaryMm) * 33;
            const bool apart = fingers == 2 && (int64_t(dx) * dx + int64_t(dy) * dy > far * far);
            pressButton_ = (fingers >= 2 && !apart) ? 2 : 1;
            if (touching_) g_.pressButton = pressButton_;
        }
        if (press != pressing_ || ((tp[1] & 2u) != 0) != second_) settleUntil_ = ms + cfg_.pressSettleMs;
        pressing_ = press;
        second_ = (tp[1] & 2u) != 0;
        uint8_t buttons = press ? pressButton_ : 0;
        if (tp[1] & 2u) buttons |= 2u;

        bool same = fingers == fingers_;
        for (unsigned f = 0; same && f < fingers; ++f) same = ids[f] == contacts_[f];
        if (fingers && same && paused) ++restarts_;

        const bool threeSwipe = fingers >= 3;
        if (fingers != fingers_ || !same || paused) {
            moveX_ = moveY_ = scrollX_ = scrollY_ = 0;
            lockX_ = lockY_ = 0;
            lock_ = 0;
            swipeX0_ = x;
            swipeY0_ = y;
        } else if (fingers) {
            const int32_t dx = x - x_, dy = y - y_;
            travel_ += uint32_t((dx < 0 ? -dx : dx) + ((dy < 0 ? -dy : dy) * kYNum + kYDen / 2) / kYDen);
            if (!ignored_ && ms >= settleUntil_) {
                if (threeSwipe) swipe(x, y);
                if (threeSwipe && cfg_.gestures) {
                } else if (fingers == 1) {
                    moveX_ += dx;
                    moveY_ += kYNum * dy;
                } else {
                    const int32_t sign = cfg_.scrollInvert ? -1 : 1;
                    scrollX_ += sign * dx;
                    scrollY_ -= sign * kYNum * dy;
                    lock(dx, dy);
                }
            }
        }

        bool tapped = !fingers && touching_ && cfg_.tap && !pressed_ && !ignored_ && !palm_ &&
                      (ms - downMs_ <= cfg_.tapMs) && (travel_ <= cfg_.tapTravel);
        if (tapped && cfg_.tapVeto && cfg_.tapVeto(cfg_.tapVetoContext, uint32_t(ms - downMs_))) {
            tapped = false;
            g_.vetoed = true;
            ++vetoes_;
        }
        const uint8_t tapButton = most_ >= 3 ? 4 : most_ == 2 ? 2 : 1;
        if (!fingers && touching_) {
            touching_ = false;
            swipeDone_ = false;
            g_.ms = uint32_t(ms - downMs_);
            g_.travel = travel_;
            g_.most = most_;
            g_.pressed = pressed_;
            g_.tap = (tapped && !buttons) ? tapButton : 0;
            ++gestures_;
            gestureDone_ = true;
        }

        fingers_ = uint8_t(fingers);
        for (unsigned f = 0; f < fingers; ++f) contacts_[f] = ids[f];
        x_ = x;
        y_ = y;

        if (lock_ == 1) scrollX_ = 0;
        if (lock_ == 2) scrollY_ = 0;
        unsigned n = 0;
        const int32_t mx = take(&moveX_, cfg_.divide, 32767), my = take(&moveY_, kYDen * cfg_.divide, 32767);
        const bool scrolling = !cfg_.axisLock || lock_;
        const int32_t sy = scrolling ? take(&scrollY_, kYDen * cfg_.scrollCounts, 127) : 0;
        const int32_t sx = scrolling ? take(&scrollX_, cfg_.scrollCounts, 127) : 0;
        if (mx || my || sy || sx || buttons != buttons_) {
            buttons_ = buttons;
            put(out[n++], buttons, mx, my, sy, sx);
            ++sent_;
            if (mx || my) ++g_.moves;
            if (sx || sy) ++g_.scrolls;
        }
        if (tapped && !buttons) {
            n = 0;
            put(out[n++], tapButton, 0, 0, 0, 0);
            put(out[n++], 0, 0, 0, 0, 0);
            sent_ += 2;
            ++taps_;
        }
        return n;
    }

    bool takeGesture(Gesture *g)
    {
        if (!gestureDone_) return false;
        gestureDone_ = false;
        *g = g_;
        return true;
    }
    Swipe takeSwipe()
    {
        const Swipe s = pendingSwipe_;
        pendingSwipe_ = Swipe::none;
        return s;
    }
    static void release(uint8_t out[kPointerBytes]) { put(out, 0, 0, 0, 0, 0); }
    void reset()
    {
        const uint32_t taps = taps_, gestures = gestures_, continuations = continuations_, swipes = swipes_, vetoes = vetoes_;
        const uint32_t reports = reports_, sent = sent_, restarts = restarts_;
        const bool countSeen = countSeen_;
        uint32_t savedGaps[kGapBuckets] = {};
        for (unsigned i = 0; i < kGapBuckets; ++i) savedGaps[i] = gaps_[i];
        *this = Pointer(cfg_);
        vetoes_ = vetoes;
        taps_ = taps;
        gestures_ = gestures;
        continuations_ = continuations;
        swipes_ = swipes;
        reports_ = reports;
        sent_ = sent;
        restarts_ = restarts;
        for (unsigned i = 0; i < kGapBuckets; ++i) gaps_[i] = savedGaps[i];
        countSeen_ = countSeen;
    }

    uint32_t taps() const { return taps_; }
    uint32_t vetoes() const { return vetoes_; }
    uint32_t gestures() const { return gestures_; }
    uint32_t continuations() const { return continuations_; }
    uint32_t swipes() const { return swipes_; }
    uint32_t reports() const { return reports_; }
    uint32_t sent() const { return sent_; }
    uint32_t restarts() const { return restarts_; }
    const uint32_t *gaps() const { return gaps_; }

  private:
    void swipe(int32_t x, int32_t y)
    {
        if (swipeDone_) return;
        const int32_t dx = x - swipeX0_, dy = (y - swipeY0_) * kYNum / kYDen;
        const int32_t ax = dx < 0 ? -dx : dx, ay = dy < 0 ? -dy : dy;
        Swipe s = Swipe::none;
        if (ay >= 300 && ay >= 2 * ax) s = dy < 0 ? Swipe::up : Swipe::down;
        else if (ax >= 300 && ax >= 2 * ay) s = dx < 0 ? Swipe::left : Swipe::right;
        if (s == Swipe::none) return;
        swipeDone_ = true;
        g_.swipe = s;
        ++swipes_;
        if (cfg_.gestures) pendingSwipe_ = s;
    }
    void lock(int32_t dx, int32_t dy)
    {
        if (!cfg_.axisLock || lock_) return;
        lockX_ += uint32_t(dx < 0 ? -dx : dx);
        lockY_ += uint32_t(((dy < 0 ? -dy : dy) * kYNum) / kYDen);
        if (lockX_ + lockY_ < 40) return;
        lock_ = lockY_ >= 2 * lockX_ ? 1 : lockX_ >= 2 * lockY_ ? 2 : 3;
    }
    static int32_t take(int32_t *pending, int32_t unit, int32_t limit)
    {
        int32_t steps = *pending / unit;
        if (steps > limit) steps = limit;
        if (steps < -limit) steps = -limit;
        *pending -= steps * unit;
        return steps;
    }
    static void put(uint8_t *o, uint8_t buttons, int32_t x, int32_t y, int32_t wheel, int32_t pan)
    {
        o[0] = buttons;
        putLe16(o + 1, uint16_t(int16_t(x)));
        putLe16(o + 3, uint16_t(int16_t(y)));
        o[5] = uint8_t(int8_t(wheel));
        o[6] = uint8_t(int8_t(pan));
    }
    PointerConfig cfg_;
    uint8_t contacts_[kFingers] = {};
    uint8_t fingers_ = 0, buttons_ = 0, most_ = 0, pressButton_ = 1, lock_ = 0;
    bool touching_ = false, pressed_ = false, pressing_ = false, second_ = false, continuation_ = false, countSeen_ = false;
    bool gestureDone_ = false, ignored_ = false, typed_ = false, swipeDone_ = false, palm_ = false;
    int32_t x_ = 0, y_ = 0, moveX_ = 0, moveY_ = 0, scrollX_ = 0, scrollY_ = 0, swipeX0_ = 0, swipeY0_ = 0;
    uint32_t travel_ = 0, lockX_ = 0, lockY_ = 0;
    uint64_t downMs_ = 0, lastKeyMs_ = 0, settleUntil_ = 0, atMs_ = 0;
    uint32_t taps_ = 0, gestures_ = 0, continuations_ = 0, swipes_ = 0, vetoes_ = 0;
    uint32_t reports_ = 0, sent_ = 0, restarts_ = 0;
    uint32_t gaps_[kGapBuckets] = {};
    Swipe pendingSwipe_ = Swipe::none;
    Gesture g_ = {};
};

// ------------------------------------------------------------------------------------------------- the cover's backlight
// Verbatim from Pad 8 Pro Wn8030Piano.h (command 0x23, 0..100, >100 disabled).
class Backlight {
  public:
    void configure(uint32_t level, uint32_t idleMs)
    {
        level_ = level;
        idleMs_ = idleMs;
        shown_ = -1;
    }
    int attach(uint64_t ms)
    {
        lastInputMs_ = ms;
        if (level_ > 100) return -1;
        return show(int(level_));
    }
    int input(uint64_t ms)
    {
        lastInputMs_ = ms;
        if (level_ > 100 || shown_ != 0 || !level_) return -1;
        return show(int(level_));
    }
    int tick(uint64_t ms)
    {
        if (!idleMs_ || level_ > 100 || !level_ || shown_ <= 0 || ms - lastInputMs_ < idleMs_) return -1;
        return show(0);
    }
    int consumer(uint16_t usage, uint64_t ms)
    {
        lastInputMs_ = ms;
        uint32_t level = level_ > 100 ? 0 : level_;
        if (usage == 0x79) level = level + 10 > 100 ? 100 : level + 10;
        else if (usage == 0x7a) level = level < 10 ? 0 : level - 10;
        else if (usage == 0x7c) { if (level) { toggled_ = level; level = 0; } else level = toggled_ ? toggled_ : 50; }
        else return -1;
        level_ = level;
        return show(int(level));
    }
    void detach() { shown_ = -1; }
    uint32_t level() const { return level_; }

  private:
    int show(int v)
    {
        if (v == shown_) return -1;
        shown_ = v;
        return v;
    }
    uint32_t level_ = 0, idleMs_ = 0, toggled_ = 0;
    int shown_ = -1;
    uint64_t lastInputMs_ = 0;
};

// ------------------------------------------------------------------------------------------------------------ the poll plan
// Verbatim from Pad 8 Pro Wn8030Piano.h: active (1s insurance poll), idle (5s), slow (10s).
enum class PollMode : uint8_t { active, idle, slow };
inline const char *pollModeName(PollMode m) { return m == PollMode::active ? "active" : m == PollMode::idle ? "idle" : "slow"; }
struct PollConfig {
    uint32_t activeMs = 4, idleMs = 30, slowMs = 200, holdMs = 2000, hallMs = 500;
};
class PollPlan {
  public:
    struct Minute {
        uint32_t wakes, inMode[3], ms;
    };
    void configure(const PollConfig &c) { cfg_ = c; }
    const PollConfig &config() const { return cfg_; }
    void activity(uint64_t now)
    {
        last_ = now;
        seen_ = true;
    }
    PollMode mode(uint64_t now, bool attached, bool closed) const
    {
        if (seen_ && now - last_ < cfg_.holdMs) return PollMode::active;
        return !attached || closed ? PollMode::slow : PollMode::idle;
    }
    uint32_t sleepMs(PollMode m) const { return m == PollMode::active ? cfg_.activeMs : m == PollMode::idle ? cfg_.idleMs : cfg_.slowMs; }
    uint32_t readEveryMs(PollMode m) const { return m == PollMode::active ? 1000 : m == PollMode::idle ? 5000 : 10000; }
    uint32_t waitMs(PollMode m, bool irqLive, bool burst, bool erroring) const
    {
        if (burst) return 0;
        if (erroring) return 1000;
        if (!irqLive) return sleepMs(m);
        const uint32_t every = readEveryMs(m);
        return every < cfg_.hallMs ? every : cfg_.hallMs;
    }
    void wake(uint64_t now, PollMode m)
    {
        if (!started_) {
            started_ = true;
            windowStart_ = now;
        }
        ++wakes_;
        ++inMode_[unsigned(m)];
        if (now - windowStart_ >= 60000) {
            minute_ = {wakes_, {inMode_[0], inMode_[1], inMode_[2]}, uint32_t(now - windowStart_)};
            ready_ = true;
            windowStart_ = now;
            wakes_ = 0;
            inMode_[0] = inMode_[1] = inMode_[2] = 0;
        }
    }
    bool takeMinute(Minute *m)
    {
        if (!ready_) return false;
        ready_ = false;
        *m = minute_;
        return true;
    }

  private:
    PollConfig cfg_;
    uint64_t last_ = 0, windowStart_ = 0;
    bool seen_ = false, ready_ = false, started_ = false;
    uint32_t wakes_ = 0, inMode_[3] = {};
    Minute minute_ = {};
};

}  // namespace pianokeyboard

#endif
