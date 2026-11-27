// SPDX-License-Identifier: BSD-2-Clause-Patent
// Scheduling policy for the WN8030 worker (Pad 8 Pro serving loop adapted onto GeniI2c).
// Eliminates the permanent irqEmpty latch deadlock: uses 3-state reads, bursts only on Valid, and applies 50 ms backoff on high IRQ.
#ifndef SHENG_KEYBOARD_SERVICE_H
#define SHENG_KEYBOARD_SERVICE_H
#include <stdint.h>

enum class KeyboardRead : uint8_t { Error = 0, Empty = 1, Valid = 2 };

struct KeyboardService {
    static constexpr uint32_t keepaliveMs = 12000, retryMs = 100, drainBudget = 8;
    uint32_t seen = 0, readFailures = 0, controlFailures = 0, recoveryTries = 0;
    uint64_t keepalive = 0, retryDue = 0, recoveryDue = 0, pollDue = 0;
    bool statePending = false, recovering = false, irqWasLow = true;

    template <typename IO> void step(IO &io) {
        if (recovering) {
            if (io.now() < recoveryDue) return;
            if (!io.recover()) {
                if (recoveryTries < 5) ++recoveryTries;
                recoveryDue = io.now() + (1000u << recoveryTries);
                return;
            }
            recovering = false;
            recoveryTries = readFailures = controlFailures = 0;
            retryDue = 0;
            irqWasLow = !io.irqHigh();
            statePending = io.attached();
            keepalive = io.now();
            pollDue = io.now() + io.pollIntervalMs();
        }
        if (retryDue && io.now() < retryDue) return;
        retryDue = 0;
        // Drain edge-triggered and buffered frames: never follow an Empty read on the same wake (prevents 2x reads and irqEmpty lockup).
        bool prevValid = false;
        bool budgetHit = false;
        for (unsigned n = 0; n < drainBudget; ++n) {
            const bool hadEdge = seen != io.irqs();
            const bool highReady = io.irqHigh() && (n == 0 ? irqWasLow : prevValid);
            if (!hadEdge && !highReady) break;
            const KeyboardRead r = io.read();
            if (r == KeyboardRead::Error) {
                prevValid = false;
                ++readFailures;
                retryDue = io.now() + retryMs;
                break;
            }
            if (hadEdge) io.consumed(++seen);
            readFailures = 0;
            if (r == KeyboardRead::Valid) {
                prevValid = true;
                io.activity(io.now());
                pollDue = io.now() + io.pollIntervalMs();
                if (n + 1 == drainBudget && io.irqHigh()) budgetHit = true;
            } else {
                prevValid = false;
                break;
            }
        }
        if (!io.irqHigh() || budgetHit) {
            irqWasLow = true;
        } else {
            irqWasLow = false;
        }
        // Poll once per PollPlan interval (1s active / 5s idle / 10s slow) as insurance against a missed rising edge.
        if (!retryDue && io.now() >= pollDue) {
            pollDue = io.now() + io.pollIntervalMs();
            if (seen == io.irqs()) {
                const KeyboardRead r = io.read();
                if (r == KeyboardRead::Error) {
                    ++readFailures;
                    retryDue = io.now() + retryMs;
                } else {
                    readFailures = 0;
                    if (r == KeyboardRead::Valid) {
                        io.activity(io.now());
                        pollDue = io.now() + io.pollIntervalMs();
                    }
                }
            }
        }
        if (!retryDue && io.now() >= keepalive) {
            if (!io.attached() || io.power()) {
                controlFailures = 0;
                keepalive = io.now() + keepaliveMs;
            } else {
                if (++controlFailures == 1) {io.release(); io.wake(); statePending = io.attached();}
                retryDue = keepalive = io.now() + retryMs;
            }
        }
        if (!retryDue && statePending) {
            if (io.apply()) {statePending = false; controlFailures = 0;}
            else {++controlFailures; retryDue = io.now() + retryMs;}
        }
        if (readFailures >= 3 || controlFailures >= 3) {
            io.release();
            recovering = true;
            recoveryDue = io.now() + 1000;
        }
    }

    uint64_t deadline(uint64_t now, uint32_t irqs, bool irqHigh = false) const {
        if (recovering) return recoveryDue;
        if (retryDue) return retryDue;
        if (seen != irqs || statePending || (irqHigh && irqWasLow)) return now;
        uint64_t due = keepalive;
        if (pollDue && pollDue < due) due = pollDue;
        return due;
    }
};
#endif
