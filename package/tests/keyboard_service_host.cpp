// SPDX-License-Identifier: BSD-2-Clause-Patent
#include "../KeyboardService.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>

static unsigned checks;
#define CHECK(x) do {if (!(x)) {std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1);} ++checks;} while (0)
struct Mock {
    uint64_t clock = 1;
    uint32_t edges = 0, reads = 0, powers = 0, applies = 0, wakes = 0, releases = 0, reloads = 0, consumedCount = 0;
    uint32_t pollMs = 5000, activities = 0;
    bool on = true, powerOK = true, applyOK = true, recoverOK = true, flood = false, lineHigh = false;
    KeyboardRead readResult = KeyboardRead::Valid;
    int emptyThenValid = 0;
    KeyboardService *service = nullptr;
    uint64_t now() const {return clock;}
    uint32_t irqs() const {return edges;}
    bool irqHigh() const {return lineHigh;}
    bool attached() const {return on;}
    KeyboardRead read() {
        ++reads; ++clock;
        if (flood && readResult == KeyboardRead::Valid) ++edges;
        if (emptyThenValid > 0) {
            if (--emptyThenValid > 0) return KeyboardRead::Empty;
            return KeyboardRead::Valid;
        }
        return readResult;
    }
    void consumed(uint32_t seen) {consumedCount = seen;}
    void activity(uint64_t) {++activities;}
    uint32_t pollIntervalMs() const {return pollMs;}
    bool power() {++powers; ++clock; return powerOK;}
    bool apply() {++applies; ++clock; return applyOK;}
    void wake() {++wakes; clock += 35;}
    void release() {++releases;}
    bool recover() {++reloads; clock += 600; if (recoverOK) service->seen = edges; return recoverOK;}
};
int main() {
    KeyboardService s;
    Mock io; io.service = &s;
    s.keepalive = 12000;
    s.pollDue = 5000;

    // A single rising edge drains valid frames up to drainBudget; stayed-high IRQ after Valid stops on Empty without 20 Hz hammering.
    io.edges = 1; io.lineHigh = true; io.emptyThenValid = 2;  // 1st read Empty -> irqWasLow cleared
    s.step(io);
    CHECK(s.seen == 1 && io.reads == 1 && !s.irqWasLow);
    CHECK(s.deadline(io.clock, io.edges, io.irqHigh()) == s.pollDue);
    // While PinIrq stays high after an Empty read, extra steps before pollDue do ZERO I2C reads (no 20 Hz bus hammering).
    io.clock += 200;
    s.step(io);
    CHECK(io.reads == 1 && s.deadline(io.clock, io.edges, io.irqHigh()) == s.pollDue);
    // Insurance poll at pollDue reads even if PinIrq stayed high (and picks up the now-Valid frame).
    io.clock = s.pollDue;
    s.step(io);
    CHECK(io.reads == 2 && io.activities > 0);
    io.lineHigh = false; io.emptyThenValid = 0;
    s.step(io);
    CHECK(s.irqWasLow && io.reads == 2);

    // Missed rising edge (irqWasLow == true, lineHigh == true, irqs == seen): deadline is immediate and step reads.
    io.lineHigh = true; io.readResult = KeyboardRead::Valid; io.reads = 0;
    CHECK(s.deadline(io.clock, io.edges, io.irqHigh()) == io.clock);
    s.step(io);
    CHECK(io.reads == KeyboardService::drainBudget && s.irqWasLow);  // budget hit keeps irqWasLow true so remaining frames drain immediately
    CHECK(s.deadline(io.clock, io.edges, io.irqHigh()) == io.clock);
    // Drain remaining 4 valid frames then go low/idle.
    struct BurstMock : Mock {
        unsigned validLeft = 0;
        KeyboardRead read() {
            ++reads; ++clock;
            if (validLeft > 0) { --validLeft; return KeyboardRead::Valid; }
            return KeyboardRead::Empty;
        }
    } burst;
    KeyboardService sb; burst.service = &sb; sb.keepalive = 12000; sb.pollDue = 5000;
    burst.edges = 1; burst.lineHigh = true; burst.validLeft = 12;
    sb.step(burst);
    CHECK(sb.seen == 1 && burst.reads == KeyboardService::drainBudget && sb.irqWasLow);
    CHECK(sb.deadline(burst.clock, burst.edges, burst.irqHigh()) == burst.clock);
    sb.step(burst);
    CHECK(burst.reads == 13 && !sb.irqWasLow);   // 4 valid + 1 Empty stops burst and clears irqWasLow
    burst.lineHigh = false;

    io.edges = 2; io.reads = 0; io.lineHigh = false; s.irqWasLow = true;
    s.step(io);
    CHECK(s.seen == 2 && io.reads == 1);

    // Failed read does not consume its edge; retry without any new interrupt succeeds.
    io.edges = 3; io.readResult = KeyboardRead::Error; io.reads = 0;
    s.step(io);
    CHECK(s.seen == 2 && io.reads == 1 && s.retryDue > io.clock && !s.recovering);
    CHECK(s.deadline(io.clock, io.edges) == s.retryDue);
    s.step(io); CHECK(io.reads == 1);
    io.clock = s.retryDue; io.readResult = KeyboardRead::Valid; s.step(io);
    CHECK(s.seen == 3 && io.reads == 2 && !s.retryDue && !s.readFailures);
    CHECK(s.deadline(io.clock, io.edges) > io.clock);

    // Input flood yields after drainBudget, and due power command still runs.
    io.flood = true; ++io.edges; io.clock = s.keepalive; s.pollDue = io.clock;
    const unsigned before = io.reads; s.step(io);
    CHECK(io.reads == before + KeyboardService::drainBudget && io.powers == 1);
    CHECK(s.deadline(io.clock, io.edges) == io.clock);
    io.flood = false; s.step(io); CHECK(s.seen == io.edges);

    // Quiet healthy controller is never reset, even across many idle intervals and insurance polls.
    for (unsigned i = 0; i < 100; ++i) {io.clock = s.keepalive; s.step(io); CHECK(!s.recovering && !io.reloads);}
    CHECK(io.powers == 101 && io.reads >= 100);

    // Failed keepalive: GPIO wake, bounded retries, then reload with backoff.
    io.powerOK = false; io.clock = s.keepalive;
    s.step(io); CHECK(io.wakes == 1 && s.controlFailures == 1 && !s.recovering);
    io.clock = s.retryDue; s.step(io); CHECK(s.controlFailures == 2);
    io.clock = s.retryDue; s.step(io); CHECK(s.recovering && io.reloads == 0);
    const uint64_t firstRecovery = s.recoveryDue;
    s.step(io); CHECK(!io.reloads);
    io.clock = firstRecovery; io.recoverOK = false; s.step(io);
    CHECK(io.reloads == 1 && s.recoveryDue == io.clock + 2000 && s.recovering);
    for (unsigned i = 0; i < 6; ++i) {io.clock = s.recoveryDue; s.step(io);}
    CHECK(s.recoveryDue == io.clock + 32000);
    io.clock = s.recoveryDue; io.recoverOK = io.powerOK = true; s.step(io);
    CHECK(!s.recovering && !s.retryDue && !s.controlFailures && !s.readFailures && io.applies == 1);

    // Repeated read errors reload and preserve pending edge.
    io.readResult = KeyboardRead::Error; ++io.edges;
    const uint32_t seen = s.seen;
    s.step(io); io.clock = s.retryDue; s.step(io); io.clock = s.retryDue; s.step(io);
    CHECK(s.recovering && s.seen == seen && s.readFailures == 3);
    io.readResult = KeyboardRead::Valid; io.clock = s.recoveryDue; s.step(io);
    CHECK(!s.recovering && s.seen == io.edges);

    // Detached covers get periodic insurance polls without power writes.
    io.on = false; const uint32_t power = io.powers, reads = io.reads;
    io.clock = s.pollDue; s.step(io);
    CHECK(io.powers == power && io.reads == reads + 1);

    // Counter wrap remains a single pending event.
    s.seen = UINT32_MAX; io.edges = 0; s.step(io); CHECK(!s.seen && !io.consumedCount);

    std::printf("PASS keyboard service: %u checks\n", checks);
}
