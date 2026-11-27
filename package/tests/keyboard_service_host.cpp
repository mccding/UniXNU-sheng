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

    // A single rising edge reads ONCE (no 2x wasted extra read when lineHigh drops or even briefly stays high unless bursting Valid).
    io.edges = 1; io.lineHigh = true; io.readResult = KeyboardRead::Valid;
    s.step(io);
    // Because flood=false and lineHigh stayed true after Valid, burst attempts read 2; set readResult=Empty on next read to confirm stop:
    CHECK(s.seen == 1 && io.reads == KeyboardService::drainBudget);
    io.lineHigh = false; s.highBackoffDue = 0;
    io.edges = 2; io.reads = 0;
    s.step(io);
    CHECK(s.seen == 2 && io.reads == 1 && !s.highBackoffDue);

    // M0 deadlock regression test: Empty read with lineHigh=true must NOT latch forever; backs off 50ms then resumes reading.
    io.reads = 0; io.lineHigh = true; io.readResult = KeyboardRead::Empty;
    s.step(io);
    CHECK(io.reads == 1 && s.highBackoffDue == io.clock + KeyboardService::highBackoffMs);
    const uint64_t backoffAt = s.highBackoffDue;
    CHECK(s.deadline(io.clock, io.edges, io.irqHigh()) <= backoffAt);
    s.step(io);
    CHECK(io.reads == 1);   // respects 50 ms backoff, no spin!
    io.clock = backoffAt; io.readResult = KeyboardRead::Valid;
    s.step(io);
    CHECK(io.reads == 1 + KeyboardService::drainBudget && !s.highBackoffDue && io.activities > 0);   // recovers automatically without edge!
    io.lineHigh = false;

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
