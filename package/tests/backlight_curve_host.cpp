// SPDX-License-Identifier: BSD-2-Clause-Patent
// Host test for ShengBacklightCurve.h (Pad 8 Pro KTZ8866 exponential curve, BL_EN bit-6 helper, and Steps ladder).
#include "../ShengBacklightCurve.h"
#include <cstdio>
#include <cstdlib>

using namespace shengbacklight;

static unsigned checks;
#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); std::exit(1); } ++checks; } while (0)

int main()
{
    // BL_EN preserves sink bits 5:0, sets/clears bit 6, and never sets bit 7 (SOFTWARE_RESET).
    CHECK(enableValue(0x7f, false) == 0x3f);
    CHECK(enableValue(0x3f, true) == 0x7f);
    CHECK(enableValue(0xff, true) == 0x7f);
    CHECK(enableValue(0xff, false) == 0x3f);
    CHECK(enableValue(0x15, true) == 0x55);
    CHECK(idIsKtz8866(0x22) && !idIsKtz8866(0x00) && !idIsKtz8866(0xff));
    CHECK(isExponential(0x52) && !isExponential(0x5a));

    // Exponential curve endpoints & smoothness across 2..600 nits.
    CHECK(codeForNits(0, true) == 0);
    CHECK(codeForNits(-100, true) == 0);
    CHECK(codeForNits(int32_t(kMinFixed), true) == kFloorCode);
    CHECK(codeForNits(int32_t(kMaxFixed), true) == kFullScale);
    const uint16_t c25 = codeForNits(150 << 16, true);
    const uint16_t c50 = codeForNits(301 << 16, true);
    const uint16_t c75 = codeForNits(450 << 16, true);
    CHECK(c25 >= 1500 && c25 <= 1620);
    CHECK(c50 >= 1760 && c50 <= 1850);
    CHECK(c75 >= 1920 && c75 <= 1990);
    uint16_t prev = kFloorCode;
    for (uint32_t nits = kMinNits; nits <= kMaxNits; ++nits) {
        const uint16_t c = codeForNits(int32_t(nits << 16), true);
        CHECK(c >= prev && c <= kFullScale);
        prev = c;
    }

    // Millinit inverse is monotonic and round-trips cleanly within +/- 3 codes.
    CHECK(milliNitsForCode(0, true) == 0);
    CHECK(milliNitsForCode(kFloorCode, true) == kMinNits * 1000u);
    CHECK(milliNitsForCode(kFullScale, true) == kMaxNits * 1000u);
    uint32_t prevMn = kMinNits * 1000u;
    for (uint16_t c = kFloorCode; c <= kFullScale; ++c) {
        const uint32_t mn = milliNitsForCode(c, true);
        CHECK(mn >= prevMn && mn <= kMaxNits * 1000u);
        prevMn = mn;
        const int32_t fixed = int32_t((uint64_t(mn) << 16) / 1000u);
        const uint16_t back = codeForNits(fixed, true);
        const int diff = int(back) - int(c);
        CHECK(diff >= -25 && diff <= 25);
    }

    // Linear fallback also maps endpoints and stays monotonic.
    CHECK(codeForNits(int32_t(kMinFixed), false) == kFloorCode);
    CHECK(codeForNits(int32_t(kMaxFixed), false) == kFullScale);
    CHECK(milliNitsForCode(kFloorCode, false) == kMinNits * 1000u);
    CHECK(milliNitsForCode(kFullScale, false) == kMaxNits * 1000u);

    // Steps ladder: exponential (17 equal steps) & linear (cube-root steps).
    Steps expSteps, linSteps;
    expSteps.build(kFloorCode, true, kFullScale);
    CHECK(expSteps.count() == 17 && expSteps.at(0) == kFloorCode && expSteps.at(16) == kFullScale);
    CHECK(expSteps.down(kFloorCode) == kFloorCode && expSteps.up(kFullScale) == kFullScale);
    CHECK(expSteps.up(kFloorCode) > kFloorCode && expSteps.down(kFullScale) < kFullScale);
    linSteps.build(kFloorCode, false, kFullScale);
    CHECK(linSteps.count() > 2 && linSteps.at(0) == kFloorCode && linSteps.at(linSteps.count() - 1) == kFullScale);
    for (unsigned i = 1; i < linSteps.count(); ++i) CHECK(linSteps.at(i) > linSteps.at(i - 1));

    std::printf("PASS backlight curve: %u checks (c25=%u c50=%u c75=%u)\n", checks, c25, c50, c75);
    return 0;
}
