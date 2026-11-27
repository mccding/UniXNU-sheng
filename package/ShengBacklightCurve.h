// SPDX-License-Identifier: BSD-2-Clause-Patent
// Portable KTZ8866 exponential/linear brightness mapping, BL_EN helper and perceptual step ladder, ported from Pad 8 Pro
// (lenzhang/UniXNU-xiaomipad8pro Ktz8866.h + AndroidBrightness.h + NativeBrightness.h) for Xiaomi Pad 6S Pro (sheng).
#ifndef SHENG_BACKLIGHT_CURVE_H
#define SHENG_BACKLIGHT_CURVE_H

#include <stdint.h>
#include "PianoAndroidBrightness.h"

namespace shengbacklight {

constexpr uint8_t kAddress = 0x11;
enum : uint8_t {
    kRegId = 0x01,
    kRegCfg1 = 0x02,
    kRegBrtLsb = 0x04,
    kRegBrtMsb = 0x05,
    kRegEn = 0x08,
};
constexpr uint8_t kEnSoftReset = 0x80, kEnBacklight = 0x40, kEnSinks = 0x3f, kCfg1LinearMap = 0x08;
constexpr uint16_t kFloorCode = PA_SDR_MIN_CODE;   // 178: minimum lit KTZ8866 code
constexpr uint16_t kFullScale = 2047;

constexpr uint32_t kMinNits = 2, kMidNits = 301, kMaxNits = 600;
constexpr uint32_t kMinFixed = kMinNits << 16, kMidFixed = kMidNits << 16, kMaxFixed = kMaxNits << 16;

inline bool idIsKtz8866(uint8_t id) { return (id & 0xf8u) == 0x20u; }
inline bool isExponential(uint8_t cfg1) { return (cfg1 & kCfg1LinearMap) == 0; }
// Preserve sink enables in bits 5:0, toggle bit 6 (BL_EN), never set bit 7 (SOFTWARE_RESET).
inline uint8_t enableValue(uint8_t found, bool on) { return uint8_t((found & kEnSinks) | (on ? kEnBacklight : 0)); }

// Maps signed 16.16 nits onto the 11-bit KTZ8866 register code [kFloorCode..kFullScale] (0 means dark / BL_EN off).
inline uint16_t codeForNits(int32_t fixed, bool exponential)
{
    if (fixed <= 0) return 0;
    uint32_t f = uint32_t(fixed);
    if (f < kMinFixed) f = kMinFixed;
    if (f > kMaxFixed) f = kMaxFixed;
    if (exponential) {
        const uint16_t sdr = pa_sdr_code(f);
        constexpr uint32_t kInSpan = PA_SDR_MAX_CODE - PA_SDR_MIN_CODE;
        constexpr uint32_t kOutSpan = kFullScale - kFloorCode;
        const uint32_t scaled = kFloorCode + (uint32_t(sdr - PA_SDR_MIN_CODE) * kOutSpan + kInSpan / 2) / kInSpan;
        return uint16_t(scaled < kFloorCode ? kFloorCode : scaled > kFullScale ? kFullScale : scaled);
    }
    const uint64_t span = kMaxFixed - kMinFixed;
    const uint64_t scaled = kFloorCode + ((uint64_t(f - kMinFixed) * (kFullScale - kFloorCode) + span / 2) / span);
    return uint16_t(scaled < kFloorCode ? kFloorCode : scaled > kFullScale ? kFullScale : scaled);
}

// Monotonic inverse of codeForNits(): maps an 11-bit KTZ8866 code back to millinits [2000..600000].
inline uint32_t milliNitsForCode(uint16_t code, bool exponential)
{
    if (code == 0) return 0;
    if (code <= kFloorCode) return kMinNits * 1000u;
    if (code >= kFullScale) return kMaxNits * 1000u;
    if (exponential) {
        constexpr uint32_t kInSpan = PA_SDR_MAX_CODE - PA_SDR_MIN_CODE;
        constexpr uint32_t kOutSpan = kFullScale - kFloorCode;
        const uint16_t sdr = uint16_t(PA_SDR_MIN_CODE + (uint32_t(code - kFloorCode) * kInSpan + kOutSpan / 2) / kOutSpan);
        const uint32_t mn = pa_sdr_millinits(pa_sdr_fixed_for_code(sdr));
        return mn < kMinNits * 1000u ? kMinNits * 1000u : mn > kMaxNits * 1000u ? kMaxNits * 1000u : mn;
    }
    const uint32_t span = kFullScale - kFloorCode;
    return kMinNits * 1000u + uint32_t((uint64_t(code - kFloorCode) * ((kMaxNits - kMinNits) * 1000u) + span / 2) / span);
}

// Ascending perceptual brightness step ladder from Pad 8 Pro Ktz8866.h.
class Steps {
  public:
    void build(uint16_t floor, bool exponential, uint16_t ceiling = kFullScale)
    {
        n_ = 0;
        floor_ = floor;
        ceiling_ = ceiling;
        if (exponential) {
            for (unsigned i = 0; i <= 16; ++i)
                code_[n_++] = uint16_t(floor + (uint32_t(ceiling - floor) * i + 8) / 16);
            return;
        }
        static constexpr uint64_t kCubeRoots[3] = {1000000, 1259921, 1587401};   // 2^(0/3), 2^(1/3), 2^(2/3) x 10^6
        uint16_t down[kMax] = {};
        unsigned m = 0;
        for (unsigned k = 0; m + 1 < kMax; ++k) {
            const uint64_t divisor = kCubeRoots[k % 3] << (k / 3);
            const uint16_t v = uint16_t((uint64_t(ceiling) * 1000000 + divisor / 2) / divisor);
            if (v <= floor) break;
            down[m++] = v;
        }
        code_[n_++] = floor;
        while (m) code_[n_++] = down[--m];
    }
    uint16_t up(uint16_t code) const
    {
        for (unsigned i = 0; i < n_; ++i)
            if (code_[i] > code) return code_[i];
        return code > ceiling_ ? ceiling_ : code;
    }
    uint16_t down(uint16_t code) const
    {
        if (code <= floor_) return code;
        for (unsigned i = n_; i-- > 0;)
            if (code_[i] < code) return code_[i];
        return floor_;
    }
    unsigned count() const { return n_; }
    uint16_t at(unsigned i) const { return code_[i]; }

  private:
    static constexpr unsigned kMax = 40;
    uint16_t code_[kMax] = {};
    unsigned n_ = 0;
    uint16_t floor_ = 0;
    uint16_t ceiling_ = kFullScale;
};

}  // namespace shengbacklight

#endif
