// rng.hpp - tiny deterministic rng for pencil
// SPDX-License-Identifier: Apache-2.0
//
// xorshift32. fast, reproducible. used for AA jitter, soft shadows,
// dOF aperture jitter, and dispersion ray selection.

#pragma once

#include <cstdint>

namespace pencil {

struct Rng {
    uint32_t state = 0x12345678u;
    explicit Rng(uint32_t s) : state(s) {}

    inline uint32_t u32() {
        state ^= state << 13u;
        state ^= state >> 17u;
        state ^= state << 5u;
        return state;
    }
    inline float unit() { return u32() / 4294967296.0f; }       // [0, 1)
    inline float sym()  { return unit() * 2.0f - 1.0f; }        // [-1, 1)

    // sample uniformly on a unit disc (Shirley's concentric mapping).
    // returns direction (dx, dy) on unit circle.
    inline void unit_disc(float& dx, float& dy) {
        float r1 = unit();
        float r2 = unit();
        float sx = 2.0f * r1 - 1.0f;
        float sy = 2.0f * r2 - 1.0f;
        if (sx == 0.0f && sy == 0.0f) { dx = 0; dy = 0; return; }
        float theta, r;
        if (std::fabs(sx) > std::fabs(sy)) {
            r = sx;
            theta = (3.14159265f / 4.0f) * (sy / sx);
        } else {
            r = sy;
            theta = (3.14159265f / 2.0f) - (3.14159265f / 4.0f) * (sx / sy);
        }
        dx = r * std::cos(theta);
        dy = r * std::sin(theta);
    }
};

} // namespace pencil
