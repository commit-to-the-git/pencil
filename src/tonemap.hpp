// tonemap.hpp - tone mapping for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.5: ACES filmic tone mapping + exposure control.
// aCES smoothly rolls off highlights instead of clipping to white.
// this makes HDR scenes (with bright lights, emissive surfaces, etc.)
// look cinematic rather than blown out.

#pragma once

#include "vec3.hpp"

#include <algorithm>
#include <cmath>

namespace pencil {

enum class ToneMap {
    None,       // no tone mapping, just gamma
    ACES,       // aCES filmic (default for v0.0.5)
    Reinhard,   // Reinhard
    Exponential,
};

// aCES filmic tone mapping.
// based on Narkowicz 2015   fast, good-looking, widely used.
inline float aces(float x) {
    float a = 2.51f;
    float b = 0.03f;
    float c = 2.43f;
    float d = 0.59f;
    float e = 0.14f;
    return std::max(0.0f, (x * (a * x + b)) / (x * (c * x + d) + e));
}

// reinhard tone mapping.
inline float reinhard(float x) {
    return x / (1.0f + x);
}

// apply tone mapping + exposure to a color.
// exposure is in stops (1.0 = +1 stop = 2x brightness).
inline Color tonemap_color(Color c, ToneMap mode, float exposure) {
    // apply exposure
    float e = std::pow(2.0f, exposure);
    c = c * e;

    // apply tone mapping per channel
    switch (mode) {
        case ToneMap::ACES:
            return { aces(c.x), aces(c.y), aces(c.z) };
        case ToneMap::Reinhard:
            return { reinhard(c.x), reinhard(c.y), reinhard(c.z) };
        case ToneMap::Exponential:
            return { 1.0f - std::exp(-c.x),
                     1.0f - std::exp(-c.y),
                     1.0f - std::exp(-c.z) };
        default:
            return c;
    }
}

} // namespace pencil
