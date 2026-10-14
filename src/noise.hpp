// noise.hpp - perlin noise for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.6: perlin noise for terrain generation and procedural textures.
// same algorithm family as minecraft uses.

#pragma once

#include <cmath>
#include <cstdint>

namespace pencil {

class PerlinNoise {
public:
    PerlinNoise(uint32_t seed = 0) {
        // generate permutation table from seed
        uint32_t s = seed;
        if (s == 0) s = 1;
        for (int i = 0; i < 256; ++i) perm_[i] = (uint8_t)i;
        // shuffle using LCG seeded by user seed
        for (int i = 255; i > 0; --i) {
            s = s * 1103515245u + 12345u;
            int j = (s / 65536u) % (i + 1);
            uint8_t tmp = perm_[i]; perm_[i] = perm_[j]; perm_[j] = tmp;
        }
        for (int i = 0; i < 256; ++i) perm_[i + 256] = perm_[i];
    }

    // 2D perlin noise in [-1, 1].
    float noise2D(float x, float y) const {
        int xi = (int)std::floor(x) & 255;
        int yi = (int)std::floor(y) & 255;
        float xf = x - std::floor(x);
        float yf = y - std::floor(y);
        float u = fade(xf);
        float v = fade(yf);

        int aa = perm_[perm_[xi] + yi];
        int ab = perm_[perm_[xi] + yi + 1];
        int ba = perm_[perm_[xi + 1] + yi];
        int bb = perm_[perm_[xi + 1] + yi + 1];

        float x1 = lerp(grad(aa, xf, yf), grad(ba, xf - 1, yf), u);
        float x2 = lerp(grad(ab, xf, yf - 1), grad(bb, xf - 1, yf - 1), u);
        return lerp(x1, x2, v);
    }

    // fractal brownian motion (multiple octaves of perlin).
    float fbm(float x, float y, int octaves = 4, float persistence = 0.5f,
              float lacunarity = 2.0f) const {
        float total = 0.0f;
        float amplitude = 1.0f;
        float frequency = 1.0f;
        float max_value = 0.0f;
        for (int i = 0; i < octaves; ++i) {
            total += noise2D(x * frequency, y * frequency) * amplitude;
            max_value += amplitude;
            amplitude *= persistence;
            frequency *= lacunarity;
        }
        return total / max_value;
    }

    // hash a string seed into a uint32
    static uint32_t hash_seed(const std::string& s) {
        uint32_t h = 2166136261u;
        for (char c : s) {
            h ^= (uint8_t)c;
            h *= 16777619u;
        }
        return h ? h : 1;
    }

private:
    uint8_t perm_[512];

    static float fade(float t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    static float lerp(float a, float b, float t) {
        return a + t * (b - a);
    }

    static float grad(int hash, float x, float y) {
        int h = hash & 7;
        float u = h < 4 ? x : y;
        float v = h < 4 ? y : x;
        return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
    }
};

} // namespace pencil
