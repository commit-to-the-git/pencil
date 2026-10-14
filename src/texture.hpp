// texture.hpp - 2D texture sampling for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.4: supports procedural textures (checker, noise, marble, wood)
// in addition to image-based textures. set via set_procedural().

#pragma once

#include "image_io.hpp"
#include "vec3.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace pencil {

enum class ProcType {
    None,
    Checker,
    Noise,
    Marble,
    Wood,
    Ridged,   // v0.0.7: ridged multifractal noise (sharp peaks/valleys)
    Fbm,      // v0.0.7: fractional brownian motion (layered octaves)
};

class Texture {
public:
    Texture() = default;

    // load from file. returns false on failure.
    bool load(const std::string& path, std::string& err) {
        int w, h;
        std::vector<uint8_t> px;
        if (!read_image(path, w, h, px, err, 3)) return false;
        width_  = w;
        height_ = h;
        data_.resize((size_t)w * h * 3);
        for (size_t i = 0; i < (size_t)w * h * 3; ++i) {
            data_[i] = px[i] / 255.0f;
        }
        proc_type_ = ProcType::None;
        return true;
    }

    // v0.0.4: set as procedural texture.
    void set_procedural(ProcType type, float scale, Color c1, Color c2) {
        proc_type_ = type;
        proc_scale_ = scale;
        proc_color1_ = c1;
        proc_color2_ = c2;
        // mark as valid by setting width/height to 1 (non-zero)
        width_ = 1;
        height_ = 1;
    }

    // sample with bilinear filtering, repeat wrap.
    // uv in [0,1]^2.
    Color sample(float u, float v) const {
        if (proc_type_ != ProcType::None) {
            return sample_procedural(u, v);
        }
        if (width_ == 0 || height_ == 0) return {1.0f, 0.0f, 1.0f};

        u = u - std::floor(u);
        v = v - std::floor(v);

        float fx = u * width_  - 0.5f;
        float fy = v * height_ - 0.5f;
        int x0 = (int)std::floor(fx);
        int y0 = (int)std::floor(fy);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        float tx = fx - x0;
        float ty = fy - y0;

        x0 = (x0 + width_)  % width_;
        x1 = (x1 + width_)  % width_;
        y0 = (y0 + height_) % height_;
        y1 = (y1 + height_) % height_;

        Color c00 = at(x0, y0);
        Color c10 = at(x1, y0);
        Color c01 = at(x0, y1);
        Color c11 = at(x1, y1);

        Color a = c00 * (1.0f - tx) + c10 * tx;
        Color b = c01 * (1.0f - tx) + c11 * tx;
        return a * (1.0f - ty) + b * ty;
    }

    Color sample_dir(const Vec3& d) const {
        if (width_ == 0) return {0,0,0};
        Vec3 dir = d.normalized();
        float u = std::atan2(dir.z, dir.x) / (2.0f * 3.14159265358979f) + 0.5f;
        float v = std::asin(clampf(dir.y, -1.0f, 1.0f)) / 3.14159265358979f + 0.5f;
        v = 1.0f - v;
        return sample(u, v);
    }

    bool valid() const { return width_ > 0; }
    int width() const { return width_; }
    int height() const { return height_; }

private:
    int width_ = 0;
    int height_ = 0;
    std::vector<float> data_;

    // procedural texture params
    ProcType proc_type_ = ProcType::None;
    float proc_scale_ = 1.0f;
    Color proc_color1_ = {1, 1, 1};
    Color proc_color2_ = {0, 0, 0};

    inline Color at(int x, int y) const {
        size_t i = ((size_t)y * width_ + x) * 3;
        return {data_[i], data_[i+1], data_[i+2]};
    }

    // hash-based value noise
    static float hash_noise(float x, float y) {
        float n = std::sin(x * 12.9898f + y * 78.233f) * 43758.5453f;
        return n - std::floor(n);
    }

    // smooth interpolated noise
    static float smooth_noise(float x, float y) {
        int ix = (int)std::floor(x);
        int iy = (int)std::floor(y);
        float fx = x - ix;
        float fy = y - iy;
        // smoothstep
        float sx = fx * fx * (3.0f - 2.0f * fx);
        float sy = fy * fy * (3.0f - 2.0f * fy);
        float n00 = hash_noise(ix,     iy);
        float n10 = hash_noise(ix + 1, iy);
        float n01 = hash_noise(ix,     iy + 1);
        float n11 = hash_noise(ix + 1, iy + 1);
        float a = n00 * (1.0f - sx) + n10 * sx;
        float b = n01 * (1.0f - sx) + n11 * sx;
        return a * (1.0f - sy) + b * sy;
    }

    Color sample_procedural(float u, float v) const {
        float s = proc_scale_;
        switch (proc_type_) {
            case ProcType::Checker: {
                float cu = std::floor(u * s);
                float cv = std::floor(v * s);
                return ((int)(cu + cv) % 2 == 0) ? proc_color1_ : proc_color2_;
            }
            case ProcType::Noise: {
                float n = smooth_noise(u * s * 10.0f, v * s * 10.0f);
                return proc_color1_ * n + proc_color2_ * (1.0f - n);
            }
            case ProcType::Marble: {
                float n = smooth_noise(u * s * 10.0f, v * s * 10.0f);
                float marble = std::sin((u * s + n * 2.0f) * 3.14159f) * 0.5f + 0.5f;
                return proc_color1_ * marble + proc_color2_ * (1.0f - marble);
            }
            case ProcType::Wood: {
                float n = smooth_noise(u * s * 5.0f, v * s * 5.0f);
                float rings = std::sin((u * s + n * 3.0f) * 20.0f) * 0.5f + 0.5f;
                return proc_color1_ * rings + proc_color2_ * (1.0f - rings);
            }
            // v0.0.7: ridged multifractal noise. creates sharp ridges
            // and valleys by taking 1 - abs(noise) and raising to a
            // power. used for displacement to get crystalline/rocky
            // surfaces without increasing triangle count.
            case ProcType::Ridged: {
                // sum 4 octaves of ridged noise
                float result = 0.0f;
                float amp = 0.5f;
                float freq = s * 10.0f;
                float norm = 0.0f;
                for (int oct = 0; oct < 4; ++oct) {
                    float n = smooth_noise(u * freq, v * freq);
                    // ridge: 1 - 2*|n - 0.5|, peaks at n=0.5
                    float ridge = 1.0f - 2.0f * std::fabs(n - 0.5f);
                    ridge = std::pow(ridge, 2.0f); // sharpen
                    result += ridge * amp;
                    norm += amp;
                    amp *= 0.5f;
                    freq *= 2.0f;
                }
                result /= (norm > 0.0f ? norm : 1.0f);
                return proc_color1_ * result + proc_color2_ * (1.0f - result);
            }
            // v0.0.7: fractional brownian motion. layers multiple
            // octaves of noise at increasing frequency and decreasing
            // amplitude. produces natural-looking turbulent detail.
            case ProcType::Fbm: {
                float result = 0.0f;
                float amp = 0.5f;
                float freq = s * 10.0f;
                float norm = 0.0f;
                for (int oct = 0; oct < 5; ++oct) {
                    float n = smooth_noise(u * freq, v * freq);
                    result += n * amp;
                    norm += amp;
                    amp *= 0.5f;
                    freq *= 2.0f;
                }
                result /= (norm > 0.0f ? norm : 1.0f);
                return proc_color1_ * result + proc_color2_ * (1.0f - result);
            }
            default:
                return proc_color1_;
        }
    }
};

using TexturePtr = std::shared_ptr<Texture>;

} // namespace pencil
