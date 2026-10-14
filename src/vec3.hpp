// copyright 2026 pencil authors
// SPDX-License-Identifier: Apache-2.0
// licensed under the apache license, version 2.0 (the "license");
// you may not use this file except in compliance with the license.
// you may obtain a copy of the license at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// unless required by applicable law or agreed to in writing, software
// distributed under the license is distributed on an "as is" basis,
// without warranties or conditions of any kind, either express or implied.
// see the license for the specific language governing permissions and
// limitations under the license.

// vec3.hpp - 3D vector math for Pencil Part of the Pencil ray tracer
#pragma once

#include <cmath>
#include <iostream>

namespace pencil {

// a 3-component vector
struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    constexpr Vec3() = default;
    constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    // component access by index, useful for generic loops
    float operator[](int i) const {
        return (i == 0) ? x : (i == 1) ? y : z;
    }
    float& operator[](int i) {
        return (i == 0) ? x : (i == 1) ? y : z;
    }

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }

    // scalar multiplication / division are the most common operations in a ray trac...
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

    // hadamard (component-wise) product, used heavily for color mixing
    Vec3 operator*(const Vec3& o) const { return {x * o.x, y * o.y, z * o.z}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(float s)       { x *= s;  y *= s;  z *= s;  return *this; }

    float dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    Vec3 cross(const Vec3& o) const {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }
    float length_squared() const { return x * x + y * y + z * z; }
    float length() const { return std::sqrt(length_squared()); }

    Vec3 normalized() const {
        float len = length();
        // guard against divide-by-zero for degenerate directions
        return (len > 1e-12f) ? (*this / len) : Vec3{0.0f, 0.0f, 0.0f};
    }

    // reflect about a unit normal N
    Vec3 reflect(const Vec3& n) const {
        return *this - n * (2.0f * this->dot(n));
    }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

// convenience aliases for readability
using Point3 = Vec3;
using Color  = Vec3;  // rGB, linear, components in [0, 1].

// schlick fresnel for dielectrics
inline float schlick(float cos, float ref_idx_ratio) {
    float r0 = (1.0f - ref_idx_ratio) / (1.0f + ref_idx_ratio);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * std::pow(1.0f - cos, 5.0f);
}

// clamp helper used throughout the renderer
inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

} // namespace pencil
