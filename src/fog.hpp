// fog.hpp - volumetric fog for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.4: homogeneous volumetric fog.
// v0.0.5: fog gradient (height-based color/density).
// fog can have different colors at top and bottom of its volume,
// blended by height. useful for realistic skies, sunsets, ground mist.
//
// rays traveling through fog get attenuated by exp(-density * dist),
// and fog contributes its own color ().
// supports height-based fog (only below a certain Y).

#pragma once

#include "vec3.hpp"

#include <cmath>

namespace pencil {

struct Fog {
 float density = 0.0f;
 Color color = {0.5f, 0.5f, 0.6f};
 float height = 1e9f;
 bool enabled = false;

 // v0.0.5: gradient fog
 bool has_gradient = false; // if true, blend between color (bottom) and color_top
 Color color_top = {0.3f, 0.4f, 0.7f}; // fog color at top of volume
 float gradient_min = -5.0f; // y where bottom color is full
 float gradient_max = 5.0f; // y where top color is full
};

// get fog color at a given world-space Y height.
// for gradient fog, blends between bottom and top colors.
inline Color fog_color_at(const Fog& fog, float y) {
 if (!fog.has_gradient) return fog.color;
 const float span = fog.gradient_max - fog.gradient_min;
 if (std::fabs(span) < 1e-8f) return fog.color_top;
 float t = (y - fog.gradient_min) / span;
 t = clampf(t, 0.0f, 1.0f);
 return fog.color * (1.0f - t) + fog.color_top * t;
}

// compute fog transmittance along a ray segment of length `dist`.
inline float fog_transmittance(const Fog& fog, float dist) {
 if (!fog.enabled || fog.density <= 0.0f || dist <= 0.0f) return 1.0f;
 return std::exp(-fog.density * dist);
}

// blend a color with fog over a distance.
// v0.0.5: takes the hit point Y to support gradient fog color.
inline Color apply_fog(const Fog& fog, const Color& c, float dist, float hit_y) {
 if (!fog.enabled || fog.density <= 0.0f || dist <= 0.0f) return c;
 float t = fog_transmittance(fog, dist);
 Color fc = fog_color_at(fog, hit_y);
 return c * t + fc * (1.0f - t);
}

} // namespace pencil
