// light.hpp - light sources for pencil
// SPDX-License-Identifier: Apache-2.0
//
// supports point and directional lights.
// v0.0.2: point lights carry an optional radius for soft shadows;
// when radius > 0, the renderer jitteres the position on a small disc
// perpendicular to the light-to-hit-point vector.

#pragma once

#include "vec3.hpp"

namespace pencil {

enum class LightKind : int {
    Point      = 0,
    Directional = 1,
};

struct Light {
    LightKind kind = LightKind::Point;
    Vec3  position_or_direction = {0.0f, 5.0f, 0.0f};
    Color intensity = {1.0f, 1.0f, 1.0f};
    float radius = 0.0f;  // soft shadow disc radius. 0 = hard.
    bool  is_emissive = false;  // if true, skip shadow test (light comes from a surface)
};

inline Light make_point_light(Vec3 pos, Color intensity, float radius = 0.0f,
                               bool is_emissive = false) {
    Light l;
    l.kind = LightKind::Point;
    l.position_or_direction = pos;
    l.intensity = intensity;
    l.radius = radius;
    l.is_emissive = is_emissive;
    return l;
}

inline Light make_directional_light(Vec3 dir, Color intensity) {
    Light l;
    l.kind = LightKind::Directional;
    l.position_or_direction = dir.normalized();
    l.intensity = intensity;
    l.radius = 0.0f;
    return l;
}

} // namespace pencil
