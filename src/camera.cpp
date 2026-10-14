// camera.cpp - camera with dof and bokeh
// SPDX-License-Identifier: Apache-2.0

#include "camera.hpp"
#include "rng.hpp"

#include <cmath>

namespace pencil {

// sample aperture shape. blades=0 circle, N=polygon.
// uses rejection sampling: sample on disc, reject if outside polygon.
// this produces correct bokeh shapes at any resolution.
static void sample_aperture(Rng& rng, int blades, float& dx, float& dy) {
    if (blades <= 2) {
        rng.unit_disc(dx, dy);
        return;
    }
    // rejection sampling: sample on disc, keep only points inside the polygon.
    // this is slower than barycentric but produces correct-shaped bokeh
    // at any resolution (the old barycentric method created uniform
    // density within each triangle sector, which made the shape
    // indistinguishable from a circle at low resolution).
    for (int attempt = 0; attempt < 16; ++attempt) {
        rng.unit_disc(dx, dy);
        // check if point is inside the regular polygon.
        // a regular polygon with N blades has vertices at angles
        // 2*pi*k/N for k=0..N-1. the point is inside if it's on the
        // correct side of all edge lines.
        float angle_step = 2.0f * 3.14159265358979f / blades;
        // rotate so the first edge is at angle 0
        float r = std::sqrt(dx * dx + dy * dy);
        float theta = std::atan2(dy, dx);
        if (theta < 0) theta += 2.0f * 3.14159265358979f;
        // find which sector the point is in
        int sector = (int)(theta / angle_step);
        // local angle within the sector
        float local_angle = theta - sector * angle_step;
        // max radius at this angle (distance to the edge midpoint)
        float half_step = angle_step * 0.5f;
        float max_r = std::cos(half_step) / std::cos(local_angle - half_step);
        if (r <= max_r) return; // inside polygon
    }
    // fallback: just use the disc sample
    rng.unit_disc(dx, dy);
}

Ray Camera::ray_at(float u, float v, Rng* rng) const {
    Vec3 dir = lower_left_ + horizontal_ * u + vertical_ * v - origin_;
    Point3 origin = origin_;

    if (rng && aperture_ > 0.0f) {
        float fd = focus_dist_;
        const Vec3 forward = (lower_left_ + horizontal_ * 0.5f + vertical_ * 0.5f - origin_).normalized();
        const float denom = dir.normalized().dot(forward);
        const float focus_t = (std::fabs(denom) > 1e-6f) ? fd / denom : fd;
        Vec3 focal_point = origin_ + dir.normalized() * focus_t;
        float dx, dy;
        int blades = bokeh_blades_;
        sample_aperture(*rng, blades, dx, dy);
        Vec3 offset = u_ * dx + v_ * dy;
        offset = offset * aperture_;
        origin = origin_ + offset;
        dir = focal_point - origin;
    }

    return Ray{origin, dir.normalized()};
}

} // namespace pencil
