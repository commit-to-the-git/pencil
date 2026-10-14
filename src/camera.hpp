// camera.hpp - perspective pinhole camera with DOF for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.2: depth of field. aperture > 0 produces defocus blur,
// rays are jittered on a disc of radius aperture around the eye,
// and refocused at the focal plane (distance focus_dist).
// v0.0.5: bokeh aperture shapes (hexagon, triangle, etc.)
// v0.0.6: set_aspect() — allow changing aspect after construction.
//   used by the distributed worker when the CLI --width/--height
//   disagree with the scene-file `size: w h`, which would otherwise
//   bake the wrong aspect into the camera at parse time and stretch
//   tiles horizontally. the worker calls `set_aspect(target_aspect)`
//   on the first tile and the camera recomputes its horizontal/vertical
//   vectors while preserving eye/look/up/vfov/aperture/focus.

#pragma once

#include "ray.hpp"
#include "vec3.hpp"

namespace pencil {

struct Rng;

class Camera {
public:
    Camera() = default;

    Camera(Point3 eye, Point3 look, Vec3 up_hint,
           float vfov_deg, float aspect,
           float aperture_ = 0.0f, float focus_dist_ = 1.0f)
        : origin_(eye), aperture_(aperture_), focus_dist_(focus_dist_),
          vfov_deg_(vfov_deg), aspect_(aspect) {
        recompute_basis(eye, look, up_hint);
        recompute_frustum();
    }

    // ray_at with optional DOF jitter. rng may be null for no jitter.
    Ray ray_at(float u, float v, Rng* rng = nullptr) const;

    Point3 origin() const { return origin_; }

    float aperture() const { return aperture_; }
    float focus_dist() const { return focus_dist_; }

    // v0.0.6: change aspect ratio on the fly. recomputes the horizontal
    // extent and lower-left corner while preserving vfov, aperture,
    // focus, eye, and basis vectors. used by the distributed worker
    // to fix the mesh_dist horizontal stretch bug (scene file `size:`
    // and CLI `--width/--height` disagreeing on aspect).
    void set_aspect(float new_aspect) {
        aspect_ = new_aspect;
        recompute_frustum();
    }
    float aspect() const { return aspect_; }
    float vfov_deg() const { return vfov_deg_; }

    // v0.0.5: bokeh aperture shape.
    // 0 = circle (default), 3 = triangle, 4 = square, 5 = pentagon,
    // 6 = hexagon, 8 = octagon.
    void set_bokeh_blades(int n) { bokeh_blades_ = n; }
    int bokeh_blades() const { return bokeh_blades_; }

private:
    // (re)derive u_, v_, w_ from eye/look/up. called by ctor and by
    // any future "set_look_at()" method. does NOT touch the frustum.
    void recompute_basis(Point3 eye, Point3 look, Vec3 up_hint) {
        w_ = (eye - look).normalized();
        u_ = up_hint.cross(w_).normalized();
        v_ = w_.cross(u_);
    }

    // (re)derive horizontal_, vertical_, lower_left_ from the current
    // vfov_deg_, aspect_, focus_dist_, origin_, and basis. called by
    // ctor and by set_aspect() so a single aspect change does not need
    // to redo the (more expensive, branchy) basis computation.
    void recompute_frustum() {
        float theta = vfov_deg_ * 3.14159265358979323846f / 180.0f;
        float half_h = std::tan(theta * 0.5f);
        float half_w = aspect_ * half_h;
        horizontal_ = u_ * (2.0f * half_w * focus_dist_);
        vertical_   = v_ * (2.0f * half_h * focus_dist_);
        lower_left_ = origin_ - horizontal_ * 0.5f
                            - vertical_   * 0.5f
                            - w_ * focus_dist_;
    }

    Point3 origin_;
    Vec3   lower_left_;
    Vec3   horizontal_;
    Vec3   vertical_;
    Vec3   u_, v_, w_;
    float  aperture_   = 0.0f;
    float  focus_dist_ = 1.0f;
    float  vfov_deg_   = 60.0f;   // v0.0.6: retained for set_aspect()
    float  aspect_     = 1.0f;    // v0.0.6: retained for set_aspect()
    int    bokeh_blades_ = 0;  // v0.0.5: 0=circle, N=polygon
};

} // namespace pencil
