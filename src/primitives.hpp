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

// primitives.hpp - Geometric primitives + hit records for Pencil
#pragma once

#include "expr.hpp"
#include "material.hpp"
#include "ray.hpp"
#include "vec3.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace pencil {

// axis-aligned bounding box
struct AABB {
    Vec3 min = { 1e30f, 1e30f, 1e30f};
    Vec3 max = {-1e30f,-1e30f,-1e30f};

    // slab test: returns true if ray r hits the box in (t_min, t_max).
    // branchy version with early-exit   faster for BVH traversal where
    // most rays miss and exit on the first or second axis.
    bool hit(const Ray& r, float t_min, float t_max) const {
        for (int a = 0; a < 3; ++a) {
            float inv_d = 1.0f / r.direction[a];
            float t0 = (min[a] - r.origin[a]) * inv_d;
            float t1 = (max[a] - r.origin[a]) * inv_d;
            if (inv_d < 0.0f) std::swap(t0, t1);
            t_min = (t0 > t_min) ? t0 : t_min;
            t_max = (t1 < t_max) ? t1 : t_max;
            if (t_max < t_min) return false;
        }
        return true;
    }

    static AABB merge(const AABB& a, const AABB& b) {
        AABB out;
        out.min = {std::fmin(a.min.x, b.min.x),
                   std::fmin(a.min.y, b.min.y),
                   std::fmin(a.min.z, b.min.z)};
        out.max = {std::fmax(a.max.x, b.max.x),
                   std::fmax(a.max.y, b.max.y),
                   std::fmax(a.max.z, b.max.z)};
        return out;
    }

    int longest_axis() const {
        Vec3 ext = max - min;
        if (ext.x >= ext.y && ext.x >= ext.z) return 0;
        if (ext.y >= ext.z) return 1;
        return 2;
    }
};

struct HitRecord {
    Point3 p      = {0.0f, 0.0f, 0.0f};
    Vec3   normal = {0.0f, 1.0f, 0.0f};
    float  t      = -1.0f;
    int    material_id = 0;
    bool   front_face   = true;

    void set_face_normal(const Ray& r, const Vec3& outward_n) {
        front_face = r.direction.dot(outward_n) < 0.0f;
        normal = front_face ? outward_n : -outward_n;
    }
};

class Primitive {
public:
    virtual ~Primitive() = default;
    virtual bool hit(const Ray& r, float t_min, float t_max,
                     HitRecord& out_rec) const = 0;
    virtual AABB aabb() const = 0;
};

// sphere: |P - C| = r
class Sphere : public Primitive {
public:
    Sphere() = default;
    Sphere(Point3 center, float radius, int material_id)
        : center_(center), radius_(radius), material_id_(material_id) {}

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        Vec3 oc = r.origin - center_;
        float a = r.direction.dot(r.direction);
        float half_b = oc.dot(r.direction);
        float c = oc.dot(oc) - radius_ * radius_;
        float disc = half_b * half_b - a * c;
        if (disc < 0.0f) return false;

        float sq = std::sqrt(disc);
        float t = (-half_b - sq) / a;
        if (t < t_min || t > t_max) {
            t = (-half_b + sq) / a;
            if (t < t_min || t > t_max) return false;
        }

        out_rec.t = t;
        out_rec.p = r.at(t);
        Vec3 outward_n = (out_rec.p - center_) / radius_;
        out_rec.set_face_normal(r, outward_n);
        out_rec.material_id = material_id_;
        return true;
    }

    AABB aabb() const override {
        return { center_ - Vec3{radius_, radius_, radius_},
                 center_ + Vec3{radius_, radius_, radius_} };
    }

    Point3 center() const { return center_; }
    float  radius() const { return radius_; }

private:
    Point3 center_ = {0.0f, 0.0f, 0.0f};
    float  radius_ = 1.0f;
    int    material_id_ = 0;
};

// triangle: three CCW vertices
class Triangle : public Primitive {
public:
    Triangle() = default;
    Triangle(Point3 v0, Point3 v1, Point3 v2, int material_id)
        : v0_(v0), v1_(v1), v2_(v2), material_id_(material_id) {
        normal_ = (v1_ - v0_).cross(v2_ - v0_).normalized();
    }

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        Vec3 edge1 = v1_ - v0_;
        Vec3 edge2 = v2_ - v0_;
        Vec3 h = r.direction.cross(edge2);
        float a = edge1.dot(h);
        if (std::fabs(a) < 1e-8f) return false;

        float f = 1.0f / a;
        Vec3 s = r.origin - v0_;
        float u = f * s.dot(h);
        if (u < 0.0f || u > 1.0f) return false;

        Vec3 q = s.cross(edge1);
        float v = f * r.direction.dot(q);
        if (v < 0.0f || u + v > 1.0f) return false;

        float t = f * edge2.dot(q);
        if (t < t_min || t > t_max) return false;

        out_rec.t = t;
        out_rec.p = r.at(t);
        out_rec.set_face_normal(r, normal_);
        out_rec.material_id = material_id_;
        return true;
    }

    AABB aabb() const override {
        AABB box;
        box.min = {std::fmin(std::fmin(v0_.x, v1_.x), v2_.x),
                   std::fmin(std::fmin(v0_.y, v1_.y), v2_.y),
                   std::fmin(std::fmin(v0_.z, v1_.z), v2_.z)};
        box.max = {std::fmax(std::fmax(v0_.x, v1_.x), v2_.x),
                   std::fmax(std::fmax(v0_.y, v1_.y), v2_.y),
                   std::fmax(std::fmax(v0_.z, v1_.z), v2_.z)};
        return box;
    }

    Point3 v0() const { return v0_; }
    Point3 v1() const { return v1_; }
    Point3 v2() const { return v2_; }
    Vec3   normal() const { return normal_; }

private:
    Point3 v0_, v1_, v2_;
    Vec3   normal_ = {0.0f, 1.0f, 0.0f};
    int    material_id_ = 0;
};

// cube: axis-aligned box from center + half_extent
class Cube : public Primitive {
public:
    Cube() = default;
    Cube(Point3 center, float half_extent, int material_id)
        : center_(center), half_(half_extent), material_id_(material_id) {}

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        Vec3 lo = center_ - Vec3{half_, half_, half_};
        Vec3 hi = center_ + Vec3{half_, half_, half_};

        float t_near = -INFINITY, t_far = INFINITY;
        int near_axis = -1, far_axis = -1;
        float near_sign = 1.0f, far_sign = -1.0f;

        for (int a = 0; a < 3; ++a) {
            const float d = r.direction[a];
            if (std::fabs(d) < 1e-12f) {
                if (r.origin[a] < lo[a] || r.origin[a] > hi[a]) return false;
                continue;
            }
            float tn = (lo[a] - r.origin[a]) / d;
            float tf = (hi[a] - r.origin[a]) / d;
            float ns = -1.0f, fs = 1.0f;
            if (tn > tf) { std::swap(tn, tf); std::swap(ns, fs); }
            if (tn > t_near) { t_near = tn; near_axis = a; near_sign = ns; }
            if (tf < t_far) { t_far = tf; far_axis = a; far_sign = fs; }
            if (t_far < t_near) return false;
        }

        const bool use_near = t_near >= t_min;
        const float t_hit = use_near ? t_near : t_far;
        const int hit_axis = use_near ? near_axis : far_axis;
        const float hit_sign = use_near ? near_sign : far_sign;
        if (hit_axis < 0 || t_hit < t_min || t_hit > t_max) return false;

        out_rec.t = t_hit;
        out_rec.p = r.at(t_hit);
        Vec3 nrm{0.0f, 0.0f, 0.0f};
        nrm[hit_axis] = hit_sign;
        out_rec.set_face_normal(r, nrm);
        out_rec.material_id = material_id_;
        return true;
    }

    AABB aabb() const override {
        return { center_ - Vec3{half_, half_, half_},
                 center_ + Vec3{half_, half_, half_} };
    }

    Point3 center() const { return center_; }
    float  half() const { return half_; }

private:
    Point3 center_ = {0.0f, 0.0f, 0.0f};
    float  half_    = 1.0f;
    int    material_id_ = 0;
};

// plane: infinite plane through `point_` with normal `normal_`
class Plane : public Primitive {
public:
    Plane() = default;
    Plane(Point3 point, Vec3 normal, int material_id)
        : point_(point), normal_(normal.normalized()), material_id_(material_id) {}

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        float denom = normal_.dot(r.direction);
        if (std::fabs(denom) < 1e-6f) return false;
        float t = (point_ - r.origin).dot(normal_) / denom;
        if (t < t_min || t > t_max) return false;
        out_rec.t = t;
        out_rec.p = r.at(t);
        out_rec.set_face_normal(r, normal_);
        out_rec.material_id = material_id_;
        return true;
    }

    // planes are infinite; return a huge AABB so the BVH always tests them
    AABB aabb() const override {
        const float BIG = 1e6f;
        return { {-BIG, -BIG, -BIG}, {BIG, BIG, BIG} };
    }

private:
    Point3 point_  = {0.0f, 0.0f, 0.0f};
    Vec3   normal_ = {0.0f, 1.0f, 0.0f};
    int    material_id_ = 0;
};

// implicitSurface: ray-marches f(x,y,z)=0 inside a bounding box
class ImplicitSurface : public Primitive {
public:
    ImplicitSurface(std::shared_ptr<Expr> expr,
                    Vec3 bbox_min, Vec3 bbox_max,
                    int material_id,
                    int max_steps = 200,
                    float step_size = 0.0f)
        : expr_(std::move(expr)),
          bbox_min_(bbox_min), bbox_max_(bbox_max),
          material_id_(material_id),
          max_steps_(max_steps),
          step_size_(step_size > 0.0f ? step_size
                                      : default_step(bbox_min, bbox_max)) {}

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        // find ray/AABB intersection range so we don't waste steps outside
        float t0 = t_min, t1 = t_max;
        if (!slab_intersect(r, t0, t1)) return false;
        if (t0 < t_min) t0 = t_min;
        if (t1 > t_max) t1 = t_max;

        float step = step_size_;
        float t = t0;
        float f_prev = eval_at(r, t);

        for (int i = 0; i < max_steps_ && t < t1; ++i) {
            t += step;
            float f_now = eval_at(r, t);
            if (f_prev * f_now <= 0.0f && f_prev != 0.0f) {
                // bisect to refine the root
                float a = t - step, b = t;
                for (int bsc = 0; bsc < 12; ++bsc) {
                    float m = 0.5f * (a + b);
                    float fm = eval_at(r, m);
                    if (f_prev * fm <= 0.0f) { b = m; }
                    else { a = m; f_prev = fm; }
                }
                float t_hit = 0.5f * (a + b);
                if (t_hit < t_min || t_hit > t_max) return false;
                out_rec.t = t_hit;
                out_rec.p = r.at(t_hit);
                out_rec.normal = estimate_normal(out_rec.p);
                out_rec.set_face_normal(r, out_rec.normal);
                out_rec.material_id = material_id_;
                return true;
            }
            f_prev = f_now;
        }
        return false;
    }

    AABB aabb() const override {
        return { bbox_min_, bbox_max_ };
    }

private:
    std::shared_ptr<Expr> expr_;
    Vec3 bbox_min_, bbox_max_;
    int   material_id_ = 0;
    int   max_steps_   = 200;
    float step_size_   = 0.02f;

    static float default_step(Vec3 lo, Vec3 hi) {
        Vec3 ext = hi - lo;
        float diag = ext.length();
        return diag / 200.0f;
    }

    float eval_at(const Ray& r, float t) const {
        Point3 p = r.at(t);
        return expr_->eval(p.x, p.y, p.z);
    }

    Vec3 estimate_normal(Point3 p) const {
        const float e = step_size_ * 0.5f;
        float dx = expr_->eval(p.x + e, p.y, p.z) - expr_->eval(p.x - e, p.y, p.z);
        float dy = expr_->eval(p.x, p.y + e, p.z) - expr_->eval(p.x, p.y - e, p.z);
        float dz = expr_->eval(p.x, p.y, p.z + e) - expr_->eval(p.x, p.y, p.z - e);
        return Vec3{dx, dy, dz}.normalized();
    }

    bool slab_intersect(const Ray& r, float& t0, float& t1) const {
        float lo = t0, hi = t1;
        for (int a = 0; a < 3; ++a) {
            const float d = r.direction[a];
            if (std::fabs(d) < 1e-12f) {
                if (r.origin[a] < bbox_min_[a] || r.origin[a] > bbox_max_[a]) return false;
                continue;
            }
            float tn = (bbox_min_[a] - r.origin[a]) / d;
            float tf = (bbox_max_[a] - r.origin[a]) / d;
            if (tn > tf) std::swap(tn, tf);
            if (tn > lo) lo = tn;
            if (tf < hi) hi = tf;
            if (hi < lo) return false;
        }
        t0 = lo; t1 = hi;
        return true;
    }
};

// helper: build two triangles from 4 vertices
inline std::vector<std::shared_ptr<Primitive>>
make_quad(Point3 a, Point3 b, Point3 c, Point3 d, int material_id) {
    std::vector<std::shared_ptr<Primitive>> tris;
    tris.push_back(std::make_shared<Triangle>(a, b, c, material_id));
    tris.push_back(std::make_shared<Triangle>(a, c, d, material_id));
    return tris;
}

// cylinder: finite along an axis, with two end caps
class Cylinder : public Primitive {
public:
    Cylinder() = default;
    Cylinder(Point3 center, Vec3 axis, float radius, float height, int material_id)
        : center_(center), axis_(axis.normalized()),
          radius_(radius), half_h_(height * 0.5f), material_id_(material_id) {}

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        bool found = false;
        float closest = t_max;
        HitRecord best;

        if (hit_side(r, t_min, closest, best)) {
            closest = best.t; out_rec = best; found = true;
        }
        if (hit_cap(r, t_min, closest, best, +1.0f)) {
            closest = best.t; out_rec = best; found = true;
        }
        if (hit_cap(r, t_min, closest, best, -1.0f)) {
            closest = best.t; out_rec = best; found = true;
        }
        return found;
    }

    AABB aabb() const override {
        Vec3 r{radius_, radius_, radius_};
        Vec3 h = axis_ * half_h_;
        // conservative bbox: full extent in each axis
        Vec3 ext = {std::fabs(h.x) + radius_,
                    std::fabs(h.y) + radius_,
                    std::fabs(h.z) + radius_};
        return { center_ - ext, center_ + ext };
    }

private:
    Point3 center_ = {0,0,0};
    Vec3   axis_   = {0,1,0};
    float  radius_ = 1.0f;
    float  half_h_ = 1.0f;
    int    material_id_ = 0;

    bool hit_side(const Ray& r, float t_min, float t_max, HitRecord& out) const {
        // ray-plane distance along axis
        Vec3 oc = r.origin - center_;
        float d_dot_a = r.direction.dot(axis_);
        float oc_dot_a = oc.dot(axis_);

        // quadratic for radial distance: |oc + t*d - (oc.a)*a|^2 = r^2 expand: |oc_perp...
        Vec3 d_perp = r.direction - axis_ * d_dot_a;
        Vec3 oc_perp = oc - axis_ * oc_dot_a;
        float a = d_perp.dot(d_perp);
        float b = 2.0f * oc_perp.dot(d_perp);
        float c = oc_perp.dot(oc_perp) - radius_ * radius_;
        if (a < 1e-9f) {
            if (std::fabs(b) < 1e-9f) return false;
            const float t = -c / b;
            if (t < t_min || t > t_max) return false;
            const float h = oc_dot_a + t * d_dot_a;
            if (std::fabs(h) > half_h_) return false;
            out.t = t;
            out.p = r.at(t);
            Vec3 axis_pt = center_ + axis_ * h;
            Vec3 outward_n = (out.p - axis_pt) / radius_;
            out.set_face_normal(r, outward_n);
            out.material_id = material_id_;
            return true;
        }
        float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f) return false;
        float sq = std::sqrt(disc);
        float t0 = (-b - sq) / (2.0f * a);
        float t1 = (-b + sq) / (2.0f * a);
        for (float t : {t0, t1}) {
            if (t < t_min || t > t_max) continue;
            float h = (oc_dot_a + t * d_dot_a);
            if (std::fabs(h) > half_h_) continue;
            out.t = t;
            out.p = r.at(t);
            // outward normal: point on circle at the same height
            Vec3 axis_pt = center_ + axis_ * h;
            Vec3 outward_n = (out.p - axis_pt) / radius_;
            out.set_face_normal(r, outward_n);
            out.material_id = material_id_;
            return true;
        }
        return false;
    }

    bool hit_cap(const Ray& r, float t_min, float t_max, HitRecord& out, float sign) const {
        // cap plane at center + axis * sign * half_h
        Point3 cap_center = center_ + axis_ * (sign * half_h_);
        Vec3 nrm = axis_ * sign;
        float denom = nrm.dot(r.direction);
        if (std::fabs(denom) < 1e-6f) return false;
        float t = (cap_center - r.origin).dot(nrm) / denom;
        if (t < t_min || t > t_max) return false;
        Point3 p = r.at(t);
        Vec3 d = p - cap_center;
        if (d.dot(d) > radius_ * radius_) return false;
        out.t = t;
        out.p = p;
        out.set_face_normal(r, nrm);
        out.material_id = material_id_;
        return true;
    }
};

// cone: finite along an axis, apex at top, base circle at bottom
class Cone : public Primitive {
public:
    Cone() = default;
    Cone(Point3 apex, Vec3 axis, float radius, float height, int material_id)
        : apex_(apex), axis_(axis.normalized()),
          radius_(radius), height_(height), material_id_(material_id) {
        base_center_ = apex_ - axis_ * height_;
    }

    bool hit(const Ray& r, float t_min, float t_max,
             HitRecord& out_rec) const override {
        bool found = false;
        float closest = t_max;
        HitRecord best;

        if (hit_side(r, t_min, closest, best)) {
            closest = best.t; out_rec = best; found = true;
        }
        if (hit_base(r, t_min, closest, best)) {
            closest = best.t; out_rec = best; found = true;
        }
        return found;
    }

    AABB aabb() const override {
        Vec3 r{radius_, radius_, radius_};
        Vec3 h = axis_ * height_;
        Vec3 ext = {std::fabs(h.x) + radius_,
                    std::fabs(h.y) + radius_,
                    std::fabs(h.z) + radius_};
        Point3 lo = {std::fmin(apex_.x, base_center_.x) - radius_,
                     std::fmin(apex_.y, base_center_.y) - radius_,
                     std::fmin(apex_.z, base_center_.z) - radius_};
        Point3 hi = {std::fmax(apex_.x, base_center_.x) + radius_,
                     std::fmax(apex_.y, base_center_.y) + radius_,
                     std::fmax(apex_.z, base_center_.z) + radius_};
        return { lo, hi };
    }

private:
    Point3 apex_       = {0,1,0};
    Vec3   axis_       = {0,1,0};
    float  radius_     = 1.0f;
    float  height_     = 1.0f;
    int    material_id_ = 0;
    Point3 base_center_ = {0,0,0};

    bool hit_side(const Ray& r, float t_min, float t_max, HitRecord& out) const {
        // cone equation: |p - apex - axis*((p-apex).axis)|^2 = k^2 * ((p-apex).axis)^2 ...
        Vec3 oc = r.origin - apex_;
        float d_dot_a = r.direction.dot(axis_);
        float oc_dot_a = oc.dot(axis_);
        float k = radius_ / height_;
        float k2 = k * k;

        Vec3 d_perp = r.direction - axis_ * d_dot_a;
        Vec3 oc_perp = oc - axis_ * oc_dot_a;
        float a = d_perp.dot(d_perp) - k2 * d_dot_a * d_dot_a;
        float b = 2.0f * (d_perp.dot(oc_perp) - k2 * d_dot_a * oc_dot_a);
        float c = oc_perp.dot(oc_perp) - k2 * oc_dot_a * oc_dot_a;
        float disc = b * b - 4.0f * a * c;
        if (disc < 0.0f || std::fabs(a) < 1e-12f) return false;
        float sq = std::sqrt(disc);
        float t0 = (-b - sq) / (2.0f * a);
        float t1 = (-b + sq) / (2.0f * a);
        for (float t : {t0, t1}) {
            if (t < t_min || t > t_max) continue;
            float h = oc_dot_a + t * d_dot_a;
            if (h < 0.0f || h > height_) continue;
            out.t = t;
            out.p = r.at(t);
            // outward normal: gradient of the cone implicit
            Vec3 axis_pt = apex_ + axis_ * h;
            Vec3 radial = out.p - axis_pt;
            // outward normal = radial_hat - k * axis_hat (normalized)
            Vec3 outward_n = (radial.normalized() - axis_ * k).normalized();
            out.set_face_normal(r, outward_n);
            out.material_id = material_id_;
            return true;
        }
        return false;
    }

    bool hit_base(const Ray& r, float t_min, float t_max, HitRecord& out) const {
        Vec3 nrm = -axis_;
        float denom = nrm.dot(r.direction);
        if (std::fabs(denom) < 1e-6f) return false;
        float t = (base_center_ - r.origin).dot(nrm) / denom;
        if (t < t_min || t > t_max) return false;
        Point3 p = r.at(t);
        Vec3 d = p - base_center_;
        if (d.dot(d) > radius_ * radius_) return false;
        out.t = t;
        out.p = p;
        out.set_face_normal(r, nrm);
        out.material_id = material_id_;
        return true;
    }
};

} // namespace pencil
