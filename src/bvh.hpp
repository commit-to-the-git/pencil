// bvh.hpp - bounding volume hierarchy for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.2: SAH (surface area heuristic) build for better mesh performance.
// the SAH minimizes expected ray-traversal cost by trying several split
// candidates and picking the one with the lowest cost.
// for small primitive counts it falls back to median split (cheap).

#pragma once

#include "primitives.hpp"
#include "ray.hpp"
#include "vec3.hpp"

#include <algorithm>
#include <cfloat>
#include <memory>
#include <vector>

namespace pencil {

class BVHNode {
public:
    AABB box;
    std::shared_ptr<Primitive> leaf;
    std::shared_ptr<BVHNode>   left;
    std::shared_ptr<BVHNode>   right;

    bool is_leaf() const { return (bool)leaf; }

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& out_rec) const {
        if (!box.hit(r, t_min, t_max)) return false;
        if (is_leaf()) return leaf->hit(r, t_min, t_max, out_rec);
        bool hit_l = left  && left ->hit(r, t_min, t_max, out_rec);
        bool hit_r = right && right->hit(r, t_min, hit_l ? out_rec.t : t_max, out_rec);
        return hit_l || hit_r;
    }

    bool occluded(const Ray& r, float t_min, float t_max) const {
        if (!box.hit(r, t_min, t_max)) return false;
        if (is_leaf()) {
            HitRecord tmp;
            return leaf->hit(r, t_min, t_max, tmp);
        }
        if (left  && left ->occluded(r, t_min, t_max)) return true;
        if (right && right->occluded(r, t_min, t_max)) return true;
        return false;
    }
};

class BVH {
public:
    // 0 = auto (use SAH for >12 prims, median for less).
    void build(std::vector<std::shared_ptr<Primitive>> prims) {
        storage_ = std::move(prims);
        if (storage_.empty()) { root_ = nullptr; return; }
        root_ = build_range(0, (int)storage_.size());
    }

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& out_rec) const {
        if (!root_) return false;
        return root_->hit(r, t_min, t_max, out_rec);
    }

    bool occluded(const Ray& r, float t_min, float t_max) const {
        if (!root_) return false;
        return root_->occluded(r, t_min, t_max);
    }

    bool empty() const { return storage_.empty(); }
    size_t size() const { return storage_.size(); }

private:
    std::shared_ptr<BVHNode> build_range(int start, int end) {
        auto node = std::make_shared<BVHNode>();
        AABB box;
        for (int i = start; i < end; ++i) box = AABB::merge(box, storage_[i]->aabb());
        node->box = box;

        int count = end - start;
        if (count == 1) {
            node->leaf = storage_[start];
            return node;
        }

        // pick axis via longest extent
        int axis = box.longest_axis();

        // for small counts, median split is fine.
        if (count <= 4) {
            median_split(start, end, axis);
        } else {
            sah_split(start, end, box, axis);
        }

        int split = start + count / 2;
        // after sah, the split point may not be the median; use the chosen split
        // (sah_split updates a member we need to expose).
        if (count > 4) split = last_split_;

        node->left  = build_range(start, split);
        node->right = build_range(split, end);
        return node;
    }

    void median_split(int start, int end, int axis) {
        auto mid = storage_.begin() + start;
        auto end_it = storage_.begin() + end;
        std::sort(mid, end_it, [axis](const std::shared_ptr<Primitive>& a,
                                       const std::shared_ptr<Primitive>& b) {
            AABB ba = a->aabb(), bb = b->aabb();
            return (ba.min[axis] + ba.max[axis]) < (bb.min[axis] + bb.max[axis]);
        });
        last_split_ = start + (end - start) / 2;
    }

    // sAH: try N candidate splits along `axis`, pick the one minimizing
    // the cost function: cost = traversal + (nL*SA_L + nR*SA_R) / SA_parent.
    void sah_split(int start, int end, const AABB& parent_box, int axis) {
        // bin the primitives into 12 buckets along the chosen axis.
        constexpr int NUM_BUCKETS = 12;
        struct Bucket {
            int count = 0;
            AABB bounds;
        };
        Bucket buckets[NUM_BUCKETS];

        float lo = parent_box.min[axis];
        float hi = parent_box.max[axis];
        if (hi <= lo) { median_split(start, end, axis); return; }

        for (int i = start; i < end; ++i) {
            AABB pb = storage_[i]->aabb();
            float center = (pb.min[axis] + pb.max[axis]) * 0.5f;
            int b = (int)((center - lo) / (hi - lo) * NUM_BUCKETS);
            if (b < 0) b = 0;
            if (b >= NUM_BUCKETS) b = NUM_BUCKETS - 1;
            buckets[b].count++;
            buckets[b].bounds = AABB::merge(buckets[b].bounds, pb);
        }

        // compute prefix/suffix bounds and counts.
        AABB prefix_bounds[NUM_BUCKETS + 1];
        int   prefix_counts[NUM_BUCKETS + 1] = {0};
        for (int i = 0; i < NUM_BUCKETS; ++i) {
            prefix_bounds[i+1] = AABB::merge(prefix_bounds[i], buckets[i].bounds);
            prefix_counts[i+1] = prefix_counts[i] + buckets[i].count;
        }

        AABB suffix_bounds[NUM_BUCKETS + 1];
        int   suffix_counts[NUM_BUCKETS + 1] = {0};
        for (int i = NUM_BUCKETS - 1; i >= 0; --i) {
            suffix_bounds[i] = AABB::merge(suffix_bounds[i+1], buckets[i].bounds);
            suffix_counts[i] = suffix_counts[i+1] + buckets[i].count;
        }

        // evaluate each split candidate
        float best_cost = FLT_MAX;
        int   best_bucket = -1;
        float sa_parent = surface_area(parent_box);
        for (int i = 1; i < NUM_BUCKETS; ++i) {
            int nL = prefix_counts[i];
            int nR = suffix_counts[i];
            if (nL == 0 || nR == 0) continue;
            float sa_L = surface_area(prefix_bounds[i]);
            float sa_R = surface_area(suffix_bounds[i]);
            float cost = 1.0f + (nL * sa_L + nR * sa_R) / (sa_parent > 0 ? sa_parent : 1.0f);
            if (cost < best_cost) {
                best_cost = cost;
                best_bucket = i;
            }
        }

        if (best_bucket < 0) { median_split(start, end, axis); return; }

        // partition primitives by their bucket.
        auto mid = storage_.begin() + start;
        auto end_it = storage_.begin() + end;
        auto pivot = std::partition(mid, end_it, [&](const std::shared_ptr<Primitive>& p) {
            AABB pb = p->aabb();
            float center = (pb.min[axis] + pb.max[axis]) * 0.5f;
            int b = (int)((center - lo) / (hi - lo) * NUM_BUCKETS);
            if (b < 0) b = 0;
            if (b >= NUM_BUCKETS) b = NUM_BUCKETS - 1;
            return b < best_bucket;
        });
        last_split_ = (int)(pivot - storage_.begin());
        // safety: ensure both sides have at least one primitive
        if (last_split_ <= start || last_split_ >= end) {
            median_split(start, end, axis);
        }
    }

    static float surface_area(const AABB& b) {
        Vec3 ext = b.max - b.min;
        return 2.0f * (ext.x * ext.y + ext.y * ext.z + ext.x * ext.z);
    }

    std::vector<std::shared_ptr<Primitive>> storage_;
    std::shared_ptr<BVHNode>                root_;
    int                                     last_split_ = 0;
};

} // namespace pencil
