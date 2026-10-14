// scene.hpp - scene container for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.2: holds an optional skybox texture for image-based background.

#pragma once

#include "bvh.hpp"
#include "caustics.hpp"
#include "fog.hpp"
#include "light.hpp"
#include "material.hpp"
#include "primitives.hpp"
#include "texture.hpp"

#include <memory>
#include <vector>

namespace pencil {

class Scene {
public:
    void add_material(const Material& m)  { materials_.push_back(m); }
    void add_light(const Light& l)        { lights_.push_back(l); }
    void add_primitive(std::shared_ptr<Primitive> p) {
        primitives_.push_back(std::move(p));
        bvh_built_ = false;
    }
    // v0.0.7: pop the last-added primitive. used by the displacement
    // tessellation path: we add the analytic sphere first, then if
    // the material has a displacement map we pop the sphere and add
    // the tessellated mesh instead (avoids z-fighting between the
    // two surfaces).
    void pop_primitive() {
        if (!primitives_.empty()) {
            primitives_.pop_back();
            bvh_built_ = false;
        }
    }

    int   material_count() const { return (int)materials_.size(); }
    int   light_count()    const { return (int)lights_.size(); }
    const std::vector<Material>& materials() const { return materials_; }
    const std::vector<Light>&    lights()    const { return lights_; }

    const Material& material(int id) const { return materials_[id]; }

    void build_bvh() {
        bvh_.build(primitives_);
        bvh_built_ = true;
    }

    bool use_bvh = true;

    bool hit(const Ray& r, float t_min, float t_max, HitRecord& out_rec) const {
        if (use_bvh && bvh_built_ && !bvh_.empty()) {
            return bvh_.hit(r, t_min, t_max, out_rec);
        }
        bool found = false;
        float closest = t_max;
        for (const auto& p : primitives_) {
            HitRecord tmp;
            if (p->hit(r, t_min, closest, tmp)) {
                closest = tmp.t;
                out_rec = tmp;
                found = true;
            }
        }
        return found;
    }

    bool occluded(const Ray& r, float t_min, float t_max) const {
        if (use_bvh && bvh_built_ && !bvh_.empty()) {
            return bvh_.occluded(r, t_min, t_max);
        }
        for (const auto& p : primitives_) {
            HitRecord tmp;
            if (p->hit(r, t_min, t_max, tmp)) return true;
        }
        return false;
    }

    Color background = {0.05f, 0.06f, 0.09f};
    Color ambient    = {0.06f, 0.06f, 0.08f};

    // v0.0.2: optional skybox. when set, rays that miss sample this
    // texture by direction instead of returning flat `background`.
    TexturePtr skybox;

    // v0.0.4: volumetric fog
    Fog fog;

    // v0.0.5: caustic photon map
    PhotonMap caustics;
    bool caustics_built = false;

    size_t primitive_count() const { return primitives_.size(); }

private:
    std::vector<Material>                       materials_;
    std::vector<Light>                          lights_;
    std::vector<std::shared_ptr<Primitive>>    primitives_;
    BVH                                         bvh_;
    bool                                        bvh_built_ = false;
};

} // namespace pencil
