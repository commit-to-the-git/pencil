// presets.hpp - built-in material presets for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.6: named material presets. use `prst: gold` in a mat block
// to get predefined material properties.

#pragma once

#include "material.hpp"
#include "vec3.hpp"

#include <cmath>
#include <string>

namespace pencil {

inline Material get_preset(const std::string& name) {
    Material m;

    if (name == "gold") {
        m.kind = MaterialKind::Metal;
        m.albedo = {0.95f, 0.75f, 0.30f};
        m.reflectivity = 0.95f;
        
    }
    else if (name == "copper") {
        m.kind = MaterialKind::Metal;
        m.albedo = {0.90f, 0.55f, 0.35f};
        m.reflectivity = 0.92f;
    }
    else if (name == "silver") {
        m.kind = MaterialKind::Metal;
        m.albedo = {0.95f, 0.95f, 0.95f};
        m.reflectivity = 0.95f;
    }
    else if (name == "iron" || name == "steel") {
        m.kind = MaterialKind::Metal;
        m.albedo = {0.60f, 0.60f, 0.65f};
        m.reflectivity = 0.85f;
    }
    else if (name == "chrome") {
        m.kind = MaterialKind::Metal;
        m.albedo = {1.0f, 1.0f, 1.0f};
        m.reflectivity = 1.0f;
    }
    else if (name == "glass" || name == "clear_glass") {
        m.kind = MaterialKind::Dielectric;
        m.ior = 1.5f;
        m.albedo = {1.0f, 1.0f, 1.0f};
    }
    else if (name == "frosted_glass") {
        m.kind = MaterialKind::Dielectric;
        m.ior = 1.5f;
        m.albedo = {0.95f, 0.95f, 0.95f};
    }
    else if (name == "water") {
        m.kind = MaterialKind::Dielectric;
        m.ior = 1.33f;
        m.albedo = {0.85f, 0.90f, 0.95f};
    }
    else if (name == "diamond") {
        m.kind = MaterialKind::Dielectric;
        m.ior = 2.42f;
        m.albedo = {1.0f, 1.0f, 1.0f};
    }
    else if (name == "rubber" || name == "plastic") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.15f, 0.15f, 0.15f};
        m.specular = 0.3f;
        m.shininess = 128.0f;
    }
    else if (name == "wood") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.45f, 0.30f, 0.15f};
    }
    else if (name == "marble") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.85f, 0.85f, 0.85f};
        m.specular = 0.5f;
        m.shininess = 64.0f;
    }
    else if (name == "skin") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.75f, 0.55f, 0.45f};
        m.sss = 0.3f;
        m.sss_radius = 0.3f;
        m.sss_color = {1.0f, 0.5f, 0.3f};
    }
    else if (name == "wax") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.90f, 0.85f, 0.70f};
        m.sss = 0.6f;
        m.sss_radius = 0.5f;
        m.sss_color = {1.0f, 0.8f, 0.5f};
    }
    else if (name == "lava" || name == "magma") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.0f, 0.0f, 0.0f};
        m.emission = {3.0f, 1.5f, 0.2f};
    }
    else if (name == "neon_blue") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.0f, 0.0f, 0.0f};
        m.emission = {0.2f, 0.5f, 3.0f};
    }
    else if (name == "neon_red") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.0f, 0.0f, 0.0f};
        m.emission = {3.0f, 0.1f, 0.05f};
    }
    else if (name == "neon_green") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.0f, 0.0f, 0.0f};
        m.emission = {0.1f, 3.0f, 0.2f};
    }
    else if (name == "mirror") {
        m.kind = MaterialKind::Metal;
        m.albedo = {1.0f, 1.0f, 1.0f};
        m.reflectivity = 1.0f;
    }
    else if (name == "concrete" || name == "cement") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.55f, 0.55f, 0.50f};
    }
    else if (name == "grass") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.20f, 0.50f, 0.15f};
    }
    else if (name == "sand") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.80f, 0.70f, 0.50f};
    }
    else if (name == "ice") {
        m.kind = MaterialKind::Dielectric;
        m.ior = 1.31f;
        m.albedo = {0.95f, 0.97f, 1.0f};
    }
    else if (name == "obsidian") {
        m.kind = MaterialKind::Diffuse;
        m.albedo = {0.03f, 0.03f, 0.05f};
        m.specular = 0.8f;
        m.shininess = 256.0f;
    }
    else if (name == "gold_brushed") {
        m.kind = MaterialKind::Metal;
        m.albedo = {0.95f, 0.75f, 0.30f};
        m.reflectivity = 0.7f;
    }
    else {
        // unknown preset   return default diffuse
        m = make_diffuse({0.7f, 0.7f, 0.7f}, 0.0f, 32.0f);
    }

    return m;
}

inline bool is_valid_preset(const std::string& name) {
    return name == "gold" || name == "copper" || name == "silver" ||
           name == "iron" || name == "steel" || name == "chrome" ||
           name == "glass" || name == "clear_glass" || name == "frosted_glass" ||
           name == "water" || name == "diamond" || name == "rubber" ||
           name == "plastic" || name == "wood" || name == "marble" ||
           name == "skin" || name == "wax" || name == "lava" || name == "magma" ||
           name == "neon_blue" || name == "neon_red" || name == "neon_green" ||
           name == "mirror" || name == "concrete" || name == "cement" ||
           name == "grass" || name == "sand" || name == "ice" ||
           name == "obsidian" || name == "gold_brushed";
}

} // namespace pencil
