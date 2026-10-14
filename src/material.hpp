// material.hpp - material model for pencil
// SPDX-License-Identifier: Apache-2.0
//
// supports: diffuse, phong, metal, dielectric, emissive.
// v0.0.2 adds: texture albedo, refraction dispersion toggle,
// shadow mode (hard/soft) per material.

#pragma once

#include "texture.hpp"
#include "vec3.hpp"

#include <memory>

namespace pencil {

enum class MaterialKind : int {
 Diffuse = 0,
 Metal = 1,
 Dielectric = 2,
 Principled = 3, // v0.0.7: disney-style principled bsdf
};

enum class ShadowMode : int {
 Hard = 0,
 Soft = 1,
};

struct Material {
 MaterialKind kind = MaterialKind::Diffuse;
 Color albedo = {0.8f, 0.8f, 0.8f};
 float reflectivity = 0.0f; // metal mirror factor [0,1]
 float ior = 1.0f; // index of refraction (1.5 ~ glass)
 float specular = 0.0f; // phong specular intensity
 float shininess = 32.0f; // phong exponent
 Color specular_color = {1.0f, 1.0f, 1.0f};
 Color emission = {0.0f, 0.0f, 0.0f};

 // v0.0.2: texture albedo (overrides albedo when present)
 TexturePtr texture;

 // v0.0.4: normal map (perturbs surface normals for fake detail)
 TexturePtr normal_map;

 // v0.0.7: true displacement map. when present, the renderer
 // tessellates the primitive into a triangle mesh and offsets each
 // vertex along its normal by (texture_value * displacement_amount).
 // this gives real silhouette + self-shadowing changes (unlike a
 // normal map which only fakes surface detail).
 TexturePtr displacement_map;
 float displacement_amount = 0.0f; // max offset in world units

 // v0.0.5: subsurface scattering (SSS).
 // 0 = off (opaque). >0 = amount of subsurface scattering.
 // light enters the surface, scatters inside, and exits nearby.
 // gives a soft "glow from within" look for wax, skin, marble, milk.
 float sss = 0.0f; // 0-1, how much light scatters
 float sss_radius = 0.5f; // scatter radius in world units
 Color sss_color = {1.0f, 0.5f, 0.3f}; // tint of scattered light

 // v0.0.2: refraction dispersion. 0 = off (uniform ior),
 // 1 = on (R=1.51, G=1.52, B=1.53, gives rainbow prism edges).
 int refdis = 0;

 // v0.0.7: disney principled bsdf parameters.
 // only used when kind == MaterialKind::Principled.
 float metallic = 0.0f;       // 0=dielectric, 1=metal
 float roughness = 0.5f;      // 0=mirror, 1=diffuse
 float specular_amt = 0.5f;  // 0-1, controls specular lobe intensity
 float specular_tint = 0.0f;  // 0=white, 1=tinted by albedo
 float sheen = 0.0f;          // 0-1, grazing retro-reflective
 float sheen_tint = 0.5f;     // 0-1
 float clearcoat = 0.0f;      // 0-1, second specular lobe
 float clearcoat_gloss = 1.0f;// 0-1
 float transmission = 0.0f;   // 0-1, glass-like transmission
 float ior_principled = 1.5f; // ior for transmission

 // v0.0.2: shadow mode. soft by default.
 ShadowMode shadow = ShadowMode::Soft;
};

inline Material make_diffuse(Color albedo,
 float spec = 0.0f,
 float shininess = 32.0f,
 Color spec_color = {1.0f, 1.0f, 1.0f}) {
 Material m;
 m.kind = MaterialKind::Diffuse;
 m.albedo = albedo;
 m.specular = spec;
 m.shininess = shininess;
 m.specular_color = spec_color;
 return m;
}

inline Material make_metal(Color albedo, float fuzz = 0.0f) {
 Material m;
 m.kind = MaterialKind::Metal;
 m.albedo = albedo;
 m.reflectivity = 1.0f - clampf(fuzz, 0.0f, 1.0f);
 m.specular = 0.0f;
 return m;
}

inline Material make_dielectric(float ior, Color tint = {1.0f, 1.0f, 1.0f}) {
 Material m;
 m.kind = MaterialKind::Dielectric;
 m.ior = ior;
 m.albedo = tint;
 m.reflectivity = 0.0f;
 return m;
}

inline Material make_emissive(Color emit) {
 Material m;
 m.kind = MaterialKind::Diffuse;
 m.albedo = {0.0f, 0.0f, 0.0f};
 m.emission = emit;
 return m;
}

// v0.0.7: principled bsdf constructor. defaults to a dielectric
// diffuse material; tweak the parameters for various looks:
//   metallic=1 -> mirror
//   roughness=0 -> smooth (sharp highlights)
//   transmission=1 -> glass
//   clearcoat=1 -> car-paint look
inline Material make_principled(Color albedo, float metallic = 0.0f,
                                float roughness = 0.5f) {
 Material m;
 m.kind = MaterialKind::Principled;
 m.albedo = albedo;
 m.metallic = metallic;
 m.roughness = roughness;
 m.specular_amt = 0.5f;
 m.specular_tint = 0.0f;
 m.sheen = 0.0f;
 m.sheen_tint = 0.5f;
 m.clearcoat = 0.0f;
 m.clearcoat_gloss = 1.0f;
 m.transmission = 0.0f;
 m.ior_principled = 1.5f;
 return m;
}

} // namespace pencil
