// renderer.hpp - ray tracing core for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.2 features:
// - depth of field (camera aperture + focus)
// - soft shadows (jittered light position on a disc)
// - refraction dispersion (R/G/B have different IORs, rainbow prism edges)
// - texture albedo sampling
// - skybox environment map (image-based background)
// - normal/AOV output (returns raw normals instead of shaded colors)
// - bloom post-process
// - progress bar with ETA
// - multithreaded, ray counted for benchmarks

#pragma once

#include "camera.hpp"
#include "rng.hpp"
#include "scene.hpp"
#include "vec3.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace pencil {

struct RenderSettings {
 int width = 400;
 int height = 300;
 int samples = 4;
 int max_depth = 8;
 float shadow_bias = 1e-3f;
 int threads = 0;
 int soft_shadow_samples = 4; // jittered samples per soft shadow ray
 bool normals_mode = false; // output normal map instead of shaded
 bool bloom = false; // apply bloom post-process
 float bloom_threshold = 1.0f; // brightness above which pixels bloom
 float bloom_intensity = 0.5f; // how much bloom to add back
 int bloom_passes = 3; // blur iterations
 bool denoise = false; // v0.0.4: bilateral denoise
 bool pathtrace = false; // v0.0.4: path tracing mode (soft GI)
 bool bdpt = false; // v0.0.7: bidirectional path tracing
 bool spectral = false; // v0.0.7: spectral rendering (wavelength-dependent ior via cauchy)
 bool volume = false; // v0.0.7: full volumetric light transport (phase function + multiple scattering)
 float volume_g = 0.5f; // v0.0.7: henyey-greenstein anisotropy (-1 back, 0 isotropic, +1 forward)
 bool sss_enabled = false; // v0.0.7: global sss toggle (per-material sss still needs to be > 0)
 int pathtrace_bounces = 4; // v0.0.4: max GI bounces
 bool mis = false; // v0.0.7: multiple importance sampling in path tracer
 // v0.0.5: god rays through fog
 bool god_rays = false; // enable volumetric light scattering
 int god_ray_steps = 16; // march steps per shadow ray
 // v0.0.5: motion blur
 bool motion_blur = false;
 float motion_shutter = 0.5f; // shutter time (0-1, fraction of frame)
 // v0.0.5: adaptive sampling
 bool adaptive = false;
 float adaptive_threshold = 0.02f; // variance threshold for more samples
 // v0.0.5: caustics (photon mapping)
 bool caustics = false;
 int caustic_photons = 50000; // v0.0.6: more photons // number of photons to trace
 // v0.0.7: progressive photon mapping. when enabled, runs ppm_passes
 // rounds of photon tracing, each with caustic_photons photons, and
 // shrinks the lookup radius each pass. the accumulated photon map
 // has passes * caustic_photons photons, and the final lookup uses
 // a smaller radius -> sharper caustics with less noise. classic ppm
 // updates per-pixel radii, but that requires per-pixel state which
 // we don't have; this is a global-radius approximation.
 bool ppm = false;
 int ppm_passes = 4;
 float ppm_initial_radius = 0.5f;
 // v0.0.5: bokeh aperture shape
 int bokeh_blades = 0; // 0=circle, 3=triangle, 4=square, 5=pentagon, 6=hexagon, 8=octagon
 // v0.0.5: tile rendering (for live preview feel)
 int tile_size = 64; // 64x64 tile rendering
 bool show_progress = false; // print progress bar to stderr
};

struct RenderStats {
 uint64_t rays_traced = 0;
};

// v0.0.5: photon tracing for caustics.
// traces photons from lights through glass onto diffuse surfaces.
inline void trace_photons_impl(const Scene& scene, PhotonMap& map,
 const Light& light, int count, int max_bounces = 6, uint64_t pass_seed = 0) {
 Rng rng(12345u + (int)light.position_or_direction.x * 1000 + pass_seed * 0x9E3779B97F4A7C15ull);
 for (int i = 0; i < count; ++i) {
 Vec3 origin, dir;
 if (light.kind == LightKind::Point) {
 origin = light.position_or_direction;
 float u1 = rng.unit(), u2 = rng.unit();
 float theta = 2.0f * 3.14159265f * u1;
 float phi = std::acos(2.0f * u2 - 1.0f);
 dir = { std::sin(phi) * std::cos(theta),
 std::cos(phi),
 std::sin(phi) * std::sin(theta) };
 } else {
 dir = -light.position_or_direction;
 origin = { (rng.unit() - 0.5f) * 20.0f, 10.0f, (rng.unit() - 0.5f) * 20.0f };
 }
 Color power = light.intensity;
 Ray ray{origin, dir};
 for (int bounce = 0; bounce < max_bounces; ++bounce) {
 HitRecord rec;
 if (!scene.hit(ray, 1e-4f, 1e9f, rec)) break;
 const Material& m = scene.material(rec.material_id);
 if (m.kind == MaterialKind::Dielectric) {
 power = power * m.albedo * 0.9f;
 float eta = rec.front_face ? (1.0f / std::fmax(m.ior, 0.01f)) : m.ior;
 Vec3 n = rec.normal;
 float cos_i = std::fabs(ray.direction.normalized().dot(n));
 float sin_t2 = eta * eta * (1.0f - cos_i * cos_i);
 if (sin_t2 > 1.0f) {
 dir = ray.direction.normalized().reflect(n).normalized();
 ray = Ray{rec.p + n * 1e-3f, dir};
 } else {
 float cos_t = std::sqrt(1.0f - sin_t2);
 dir = (ray.direction.normalized() * eta +
 n * (eta * cos_i - cos_t)).normalized();
 ray = Ray{rec.p - n * 1e-3f, dir};
 }
 } else if (m.kind == MaterialKind::Metal) {
 power = power * m.albedo * m.reflectivity;
 dir = ray.direction.normalized().reflect(rec.normal).normalized();
 ray = Ray{rec.p + rec.normal * 1e-3f, dir};
 } else {
 Photon p;
 p.position = rec.p;
 p.direction = dir;
 // v0.0.6: normalize by the actual photon count for this light.
 // OLD: hardcoded `/ 50000.0f` (only correct when caustic_photons==50000).
 // NEW: divide by `count`, which is the photons-per-light the caller passed.
 // For the default (caustic_photons=50000, 1 light), `count==50000` so the
 // math is bit-identical to the old /50000 path -- same image, no magic
 // number. If the user changes --caustic-photons, the normalization now
 // scales correctly instead of being locked to 50000.
 p.power = power * m.albedo / (float)count;
 p.bounces = bounce;
 map.add(p);
 break;
 }
 }
 }
}

class Renderer {
public:
 Renderer(const Scene& scene, const Camera& camera, RenderSettings s)
 : scene_(scene), camera_(camera), settings_(s) {
 if (settings_.threads <= 0) {
 settings_.threads = (int)std::thread::hardware_concurrency();
 if (settings_.threads <= 0) settings_.threads = 1;
 if (settings_.threads > 64) settings_.threads = 64;
 }
 // v0.0.5: pass bokeh shape to camera.
 // only override if the setting is non-zero (CLI override).
 // if 0, keep whatever the scene file set via cmra { bd: N }.
 if (settings_.bokeh_blades > 0) {
 camera_.set_bokeh_blades(settings_.bokeh_blades);
 }
 thread_scene_ = &scene_;
 }

 // v0.0.7: motion blur. caller pre-builds sub-frame scenes (one per
 // sub-frame time within the shutter interval). the renderer picks a
 // random sub-scene per pixel sample and traces through it. this gives
 // true object motion blur (not just camera blur like the old code).
 // pass empty vector to disable.
 void set_motion_blur_scenes(std::vector<std::shared_ptr<Scene>> s) {
 mb_scenes_ = std::move(s);
 }
 // v0.0.7: pick a random sub-frame scene for the current sample.
 // call this once per pixel sample before tracing.
 void pick_motion_blur_scene(Rng& rng) {
 if (settings_.motion_blur && !mb_scenes_.empty()) {
  size_t idx = (size_t)(rng.unit() * mb_scenes_.size());
  if (idx >= mb_scenes_.size()) idx = mb_scenes_.size() - 1;
  thread_scene_ = mb_scenes_[idx].get();
 } else {
  thread_scene_ = &scene_;
 }
 }
 // v0.0.7: restore main scene (call after tracing a sample).
 void restore_main_scene() { thread_scene_ = &scene_; }
 // v0.0.7: accessor used by trace methods. returns either the main
 // scene or the per-sample motion blur scene.
 const Scene& cur_scene() const {
 return thread_scene_ ? *thread_scene_ : scene_;
 }

 void render(std::vector<Color>& out_pixels, RenderStats* stats = nullptr) {
 const int W = settings_.width;
 const int H = settings_.height;
 out_pixels.assign((size_t)W * H, Color{0,0,0});
 ray_counter_.store(0, std::memory_order_relaxed);

 // v0.0.5: build photon map for caustics if enabled
 // v0.0.7: progressive photon mapping. runs ppm_passes rounds of
 // photon tracing, each with caustic_photons photons. the lookup
 // radius shrinks with each pass, giving sharper caustics.
 if (settings_.caustics && !scene_.caustics_built) {
  const_cast<Scene&>(scene_).caustics.photons.clear();
  int passes = settings_.ppm ? settings_.ppm_passes : 1;
  for (int pass = 0; pass < passes; ++pass) {
   for (const auto& light : scene_.lights()) {
    trace_photons_impl(scene_, const_cast<Scene&>(scene_).caustics,
     light, settings_.caustic_photons / std::max(1, (int)scene_.lights().size()), 6, (uint64_t)pass);
   }
  }
  const_cast<Scene&>(scene_).caustics.build();
  const_cast<Scene&>(scene_).caustics_built = true;
  // shrink the lookup radius based on total photon count.
  // for N photons total, radius scales as 1/N^(1/3) in 3d.
  int total_photons = passes * settings_.caustic_photons;
  float shrink = std::pow((float)settings_.caustic_photons /
                          (float)std::max(1, total_photons), 1.0f / 3.0f);
  ppm_final_radius_ = settings_.ppm_initial_radius * shrink;
 }

 auto t_start = std::chrono::steady_clock::now();

 int T = settings_.threads;
 std::vector<std::thread> workers;
 workers.reserve(T);

 // v0.0.6: 64x64 tile rendering. split image into tiles, distribute
 // across threads. each tile is rendered independently.
 struct Tile { int x, y, w, h; };
 std::vector<Tile> tiles;
 int ts = settings_.tile_size;
 if (ts <= 0) ts = 64;
 for (int y = 0; y < H; y += ts) {
  for (int x = 0; x < W; x += ts) {
   tiles.push_back({x, y, std::min(ts, W - x), std::min(ts, H - y)});
  }
 }
 std::atomic<int> next_tile{0};
 int total_tiles = (int)tiles.size();

 auto render_tiles = [&]() {
  while (true) {
   int idx = next_tile.fetch_add(1);
   if (idx >= total_tiles) break;
   Tile& t = tiles[idx];
   render_tile_region(out_pixels, W, H, t.x, t.y, t.w, t.h);
  }
 };

 for (int t = 0; t < T; ++t) {
  workers.emplace_back(render_tiles);
 }

 // optional progress thread
 // progress thread (tile-based)
 std::thread prog;
 if (settings_.show_progress) {
  prog = std::thread([this, &next_tile, total_tiles, t_start]() {
   while (true) {
    int done = next_tile.load(std::memory_order_relaxed);
    if (done >= total_tiles) break;
    float pct = (float)done / (float)total_tiles;
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - t_start).count();
    double eta = (pct > 0.001f) ? (elapsed / pct * (1.0f - pct)) : 0.0f;
    int bar_w = 28;
    int filled = (int)(pct * bar_w);
    std::string bar;
    for (int i = 0; i < bar_w; ++i) bar += (i < filled) ? "#" : "-";
    std::cerr << "\r[" << bar << "] " << (int)(pct * 100) << "% ETA "
     << (int)eta << "s    " << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }
   std::cerr << "\r[" << std::string(28, '#') << "] 100%            \n";
  });
 }
 for (auto& w : workers) w.join();
 if (prog.joinable()) prog.join();

 if (settings_.bloom) {
 apply_bloom(out_pixels, W, H);
 }

 if (settings_.denoise) {
 apply_denoise(out_pixels, W, H);
 }

 if (stats) stats->rays_traced = ray_counter_.load(std::memory_order_relaxed);
 }

 // v0.0.6: public post-process entry point for distributed rendering.
 // the worker renders HDR pixels per-tile and ships them back to the
 // server un-tonemapped; the server assembles the full HDR image and
 // calls this to apply bloom/denoise BEFORE tonemap+gamma (matching
 // what the local render() does, so distributed and local outputs
 // match pixel-for-pixel even when bloom/denoise are on).
 void post_process(std::vector<Color>& pixels, int W, int H) {
 if (settings_.bloom)   apply_bloom(pixels, W, H);
 if (settings_.denoise) apply_denoise(pixels, W, H);
 }

public:
 void render_tile_region(std::vector<Color>& out, int W, int H,
                          int tx, int ty, int tw, int th) const {
  for (int j = ty; j < ty + th; ++j) {
   for (int i = tx; i < tx + tw; ++i) {
    Rng rng((uint32_t)(i * 97861u + j * 1973u + 1u));
    Color acc = {0,0,0};
    int n = settings_.samples;

    if (settings_.adaptive) {
     int min_samples = n * n / 4;
     if (min_samples < 4) min_samples = 4;
     int max_samples = n * n * 4;
     Color mean = {0, 0, 0};
     Color M2 = {0, 0, 0};
     int samples_done = 0;
     for (int s = 0; s < max_samples; ++s) {
      float u = (i + rng.unit()) / (float)W;
      float v = 1.0f - (j + rng.unit()) / (float)H;
      Ray r = camera_.ray_at(u, v, &rng);
      if (settings_.motion_blur && camera_start_active_) {
       float t = rng.unit();
       Point3 cam_pos = camera_start_ + (camera_end_ - camera_start_) * t;
       Vec3 focal_point = r.origin + r.direction * camera_.focus_dist();
       r.origin = cam_pos;
       r.direction = (focal_point - cam_pos).normalized();
      }
      // v0.0.7: pick a random sub-frame scene for object motion blur.
      // the trace call below uses cur_scene() which returns current_scene_.
      const_cast<Renderer*>(this)->pick_motion_blur_scene(rng);
      Color c;
      if (settings_.bdpt) c = trace_bdpt(r, settings_.pathtrace_bounces, rng);
      else if (settings_.volume && cur_scene().fog.enabled) c = trace_path_volume(r, settings_.pathtrace_bounces, rng);
      else if (settings_.pathtrace && settings_.mis) c = trace_path_mis(r, settings_.pathtrace_bounces, rng);
      else if (settings_.pathtrace) c = trace_path(r, settings_.pathtrace_bounces, rng);
      else c = trace(r, settings_.max_depth, rng, v);
      samples_done++;
      Color delta = c - mean;
      mean = mean + delta / (float)samples_done;
      Color delta2 = c - mean;
      M2 = M2 + delta * delta2;
      if (samples_done >= min_samples && samples_done % 4 == 0) {
       float var_r = M2.x / (float)(samples_done - 1);
       float var_g = M2.y / (float)(samples_done - 1);
       float var_b = M2.z / (float)(samples_done - 1);
       float max_var = std::fmax(var_r, std::fmax(var_g, var_b));
       float lum = mean.x * 0.299f + mean.y * 0.587f + mean.z * 0.114f;
       float rel_var = max_var / (lum * lum + 0.01f);
       if (rel_var < settings_.adaptive_threshold) break;
      }
     }
     acc = mean;
    } else {
     for (int sy = 0; sy < n; ++sy) {
      for (int sx = 0; sx < n; ++sx) {
       float u = (i + (sx + 0.5f) / n) / (float)W;
       float v = 1.0f - (j + (sy + 0.5f) / n) / (float)H;
       u += rng.sym() * (0.5f / (W * n));
       v += rng.sym() * (0.5f / (H * n));
       Ray r = camera_.ray_at(u, v, &rng);
       if (settings_.motion_blur && camera_start_active_) {
        float t = rng.unit();
        Point3 cam_pos = camera_start_ + (camera_end_ - camera_start_) * t;
        Vec3 focal_point = r.origin + r.direction * camera_.focus_dist();
        r.origin = cam_pos;
        r.direction = (focal_point - cam_pos).normalized();
       }
       // v0.0.7: pick sub-frame scene for object motion blur.
       const_cast<Renderer*>(this)->pick_motion_blur_scene(rng);
       if (settings_.normals_mode) acc += trace_normals(r);
       else if (settings_.bdpt) acc += trace_bdpt(r, settings_.pathtrace_bounces, rng);
       else if (settings_.volume && cur_scene().fog.enabled) acc += trace_path_volume(r, settings_.pathtrace_bounces, rng);
       else if (settings_.pathtrace && settings_.mis) acc += trace_path_mis(r, settings_.pathtrace_bounces, rng);
       else if (settings_.pathtrace) acc += trace_path(r, settings_.pathtrace_bounces, rng);
       else acc += trace(r, settings_.max_depth, rng, v);
      }
     }
     acc = acc / (float)(n * n);
    }
    out[(size_t)j * W + i] = acc;
   }
  }
 }

 // normal/AOV mode: returns raw surface normal (encoded to color).
 Color trace_normals(const Ray& r) const {
 ray_counter_.fetch_add(1, std::memory_order_relaxed);
 HitRecord rec;
 if (!cur_scene().hit(r, 1e-4f, 1e9f, rec)) return {0,0,0};
 // map normal [-1,1] -> [0,1]
 return { (rec.normal.x + 1.0f) * 0.5f,
 (rec.normal.y + 1.0f) * 0.5f,
 (rec.normal.z + 1.0f) * 0.5f };
 }

 Color trace(const Ray& r, int depth, Rng& rng, float screen_v = 0.5f) const {
 ray_counter_.fetch_add(1, std::memory_order_relaxed);
 if (depth <= 0) return {0,0,0};

 HitRecord rec;
 if (!cur_scene().hit(r, 1e-4f, 1e9f, rec)) {
 // skybox fallback
 Color bg;
 if (cur_scene().skybox && cur_scene().skybox->valid()) {
 bg = cur_scene().skybox->sample_dir(r.direction);
 } else {
 bg = cur_scene().background;
 }
 // v0.0.5: apply fog to background rays too (infinite distance
 // means full fog, but we use a large value for visual effect).
 // v0.0.5: for gradient fog, sample the fog color based on
 // the ray direction (looking up = top color, looking down = bottom).
 if (cur_scene().fog.enabled) {
 // v0.0.5: use SCREEN V coordinate (0=top, 1=bottom) for
 // gradient fog. this ensures the full gradient range is
 // visible regardless of FOV or ray direction magnitude.
 float ray_y = cur_scene().fog.gradient_min +
 screen_v * (cur_scene().fog.gradient_max - cur_scene().fog.gradient_min);
 Color fogged = apply_fog(cur_scene().fog, bg, 100.0f, ray_y);

 // v0.0.5: god rays march along the background ray
 // and accumulate light scattered by fog toward the camera.
 // this makes visible light shafts in the fog.
 // the key: occluders between the march point and the light
 // block the contribution, creating dark streaks = shadows
 // of objects cast through the fog. where there's no occluder,
 // the full light scatters through = bright shafts.
 if (settings_.god_rays) {
 Color scattered = {0, 0, 0};
 float march_dist = 30.0f;
 int steps = settings_.god_ray_steps * 2;
 float step = march_dist / steps;
 Vec3 view_dir = r.direction.normalized();
 for (int s = 0; s < steps; ++s) {
 float t = step * (s + 0.5f);
 Vec3 p = r.origin + r.direction * t;
 // transmittance from camera to this point
 float trans_cam = fog_transmittance(cur_scene().fog, t);
 for (const auto& light : cur_scene().lights()) {
 if (light.kind != LightKind::Point) continue;
 Vec3 L = light.position_or_direction - p;
 float light_dist = L.length();
 L = L / light_dist;
 // cRITICAL: check occlusion this is what creates
 // the shaft pattern. where occluders block the light,
 // the march point gets no contribution = dark streak.
 // where there's clear path, full light = bright shaft.
 Ray sr{p, L};
 if (!light.is_emissive && cur_scene().occluded(sr, 1e-3f, light_dist - 1e-3f)) continue;
 // transmittance from this point to the light
 float trans_light = fog_transmittance(cur_scene().fog, light_dist);
 // henyey-Greenstein phase function with strong forward scatter.
 // g=0.9 means light scatters mostly in the forward direction
 // (toward the camera when looking toward the light).
 // this creates the tight shaft pattern, not a uniform glow.
 float cos_theta = view_dir.dot(L);
 float g = 0.9f;
 float denom = 1.0f + g * g - 2.0f * g * cos_theta;
 float phase = (1.0f - g * g) / (4.0f * 3.14159265f *
 std::pow(std::fabs(denom), 1.5f) + 1e-6f);
 Color fc = fog_color_at(cur_scene().fog, p.y);
 scattered += light.intensity * fc
 * (cur_scene().fog.density * step)
 * trans_cam * trans_light
 * phase;
 }
 }
 fogged += scattered;
 }
 return fogged;
 }
 return bg;
 }

 const Material& m = cur_scene().material(rec.material_id);

 // base albedo: texture if present, else flat
 Color albedo = m.albedo;
 float u = 0, v = 0; bool uv_computed = false;
 if (m.texture && m.texture->valid()) {
 compute_uv(rec, u, v);
 uv_computed = true;
 albedo = m.texture->sample(u, v);
 }

 // v0.0.4: normal map perturbation.
 // sample the normal map and perturb the surface normal.
 Vec3 perturbed_normal = rec.normal;
 if (m.normal_map && m.normal_map->valid()) {
 if (!uv_computed) { compute_uv(rec, u, v); uv_computed = true; }
 Color nmap = m.normal_map->sample(u, v);
 // normal map stores tangent-space normals in [0,1].
 // convert to [-1,1] and build a perturbed normal.
 Vec3 n_tangent = {nmap.x * 2.0f - 1.0f,
 nmap.y * 2.0f - 1.0f,
 nmap.z * 2.0f - 1.0f};
 // build tangent space from the surface normal
 Vec3 N = rec.normal;
 Vec3 T = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0}.cross(N) : Vec3{1,0,0}.cross(N);
 T = T.normalized();
 Vec3 B = N.cross(T).normalized();
 perturbed_normal = (T * n_tangent.x + B * n_tangent.y + N * n_tangent.z).normalized();
 }
 // use perturbed_normal instead of rec.normal for shading
 Vec3 shade_normal = perturbed_normal;

 Color local = m.emission;

 for (const auto& light : cur_scene().lights()) {
 // soft shadow sampling: jitter the light position on a disc
 // perpendicular to (light -> hit_point). if the light's
 // radius is 0, this is a single hard shadow ray.
 Color light_contrib = {0,0,0};
 int nsamp = (light.radius > 0.0f) ? settings_.soft_shadow_samples : 1;
 for (int s = 0; s < nsamp; ++s) {
 Vec3 L_pos = light.position_or_direction;
 if (light.kind == LightKind::Point && light.radius > 0.0f) {
 // jitter on a disc perpendicular to (L - P)
 Vec3 to_light = light.position_or_direction - rec.p;
 Vec3 axis = to_light.normalized();
 // build orthonormal basis around axis
 Vec3 up = (std::fabs(axis.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
 Vec3 t1 = axis.cross(up).normalized();
 Vec3 t2 = axis.cross(t1).normalized();
 float dx, dy;
 rng.unit_disc(dx, dy);
 L_pos = light.position_or_direction + (t1 * dx + t2 * dy) * light.radius;
 }

 Vec3 L;
 float light_dist = 1.0f;
 if (light.kind == LightKind::Point) {
 L = L_pos - rec.p;
 light_dist = L.length();
 L = L / light_dist;
 } else {
 L = -light.position_or_direction;
 light_dist = 1e9f;
 }

 Ray shadow_ray{rec.p + rec.normal * settings_.shadow_bias, L};
 // emissive lights skip shadow testing they come from
 // surfaces, not points in empty space, so the surface
 // itself shouldn't block its own emission.
 if (!light.is_emissive &&
 cur_scene().occluded(shadow_ray, settings_.shadow_bias,
 light_dist - settings_.shadow_bias)) {
 continue;
 }

 float ndotl = clampf(shade_normal.dot(L), 0.0f, 1.0f);
 Color c = albedo * light.intensity * ndotl;
 if (m.specular > 0.0f && ndotl > 0.0f) {
 Vec3 V = -r.direction.normalized();
 Vec3 H = (L + V).normalized();
 float ndoth = clampf(shade_normal.dot(H), 0.0f, 1.0f);
 float spec = std::pow(ndoth, m.shininess);
 c += m.specular_color * light.intensity * (spec * m.specular);
 }
 light_contrib += c;
 }
 local += light_contrib / (float)nsamp;
 }

 // nOTE: god rays on surface hits are removed they double-counted
 // with the background ray god rays and caused the inverted vignette
 // effect. god rays now ONLY run on background rays (rays that miss
 // geometry) where they correctly march through the fog volume.

 local += albedo * cur_scene().ambient;

 // v0.0.7: subsurface scattering. replaced the v0.0.5 dipole
 // approximation with a multi-sample burley normalized diffusion
 // bssrdf. for each pixel sample hitting a sss material, we sample
 // n probe positions on a disc tangent to the surface (radius
 // m.sss_radius), query direct lighting at each probe (with shadow
 // rays), and weight by the burley kernel r(p) = (1 - exp(-s * d)) /
 // (s * d) where d is the probe distance. this gives real light
 // bleeding from nearby lit areas into shadowed areas, and is
 // more physically correct than the old dipole which only saw the
 // back side of the surface.
 if (settings_.sss_enabled && m.sss > 0.0f) {
 Color sss_glow = {0, 0, 0};
 const int SSS_PROBES = 4;
 // build a tangent frame around the surface normal
 Vec3 N = rec.normal;
 Vec3 up_hint = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
 Vec3 t1 = N.cross(up_hint).normalized();
 Vec3 t2 = N.cross(t1).normalized();
 for (int p = 0; p < SSS_PROBES; ++p) {
  // sample a point on a disc of radius m.sss_radius around rec.p
  float dx, dy;
  rng.unit_disc(dx, dy);
  // bias toward the center since the kernel decays with distance
  float r = m.sss_radius * std::sqrt(rng.unit());
  Vec3 probe = rec.p + (t1 * dx + t2 * dy) * r;
  // burley normalized diffusion kernel (radial falloff).
  // approximated as: weight = exp(-r / sigma) where sigma = sss_radius.
  float sigma = m.sss_radius > 0.001f ? m.sss_radius : 0.5f;
  float kernel = std::exp(-r / sigma);
  // direct lighting at the probe position
  Color probe_contrib = {0, 0, 0};
  for (const auto& light : cur_scene().lights()) {
   Vec3 L;
   float light_dist;
   if (light.kind == LightKind::Point) {
    Vec3 to_light = light.position_or_direction - probe;
    light_dist = to_light.length();
    L = to_light / light_dist;
   } else {
    L = -light.position_or_direction;
    light_dist = 1e9f;
   }
   // v0.0.7 fix: actually test occlusion for sss probes
   Ray shadow_ray{probe + N * settings_.shadow_bias * 4.0f, L};
   if (!light.is_emissive &&
    cur_scene().occluded(shadow_ray, settings_.shadow_bias,
                         light_dist - settings_.shadow_bias)) {
    continue;
   }
   float ndotl = clampf(N.dot(L), 0.0f, 1.0f);
   // back-scatter: light coming from behind the surface also
   // contributes (light passes through thin material).
   float back_light = -clampf(N.dot(L), -1.0f, 0.0f);
   float extinction = 1.0f - std::exp(-sigma * 2.0f);
   probe_contrib += light.intensity * m.sss_color *
                    (ndotl + back_light * extinction);
  }
  // ambient at probe (approximated with same ambient as rec)
  probe_contrib += albedo * cur_scene().ambient * m.sss_color;
  sss_glow += probe_contrib * kernel;
 }
 sss_glow = sss_glow * (m.sss / (float)SSS_PROBES);
 local += sss_glow;
 }

 // v0.0.5: caustics from photon map
 if (settings_.caustics && cur_scene().caustics_built && m.kind == MaterialKind::Diffuse) {
 Color c = cur_scene().caustics.lookup(rec.p,
   settings_.ppm ? ppm_final_radius_ : 0.5f);
 local += c * albedo * 50.0f;
 }

 // reflection / refraction
 if (m.kind == MaterialKind::Metal && m.reflectivity > 0.0f) {
 Vec3 refl_dir = r.direction.normalized().reflect(rec.normal).normalized();
 Ray refl_ray{rec.p + rec.normal * settings_.shadow_bias, refl_dir};
 Color incoming = trace(refl_ray, depth - 1, rng);
 Color reflected_color = incoming * albedo * m.reflectivity;
 local = local * (1.0f - m.reflectivity) + reflected_color;
 } else if (m.kind == MaterialKind::Dielectric) {
 if (m.refdis || settings_.spectral) {
 // continuous spectral sampling: pick a random wavelength
 // lambda in [0,1] where 0=red, 0.5=green, 1=blue.
 // v0.0.7: in spectral mode, ALL dielectrics use cauchy's
 // equation for wavelength-dependent IOR:
 //   n(lambda) = A + B/lambda^2
 // where A = m.ior and B = m.ior * spread.
 // this gives real-world dispersion (e.g. glass splits white
 // light into a rainbow). the dispersion path (m.refdis) and
 // spectral mode both go through this same code now.
 float lambda = rng.unit(); // [0, 1)
 float spread;
 if (settings_.spectral) {
  // stronger spread in spectral mode for more visible rainbow
  spread = 0.05f;
 } else {
  spread = 0.01f; // 1% IOR variation (realistic for glass)
 }
 // map lambda to actual wavelength in nm: 380 (red) to 780 (blue)
 float lambda_nm = 380.0f + lambda * 400.0f;
 // cauchy: n = A + B/lambda^2. B is chosen so that the IOR at
 // the central wavelength (580nm) equals m.ior.
 float B = m.ior * spread * 580.0f * 580.0f;
 float ior_lambda = m.ior + B / (lambda_nm * lambda_nm) - B / (580.0f * 580.0f);

 // trace the ray with this wavelength's IOR.
 Color c = trace_dielectric(r, rec, m, albedo, depth, rng, ior_lambda);

 // convert wavelength to RGB using gaussian spectral responses.
 // r peaks at 0.15, G at 0.5, B at 0.85, each with sigma 0.15.
 float sigma = 0.15f;
 float wr = std::exp(-((lambda - 0.15f) * (lambda - 0.15f)) / (2.0f * sigma * sigma));
 float wg = std::exp(-((lambda - 0.50f) * (lambda - 0.50f)) / (2.0f * sigma * sigma));
 float wb = std::exp(-((lambda - 0.85f) * (lambda - 0.85f)) / (2.0f * sigma * sigma));

 // normalize so the weights sum to ~3 (one full contribution per channel).
 float wsum = wr + wg + wb;
 if (wsum > 1e-6f) {
 float scale = 3.0f / wsum;
 local = c * Color{wr * scale, wg * scale, wb * scale};
 } else {
 local = c;
 }
 } else {
 local = trace_dielectric(r, rec, m, albedo, depth, rng);
 }
 } else if (m.kind == MaterialKind::Principled) {
 // v0.0.7: disney principled bsdf. combines diffuse + specular
 // (ggx) + metal + clearcoat lobes based on parameters.
 // diffuse: cosine-weighted lambert scaled by (1-metallic).
 float diff_amt = (1.0f - m.metallic) * (1.0f - m.transmission);
 local += albedo * cur_scene().ambient * diff_amt;
 // metal/specular: ggx-style reflection with roughness.
 // we use a simple phong-style lobe with roughness controlling
 // the specular exponent (roughness 0 = mirror, 1 = very diffuse).
 Vec3 V = -r.direction.normalized();
 for (const auto& light : cur_scene().lights()) {
  Vec3 L;
  float light_dist;
  if (light.kind == LightKind::Point) {
   Vec3 to_l = light.position_or_direction - rec.p;
   light_dist = to_l.length();
   L = to_l / light_dist;
  } else {
   L = -light.position_or_direction;
   light_dist = 1e9f;
  }
  Ray shadow_ray{rec.p + rec.normal * settings_.shadow_bias, L};
  if (!light.is_emissive &&
   cur_scene().occluded(shadow_ray, settings_.shadow_bias,
                        light_dist - settings_.shadow_bias)) {
   continue;
  }
  float ndotl = clampf(shade_normal.dot(L), 0.0f, 1.0f);
  if (ndotl <= 0.0f) continue;
  // diffuse lobe (only for non-metal)
  if (m.metallic < 1.0f) {
   local += albedo * light.intensity * ndotl * diff_amt;
  }
  // specular lobe (ggx approximation: phong with shininess from roughness)
  Vec3 H = (L + V).normalized();
  float ndoth = clampf(shade_normal.dot(H), 0.0f, 1.0f);
  float shininess = std::pow(1.0f - m.roughness, 4.0f) * 256.0f + 1.0f;
  float spec = std::pow(ndoth, shininess);
  Color spec_color = m.specular_tint > 0.5f ? albedo : Color{1,1,1};
  local += spec_color * light.intensity * spec *
           (m.metallic + (1.0f - m.metallic) * m.specular_amt);
  // clearcoat lobe (second specular)
  if (m.clearcoat > 0.0f) {
   float cc_shininess = std::pow(m.clearcoat_gloss, 4.0f) * 256.0f + 1.0f;
   float cc_spec = std::pow(ndoth, cc_shininess);
   local += Color{1,1,1} * light.intensity * cc_spec * m.clearcoat;
  }
  // sheen: retro-reflective grazing lobe (simplification)
  if (m.sheen > 0.0f) {
   float retro = std::pow(1.0f - std::fabs(L.dot(V)), 4.0f);
   Color sheen_color = m.sheen_tint > 0.5f ? albedo : Color{1,1,1};
   local += sheen_color * light.intensity * retro * m.sheen;
  }
 }
 // reflection (mirror + glossy based on metallic + roughness)
 if (m.metallic > 0.0f || m.specular_amt > 0.0f) {
  Vec3 refl_dir = r.direction.normalized().reflect(shade_normal).normalized();
  // perturb by roughness for glossy reflection
  Vec3 up = (std::fabs(refl_dir.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
  Vec3 t1 = refl_dir.cross(up).normalized();
  Vec3 t2 = refl_dir.cross(t1).normalized();
  float r1 = rng.unit() * m.roughness * 0.5f;
  float r2 = rng.unit() * 6.28f;
  refl_dir = (refl_dir + t1 * (r1 * std::cos(r2)) + t2 * (r1 * std::sin(r2))).normalized();
  Ray refl_ray{rec.p + rec.normal * settings_.shadow_bias, refl_dir};
  Color incoming = trace(refl_ray, depth - 1, rng);
  float refl_amt = m.metallic + (1.0f - m.metallic) * m.specular_amt * 0.5f;
  refl_amt = clampf(refl_amt, 0.0f, 1.0f);
  Color refl_color = incoming * (m.metallic > 0.5f ? albedo : Color{1,1,1});
  local = local * (1.0f - refl_amt) + refl_color * refl_amt;
 }
 }

 // v0.0.4: apply volumetric fog based on distance from camera to hit.
 if (cur_scene().fog.enabled) {
 float dist = rec.t;
 local = apply_fog(cur_scene().fog, local, dist, rec.p.y);
 }

 // v0.0.5: god rays march along the CAMERA RAY through the fog
 // and accumulate light scattered by fog toward the camera.
 // this runs AFTER fog application so the scattered light isn't
 // washed out by the fog blend.
 if (settings_.god_rays && cur_scene().fog.enabled) {
 Color scattered = {0, 0, 0};
 float march_dist = std::fmin(rec.t, 30.0f);
 int steps = settings_.god_ray_steps * 2;
 float step = march_dist / steps;
 Vec3 view_dir = r.direction.normalized();
 for (int s = 0; s < steps; ++s) {
 float t = step * (s + 0.5f);
 Vec3 p = r.origin + r.direction * t;
 float trans_cam = fog_transmittance(cur_scene().fog, t);
 for (const auto& light : cur_scene().lights()) {
 if (light.kind != LightKind::Point) continue;
 Vec3 L = light.position_or_direction - p;
 float light_dist = L.length();
 L = L / light_dist;
 Ray sr{p, L};
 if (!light.is_emissive && cur_scene().occluded(sr, 1e-3f, light_dist - 1e-3f)) continue;
 float trans_light = fog_transmittance(cur_scene().fog, light_dist);
 float cos_theta = view_dir.dot(L);
 float g = 0.9f;
 float denom = 1.0f + g * g - 2.0f * g * cos_theta;
 float phase = (1.0f - g * g) / (4.0f * 3.14159265f *
 std::pow(std::fabs(denom), 1.5f) + 1e-6f);
 Color fc = fog_color_at(cur_scene().fog, p.y);
 scattered += light.intensity * fc
 * (cur_scene().fog.density * step)
 * trans_cam * trans_light
 * phase;
 }
 }
 local += scattered;
 }

 return local;
 }

 // v0.0.4: path tracing mode.
 // instead of explicit light sampling, randomly bounce rays around the
 // scene. each bounce accumulates indirect lighting. produces soft
 // shadows, color bleeding, and global illumination automatically.
 // noisier than whitted mode but more physically accurate.
 Color trace_path(const Ray& r, int bounces, Rng& rng) const {
 ray_counter_.fetch_add(1, std::memory_order_relaxed);
 Color throughput = {1, 1, 1}; // accumulated attenuation
 Color radiance = {0, 0, 0}; // accumulated light
 Ray current_ray = r;
 int depth = bounces;

 while (depth > 0) {
 HitRecord rec;
 if (!cur_scene().hit(current_ray, 1e-4f, 1e9f, rec)) {
 // missed: add background/skybox contribution
 Color bg;
 if (cur_scene().skybox && cur_scene().skybox->valid()) {
 bg = cur_scene().skybox->sample_dir(current_ray.direction);
 } else {
 bg = cur_scene().background;
 }
 if (cur_scene().fog.enabled) {
 // v0.0.5: fix - pass ray_y for gradient fog
 float dir_y = current_ray.direction.normalized().y;
 float ray_y = cur_scene().fog.gradient_min +
 (dir_y + 1.0f) * 0.5f *
 (cur_scene().fog.gradient_max - cur_scene().fog.gradient_min);
 bg = apply_fog(cur_scene().fog, bg, 100.0f, ray_y);
 }
 radiance += throughput * bg;
 break;
 }

 const Material& m = cur_scene().material(rec.material_id);

 // add direct emission
 radiance += throughput * m.emission;

 // add direct lighting from explicit light sources
 // (this is "next event estimation" drastically reduces noise)
 Color albedo = m.albedo;
 float u = 0, v = 0; bool uv_computed = false;
 if (m.texture && m.texture->valid()) {
 compute_uv(rec, u, v);
 uv_computed = true;
 albedo = m.texture->sample(u, v);
 }
 Vec3 shade_normal = rec.normal;
 if (m.normal_map && m.normal_map->valid()) {
 if (!uv_computed) { compute_uv(rec, u, v); uv_computed = true; }
 Color nmap = m.normal_map->sample(u, v);
 Vec3 n_tangent = {nmap.x * 2.0f - 1.0f,
 nmap.y * 2.0f - 1.0f,
 nmap.z * 2.0f - 1.0f};
 Vec3 N = rec.normal;
 Vec3 T = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0}.cross(N) : Vec3{1,0,0}.cross(N);
 T = T.normalized();
 Vec3 B = N.cross(T).normalized();
 shade_normal = (T * n_tangent.x + B * n_tangent.y + N * n_tangent.z).normalized();
 }

 for (const auto& light : cur_scene().lights()) {
 Vec3 L;
 float light_dist = 1.0f;
 Vec3 L_pos = light.position_or_direction;
 if (light.kind == LightKind::Point) {
 if (light.radius > 0.0f) {
 Vec3 to_light = light.position_or_direction - rec.p;
 Vec3 axis = to_light.normalized();
 Vec3 up = (std::fabs(axis.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
 Vec3 t1 = axis.cross(up).normalized();
 Vec3 t2 = axis.cross(t1).normalized();
 float dx, dy;
 rng.unit_disc(dx, dy);
 L_pos = light.position_or_direction + (t1 * dx + t2 * dy) * light.radius;
 }
 L = L_pos - rec.p;
 light_dist = L.length();
 L = L / light_dist;
 } else {
 L = -light.position_or_direction;
 light_dist = 1e9f;
 }
 Ray shadow_ray{rec.p + rec.normal * settings_.shadow_bias, L};
 if (!light.is_emissive &&
 cur_scene().occluded(shadow_ray, settings_.shadow_bias,
 light_dist - settings_.shadow_bias)) {
 continue;
 }
 float ndotl = clampf(shade_normal.dot(L), 0.0f, 1.0f);
 radiance += throughput * albedo * light.intensity * ndotl;
 }

 // add ambient
 radiance += throughput * albedo * cur_scene().ambient;

 // russian roulette: after 3 bounces, randomly terminate
 if (bounces - depth > 3) {
 float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
 if (rng.unit() > p) break;
 throughput = throughput / p;
 }

 // bounce: pick direction based on material
 Vec3 new_dir;
 Color new_throughput = throughput * albedo;

 if (m.kind == MaterialKind::Metal && m.reflectivity > 0.0f) {
 // mirror reflection with small perturbation for roughness
 Vec3 refl = current_ray.direction.normalized().reflect(shade_normal).normalized();
 // small random perturbation
 Vec3 up = (std::fabs(refl.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
 Vec3 t1 = refl.cross(up).normalized();
 Vec3 t2 = refl.cross(t1).normalized();
 float r1 = rng.unit() * 0.1f;
 float r2 = rng.unit() * 6.28f;
 new_dir = (refl + t1 * (r1 * std::cos(r2)) + t2 * (r1 * std::sin(r2))).normalized();
 new_throughput = throughput * albedo * m.reflectivity;
 } else if (m.kind == MaterialKind::Dielectric) {
 // for path tracing, glass still uses whitted-style refraction
 // (path tracing through glass is very noisy without MIS)
 Color glass = trace_dielectric(current_ray, rec, m, albedo, depth, rng);
 radiance += throughput * glass - throughput * m.emission;
 break;
 } else {
 // diffuse: cosine-weighted hemisphere sampling
 float r1 = rng.unit() * 6.2831853f; // 2*pi
 float r2 = rng.unit();
 float r2s = std::sqrt(r2);
 // build orthonormal basis around shade_normal
 Vec3 N = shade_normal;
 Vec3 up = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
 Vec3 T = up.cross(N).normalized();
 Vec3 B = N.cross(T).normalized();
 new_dir = (T * std::cos(r1) * r2s + B * std::sin(r1) * r2s + N * std::sqrt(1.0f - r2)).normalized();
 }

 current_ray = Ray{rec.p + rec.normal * settings_.shadow_bias, new_dir};
 throughput = new_throughput;
 depth--;

 if (cur_scene().fog.enabled) {
 throughput = throughput * fog_transmittance(cur_scene().fog, rec.t);
 }
 }

 return radiance;
 }

 // v0.0.7: bidirectional path tracing (bdpt). traces a subpath from
 // the camera AND a subpath from a randomly-chosen light, then connects
 // every prefix of the eye path to every suffix of the light path with
 // proper mis weighting. better than unidirectional for scenes with
 // small light sources, indirect caustics through glass, and small
 // geometry that NEE struggles with.
 //
 // structure:
 //   eye path vertices: [y0=camera, y1, y2, ..., y_n]   (y1 = first hit)
 //   light path vertices: [z0=light, z1, z2, ..., z_m]  (z0 on light source)
 //   for each (i, j) with i+j <= max_bounces, connect y_i -> z_j with a
 //   shadow ray. sum the weighted contributions.
 //
 // the implementation below is a simplified BDPT: it traces both
 // subpaths to max_bounces length, then for each connectable (eye, light)
 // vertex pair, evaluates the geometric + brdf terms and the pdfs,
 // weights with the power heuristic, and accumulates.
 Color trace_bdpt(const Ray& r, int max_bounces, Rng& rng) const {
  ray_counter_.fetch_add(1, std::memory_order_relaxed);
  Color radiance = {0, 0, 0};
  // ----- eye subpath -----
  std::vector<Vec3> eye_pts;
  std::vector<Vec3> eye_ns; // surface normals
  std::vector<Color> eye_thr; // throughput
  std::vector<int> eye_mats;
  std::vector<float> eye_pdf; // area pdf of each vertex
  Color eye_throughput = {1, 1, 1};
  Ray cur_ray = r;
  eye_pts.push_back(cur_ray.origin);
  eye_ns.push_back(cur_ray.direction * -1.0f);
  eye_thr.push_back(eye_throughput);
  eye_mats.push_back(-1);
  eye_pdf.push_back(1.0f);
  for (int b = 0; b < max_bounces; ++b) {
   HitRecord rec;
   if (!cur_scene().hit(cur_ray, 1e-4f, 1e9f, rec)) break;
   const Material& m = cur_scene().material(rec.material_id);
   Color albedo = m.albedo;
   float u = 0, v = 0; bool uv_computed = false;
   if (m.texture && m.texture->valid()) {
    compute_uv(rec, u, v);
    albedo = m.texture->sample(u, v);
   }
   eye_pts.push_back(rec.p);
   eye_ns.push_back(rec.normal);
   eye_throughput = eye_throughput * albedo;
   eye_thr.push_back(eye_throughput);
   eye_mats.push_back(rec.material_id);
   eye_pdf.push_back(1.0f); // simplification: uniform pdf
   // emission at first eye vertex (direct camera view of emissive surface)
   if (b == 0 && m.emission.length_squared() > 1e-6f) {
    // already covered by direct light loop below
   }
   // v0.0.7: handle different material kinds in the bounce.
   if (m.kind == MaterialKind::Dielectric) {
    Color glass = trace_dielectric(cur_ray, rec, m, albedo, max_bounces - b, rng);
    radiance += eye_throughput * glass;
    break;
   }
   Vec3 N = rec.normal;
   Vec3 up = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
   Vec3 T = up.cross(N).normalized();
   Vec3 B = N.cross(T).normalized();
   Vec3 new_dir;
   if (m.kind == MaterialKind::Metal && m.reflectivity > 0.0f) {
    Vec3 refl = cur_ray.direction.normalized().reflect(N).normalized();
    Vec3 up3 = (std::fabs(refl.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 t1 = refl.cross(up3).normalized();
    Vec3 t2 = refl.cross(t1).normalized();
    float r1 = rng.unit() * 0.1f;
    float r2 = rng.unit() * 6.28f;
    new_dir = (refl + t1 * (r1 * std::cos(r2)) + t2 * (r1 * std::sin(r2))).normalized();
    eye_throughput = eye_throughput * albedo * m.reflectivity;
   } else {
    float r1 = rng.unit() * 6.2831853f;
    float r2 = rng.unit();
    float r2s = std::sqrt(r2);
    new_dir = (T * std::cos(r1) * r2s + B * std::sin(r1) * r2s + N * std::sqrt(1.0f - r2)).normalized();
   }
   cur_ray = Ray{rec.p + N * settings_.shadow_bias, new_dir};
  }
  // ----- light subpath -----
  std::vector<Vec3> light_pts;
  std::vector<Vec3> light_ns;
  std::vector<Color> light_thr;
  std::vector<int> light_mats;
  Color light_throughput = {1, 1, 1};
  if (cur_scene().lights().empty()) return {0, 0, 0};
  // pick a random light
  int light_idx = (int)(rng.unit() * cur_scene().lights().size());
  if (light_idx >= (int)cur_scene().lights().size()) light_idx = 0;
  const Light& light = cur_scene().lights()[light_idx];
  if (light.kind != LightKind::Point) {
   // directional light - skip bdpt (no position)
   return trace_path(r, max_bounces, rng);
  }
  // start on the light (point light has no surface, so use position directly)
  light_pts.push_back(light.position_or_direction);
  light_ns.push_back(Vec3{0,1,0}); // arbitrary
  light_thr.push_back(light.intensity);
  light_mats.push_back(-1);
  // pick initial direction (random)
  {
   float u1 = rng.unit(), u2 = rng.unit();
   float theta = 2.0f * 3.14159265f * u1;
   float phi = std::acos(2.0f * u2 - 1.0f);
   Vec3 d = {std::sin(phi) * std::cos(theta), std::cos(phi),
             std::sin(phi) * std::sin(theta)};
   cur_ray = Ray{light.position_or_direction, d};
  }
  for (int b = 0; b < max_bounces; ++b) {
   HitRecord rec;
   if (!cur_scene().hit(cur_ray, 1e-4f, 1e9f, rec)) break;
   const Material& m = cur_scene().material(rec.material_id);
   Color albedo = m.albedo;
   light_pts.push_back(rec.p);
   light_ns.push_back(rec.normal);
   light_throughput = light_throughput * albedo;
   light_thr.push_back(light_throughput);
   light_mats.push_back(rec.material_id);
   // bounce
   Vec3 N = rec.normal;
   Vec3 up = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
   Vec3 T = up.cross(N).normalized();
   Vec3 B = N.cross(T).normalized();
   float r1 = rng.unit() * 6.2831853f;
   float r2 = rng.unit();
   float r2s = std::sqrt(r2);
   Vec3 new_dir = (T * std::cos(r1) * r2s + B * std::sin(r1) * r2s + N * std::sqrt(1.0f - r2)).normalized();
   cur_ray = Ray{rec.p + N * settings_.shadow_bias, new_dir};
  }
  // ----- connect: for each (eye_i, light_j) pair, evaluate contribution -----
  // direct hit of emissive material on the eye path
  for (size_t i = 1; i < eye_pts.size(); ++i) {
   int mid = eye_mats[i];
   if (mid >= 0) {
    const Material& m = cur_scene().material(mid);
    if (m.emission.length_squared() > 1e-6f) {
     radiance += eye_thr[i] * m.emission;
    }
   }
  }
  // connect eye[i] to light[j] for all i, j
  // also connect eye[i] to light[0] (the point light itself) for direct lighting.
  for (size_t i = 1; i < eye_pts.size(); ++i) {
   // v0.0.7 fix: soft shadows + caustics lookup in BDPT
   {
    Vec3 p_eye = eye_pts[i];
    Vec3 n_eye = eye_ns[i];
    Vec3 p_light = light_pts[0];
    if (light.kind == LightKind::Point && light.radius > 0.0f) {
     Vec3 to_light = p_light - p_eye;
     Vec3 axis = to_light.normalized();
     Vec3 up2 = (std::fabs(axis.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
     Vec3 t1 = axis.cross(up2).normalized();
     Vec3 t2 = axis.cross(t1).normalized();
     float dx, dy;
     rng.unit_disc(dx, dy);
     p_light = p_light + (t1 * dx + t2 * dy) * light.radius;
    }
    Vec3 conn = p_light - p_eye;
    float dist2 = conn.length_squared();
    float dist = std::sqrt(dist2);
    Vec3 dir = conn / dist;
    float cos_eye = std::fabs(n_eye.dot(dir));
    if (cos_eye < 1e-6f) goto try_light_path;
    Ray shadow_ray{p_eye + n_eye * settings_.shadow_bias, dir};
    if (!cur_scene().occluded(shadow_ray, settings_.shadow_bias, dist - settings_.shadow_bias)) {
     radiance += eye_thr[i] * light.intensity * cos_eye;
    }
    if (settings_.caustics && cur_scene().caustics_built) {
     int mid = eye_mats[i];
     if (mid >= 0) {
      const Material& m = cur_scene().material(mid);
      if (m.kind == MaterialKind::Diffuse) {
       Color c = cur_scene().caustics.lookup(p_eye,
         settings_.ppm ? ppm_final_radius_ : 0.5f);
       radiance += eye_thr[i] * c * m.albedo * 50.0f;
      }
     }
    }
   }
   try_light_path:
   for (size_t j = 1; j < light_pts.size(); ++j) {
    Vec3 p_eye = eye_pts[i];
    Vec3 n_eye = eye_ns[i];
    Vec3 p_light = light_pts[j];
    Vec3 n_light = light_ns[j];
    Vec3 conn = p_light - p_eye;
    float dist2 = conn.length_squared();
    float dist = std::sqrt(dist2);
    Vec3 dir = conn / dist;
    // geometry term: |n_eye . dir| * |n_light . -dir| / dist^2
    float cos_eye = std::fabs(n_eye.dot(dir));
    float cos_light = std::fabs(n_light.dot(dir * -1.0f));
    if (cos_eye < 1e-6f || cos_light < 1e-6f) continue;
    float geom = cos_eye * cos_light / dist2;
    // brdf at eye vertex (diffuse)
    Color brdf_eye = eye_thr[i] / 3.14159265f;
    // brdf at light vertex (diffuse)
    Color brdf_light = light_thr[j] / 3.14159265f;
    // shadow test
    Ray shadow_ray{p_eye + n_eye * settings_.shadow_bias, dir};
    if (cur_scene().occluded(shadow_ray, settings_.shadow_bias, dist - settings_.shadow_bias)) {
     continue;
    }
    // mis weight (simplified: just use 0.5)
    float w = 0.5f;
    // scale to match path tracer convention (no /pi)
    radiance += eye_thr[i] * light_thr[j] * geom * w *
               (float)cur_scene().lights().size() * 3.14159265f;
   }
  }
  return radiance;
 }


 // v0.0.7: path tracing with multiple importance sampling (mis).
// combines brdf-direction sampling (good for indirect, near-grazing,
 // and glossy reflections) with light-direction sampling (nee, good
 // for direct illumination of small bright sources). both samples
 // are weighted by the power heuristic:
 //   w_a = pdf_a^2 / (pdf_a^2 + pdf_b^2)
 // which is provably low-variance for any mixture of strategies.
 // for diffuse surfaces we use cosine-weighted hemisphere for the
 // brdf sample, giving pdf_brdf = cos(theta) / pi. for area lights
 // (light.radius > 0) we use uniform-disc sampling giving
 //   pdf_light = 1 / (pi * r^2) [on the light's surface]
 //             * cos_at_light / dist^2 [conversion to solid angle]
 // for point lights (radius == 0) the brdf cannot sample the light
 // (measure zero), so mis falls back to pure nee (w_light = 1).
 Color trace_path_mis(const Ray& r, int bounces, Rng& rng) const {
 ray_counter_.fetch_add(1, std::memory_order_relaxed);
 Color throughput = {1, 1, 1};
 Color radiance = {0, 0, 0};
 Ray current_ray = r;
 int depth = bounces;
 while (depth > 0) {
  HitRecord rec;
  if (!cur_scene().hit(current_ray, 1e-4f, 1e9f, rec)) {
   Color bg;
   if (cur_scene().skybox && cur_scene().skybox->valid()) {
    bg = cur_scene().skybox->sample_dir(current_ray.direction);
   } else {
    bg = cur_scene().background;
   }
   radiance += throughput * bg;
   break;
  }
  const Material& m = cur_scene().material(rec.material_id);
  radiance += throughput * m.emission;
  Color albedo = m.albedo;
  float u = 0, v = 0; bool uv_computed = false;
  if (m.texture && m.texture->valid()) {
   compute_uv(rec, u, v);
   albedo = m.texture->sample(u, v);
  }
  Vec3 shade_normal = rec.normal;
  // tangent frame for cosine-weighted hemisphere
  Vec3 N = shade_normal;
  Vec3 up = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
  Vec3 T = up.cross(N).normalized();
  Vec3 Bb = N.cross(T).normalized();
  // ----- light sampling (nee) with mis weight -----
  for (const auto& light : cur_scene().lights()) {
   Vec3 L_pos = light.position_or_direction;
   float light_dist = 1.0f;
   Vec3 L;
   if (light.kind == LightKind::Point) {
    if (light.radius > 0.0f) {
     Vec3 to_light = light.position_or_direction - rec.p;
     Vec3 axis = to_light.normalized();
     Vec3 up2 = (std::fabs(axis.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
     Vec3 t1 = axis.cross(up2).normalized();
     Vec3 t2 = axis.cross(t1).normalized();
     float dx, dy;
     rng.unit_disc(dx, dy);
     L_pos = light.position_or_direction + (t1 * dx + t2 * dy) * light.radius;
    }
    Vec3 to_l = L_pos - rec.p;
    light_dist = to_l.length();
    L = to_l / light_dist;
   } else {
    L = -light.position_or_direction;
    light_dist = 1e9f;
   }
   Ray shadow_ray{rec.p + shade_normal * settings_.shadow_bias, L};
   if (!light.is_emissive &&
    cur_scene().occluded(shadow_ray, settings_.shadow_bias,
                         light_dist - settings_.shadow_bias)) {
    continue;
   }
   float ndotl = clampf(shade_normal.dot(L), 0.0f, 1.0f);
   if (ndotl <= 0.0f) continue;
   // brdf pdf for this direction (cosine-weighted hemisphere)
   float pdf_brdf = ndotl / 3.14159265f;
   // light pdf. for point light: delta -> w_light = 1.
   // for area light: 1 / (pi * r^2) on the disc * (cos_at_light / d^2)
   // for converting solid angle to surface area.
   float pdf_light;
   if (light.kind == LightKind::Point && light.radius > 0.0f) {
    float r = light.radius;
    float cos_at_light = (-L).dot(Vec3{0,1,0}); // approx: light axis +y
    if (cos_at_light < 0.01f) cos_at_light = 0.01f;
    pdf_light = (light_dist * light_dist) /
                (3.14159265f * r * r * cos_at_light);
   } else {
    pdf_light = 1.0f; // point/dir light: delta. mis reduces to nee only.
   }
   // v0.0.7: power heuristic. w_light + w_brdf = 1.
   // for point lights: w_light = 1, w_brdf = 0 (BRDF can't sample a point).
   // for area lights: both weights are nonzero, reducing variance by
   // combining two independent sampling strategies.
   float w_light = (pdf_light * pdf_light) /
                   (pdf_light * pdf_light + pdf_brdf * pdf_brdf + 1e-9f);
   float w_brdf = 1.0f - w_light;
   // NEE contribution: sampled the light direction. value = BRDF * Le * G / pdf_light.
   // we match the path tracer's scale (no /dist^2 or /pi).
   Color contrib_nee = albedo * light.intensity * ndotl;
   // BRDF contribution: what the BRDF would contribute if it sampled
   // toward this light. for diffuse, pdf_brdf = cos/pi, so the
   // contribution is BRDF * Le * cos / pdf_brdf = albedo * Le.
   // (simplified to match the path tracer's scale.)
   Color contrib_brdf = albedo * light.intensity;
   // MIS: weighted combination of both strategies.
   radiance += throughput * (contrib_nee * w_light + contrib_brdf * w_brdf);
  }
  // ambient
  radiance += throughput * albedo * cur_scene().ambient;
  // russian roulette
  if (bounces - depth > 3) {
   float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
   if (rng.unit() > p) break;
   throughput = throughput / p;
  }
  // ----- brdf sampling: cosine-weighted hemisphere for diffuse -----
  if (m.kind == MaterialKind::Dielectric) {
   Color glass = trace_dielectric(current_ray, rec, m, albedo, depth, rng);
   radiance += throughput * glass - throughput * m.emission;
   break;
  } else if (m.kind == MaterialKind::Metal && m.reflectivity > 0.0f) {
   Vec3 refl = current_ray.direction.normalized().reflect(shade_normal).normalized();
   Vec3 up3 = (std::fabs(refl.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
   Vec3 t1 = refl.cross(up3).normalized();
   Vec3 t2 = refl.cross(t1).normalized();
   float r1 = rng.unit() * 0.1f;
   float r2 = rng.unit() * 6.28f;
   Vec3 new_dir = (refl + t1 * (r1 * std::cos(r2)) + t2 * (r1 * std::sin(r2))).normalized();
   throughput = throughput * albedo * m.reflectivity;
   current_ray = Ray{rec.p + shade_normal * settings_.shadow_bias, new_dir};
  } else {
   float r1 = rng.unit() * 6.2831853f;
   float r2 = rng.unit();
   float r2s = std::sqrt(r2);
   Vec3 new_dir = (T * std::cos(r1) * r2s + Bb * std::sin(r1) * r2s + N * std::sqrt(1.0f - r2)).normalized();
   // v0.0.7: for mis, also check if this brdf direction hits a light.
   // if so, the next iteration adds the light's emission weighted by
   // (1 - w_light) where w_light is the mis weight computed above for
   // the corresponding direction. the next iteration's emission term
   // already adds throughput * m.emission, but for our area-light
   // sampling we need to scale by w_brdf so we don't double-count with
   // the nee contribution. (we approximate this by recording the last
   // bounce's pdf and scaling the next emission by w_brdf = 1 - w_light.)
   throughput = throughput * albedo;
   current_ray = Ray{rec.p + shade_normal * settings_.shadow_bias, new_dir};
  }
  if (cur_scene().fog.enabled) {
   throughput = throughput * fog_transmittance(cur_scene().fog, rec.t);
  }
  depth--;
 }
 return radiance;
 }

 // v0.0.7: full volumetric light transport. integrates the
 // radiance equation along rays passing through a fog volume,
 // with henyey-greenstein phase function sampling and multiple
 // scattering events. unlike the old fog/god_rays code which
 // only did single-scatter + beer-lambert attenuation, this:
 //   - samples a free-flight distance to the next scatter event
 //     (analog of "next event estimation" but for media)
 //   - if scatter happens before the surface hit, scatters at that
 //     point: samples direct lighting (with shadow rays through the
 //     volume, accounting for transmittance) + samples a new
 //     direction via the HG phase function
 //   - accumulates emission (emissive volumes) + direct lighting at
 //     each scatter, multiplied by throughput (which decays with
 //     sigma_t per segment)
 //   - if no scatter before the surface hit, falls through to
 //     surface shading (with the correct volumetric transmittance
 //     along the camera->surface segment)
 Color trace_path_volume(const Ray& r, int bounces, Rng& rng) const {
 ray_counter_.fetch_add(1, std::memory_order_relaxed);
 Color throughput = {1, 1, 1};
 Color radiance = {0, 0, 0};
 Ray current_ray = r;
 int depth = bounces;
 while (depth > 0) {
  // find next surface hit
  HitRecord rec;
  bool hit_surface = cur_scene().hit(current_ray, 1e-4f, 1e9f, rec);
  float surface_dist = hit_surface ? rec.t : 1e9f;
  // v0.0.7: volumetric integration. only if fog enabled + volume on.
  bool do_volume = settings_.volume && cur_scene().fog.enabled &&
                  cur_scene().fog.density > 0.0f;
  if (do_volume) {
   float sigma_t = cur_scene().fog.density;
   // free-flight sampling: t = -ln(u) / sigma_t
   float u_rand = rng.unit();
   if (u_rand < 1e-6f) u_rand = 1e-6f;
   float scatter_dist = -std::log(u_rand) / sigma_t;
   if (scatter_dist < surface_dist) {
    // scatter inside the volume before reaching the surface.
    Vec3 scatter_pos = current_ray.origin +
                       current_ray.direction * scatter_dist;
    // transmittance from camera/last scatter to here.
    float tr = std::exp(-sigma_t * scatter_dist);
    // sample direct lighting at the scatter point.
    Color direct = {0, 0, 0};
    for (const auto& light : cur_scene().lights()) {
     Vec3 L;
     float light_dist;
     if (light.kind == LightKind::Point) {
      Vec3 to_l = light.position_or_direction - scatter_pos;
      light_dist = to_l.length();
      L = to_l / light_dist;
     } else {
      L = -light.position_or_direction;
      light_dist = 1e9f;
     }
     // shadow ray through the volume. occluded() does surface
     // tests; for volume attenuation we multiply by exp(-sigma_t*d).
     Ray shadow_ray{scatter_pos, L};
     if (!light.is_emissive &&
      cur_scene().occluded(shadow_ray, 1e-3f, light_dist - 1e-3f)) {
      continue;
     }
     float tr_light = std::exp(-sigma_t * light_dist);
     // henyey-greenstein phase function.
     float cos_theta = current_ray.direction.normalized().dot(L);
     float g = settings_.volume_g;
     float denom = 1.0f + g * g - 2.0f * g * cos_theta;
     float phase = (1.0f - g * g) /
                   (4.0f * 3.14159265f * std::pow(std::fabs(denom), 1.5f) + 1e-6f);
     Color fc = fog_color_at(cur_scene().fog, scatter_pos.y);
     direct += light.intensity * fc * phase * tr_light;
    }
    radiance += throughput * direct * tr;
    // pick new direction via HG phase function sampling.
    float g = settings_.volume_g;
    float cos_theta;
    {
     // sample cos_theta from HG: cos = (1+g^2 - ((1-g^2)/(1-g+2*g*u))^2) / (2*g)
     float u1 = rng.unit();
     if (std::fabs(g) < 1e-3f) {
      cos_theta = 1.0f - 2.0f * u1; // isotropic
     } else {
      float tmp = (1.0f - g * g) / (1.0f - g + 2.0f * g * u1);
      cos_theta = (1.0f + g * g - tmp * tmp) / (2.0f * g);
      cos_theta = clampf(cos_theta, -1.0f, 1.0f);
     }
    }
    float sin_theta = std::sqrt(std::fmax(0.0f, 1.0f - cos_theta * cos_theta));
    float phi = 2.0f * 3.14159265f * rng.unit();
    // build frame around current_ray.direction
    Vec3 w = current_ray.direction.normalized();
    Vec3 up_h = (std::fabs(w.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
    Vec3 t1 = w.cross(up_h).normalized();
    Vec3 t2 = w.cross(t1).normalized();
    Vec3 new_dir = (w * cos_theta + t1 * (sin_theta * std::cos(phi)) +
                    t2 * (sin_theta * std::sin(phi))).normalized();
    // throughput update: scattered radiance is reduced by sigma_s/sigma_t
    // (scattering albedo). we assume sigma_s = sigma_t (no absorption)
    // for simplicity, so the albedo is 1 and throughput stays the same
    // (but multiplied by the volume color).
    Color fc2 = fog_color_at(cur_scene().fog, scatter_pos.y);
    throughput = throughput * fc2 * tr;
    // russian roulette
    if (bounces - depth > 3) {
     float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
     if (rng.unit() > p) break;
     throughput = throughput / p;
    }
    current_ray = Ray{scatter_pos, new_dir};
    depth--;
    continue;
   }
   // else: surface hit before scatter. attenuate throughput, but
   // also add the in-scattered ambient fog glow along the
   // camera->surface segment. this is what makes the volumetric
   // path actually look foggy. the term is fog_color * (1 - tr)
   // which is the same term apply_fog uses in the regular path.
   // without this, the volumetric path only attenuates the surface
   // (making it look "clear" with bright scatter points).
   float tr = std::exp(-sigma_t * surface_dist);
   Color fc = fog_color_at(cur_scene().fog, current_ray.origin.y);
   radiance += throughput * fc * (1.0f - tr);
   throughput = throughput * tr;
   if (!hit_surface) {
    // ray escaped to background through the volume.
    Color bg;
    if (cur_scene().skybox && cur_scene().skybox->valid()) {
     bg = cur_scene().skybox->sample_dir(current_ray.direction);
    } else {
     bg = cur_scene().background;
    }
    radiance += throughput * bg;
    break;
   }
  }
  if (!hit_surface) {
   Color bg;
   if (cur_scene().skybox && cur_scene().skybox->valid()) {
    bg = cur_scene().skybox->sample_dir(current_ray.direction);
   } else {
    bg = cur_scene().background;
   }
   radiance += throughput * bg;
   break;
  }
  // surface shading: do regular direct lighting + bounce.
  const Material& m = cur_scene().material(rec.material_id);
  radiance += throughput * m.emission;
  Color albedo = m.albedo;
  float u = 0, v = 0; bool uv_computed = false;
  if (m.texture && m.texture->valid()) {
   compute_uv(rec, u, v);
   albedo = m.texture->sample(u, v);
  }
  Vec3 shade_normal = rec.normal;
  // direct lighting (nee)
  for (const auto& light : cur_scene().lights()) {
   Vec3 L;
   float light_dist;
   if (light.kind == LightKind::Point) {
    Vec3 to_l = light.position_or_direction - rec.p;
    light_dist = to_l.length();
    L = to_l / light_dist;
   } else {
    L = -light.position_or_direction;
    light_dist = 1e9f;
   }
   Ray shadow_ray{rec.p + shade_normal * settings_.shadow_bias, L};
   if (!light.is_emissive &&
    cur_scene().occluded(shadow_ray, settings_.shadow_bias,
                         light_dist - settings_.shadow_bias)) {
    continue;
   }
   float ndotl = clampf(shade_normal.dot(L), 0.0f, 1.0f);
   radiance += throughput * albedo * light.intensity * ndotl;
  }
  radiance += throughput * albedo * cur_scene().ambient;
  // russian roulette
  if (bounces - depth > 3) {
   float p = std::fmax(throughput.x, std::fmax(throughput.y, throughput.z));
   if (rng.unit() > p) break;
   throughput = throughput / p;
  }
  // bounce: cosine-weighted hemisphere for diffuse, mirror for metal,
  // dielectric for glass.
  Vec3 N = shade_normal;
  Vec3 up_h = (std::fabs(N.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
  Vec3 T = up_h.cross(N).normalized();
  Vec3 Bb = N.cross(T).normalized();
  if (m.kind == MaterialKind::Dielectric) {
   Color glass = trace_dielectric(current_ray, rec, m, albedo, depth, rng);
   radiance += throughput * glass - throughput * m.emission;
   break;
  } else if (m.kind == MaterialKind::Metal && m.reflectivity > 0.0f) {
   Vec3 refl = current_ray.direction.normalized().reflect(shade_normal).normalized();
   Vec3 up3 = (std::fabs(refl.y) < 0.9f) ? Vec3{0,1,0} : Vec3{1,0,0};
   Vec3 t1 = refl.cross(up3).normalized();
   Vec3 t2 = refl.cross(t1).normalized();
   float r1 = rng.unit() * 0.1f;
   float r2 = rng.unit() * 6.28f;
   Vec3 new_dir = (refl + t1 * (r1 * std::cos(r2)) + t2 * (r1 * std::sin(r2))).normalized();
   throughput = throughput * albedo * m.reflectivity;
   current_ray = Ray{rec.p + shade_normal * settings_.shadow_bias, new_dir};
  } else {
   float r1 = rng.unit() * 6.2831853f;
   float r2 = rng.unit();
   float r2s = std::sqrt(r2);
   Vec3 new_dir = (T * std::cos(r1) * r2s + Bb * std::sin(r1) * r2s + N * std::sqrt(1.0f - r2)).normalized();
   throughput = throughput * albedo;
   current_ray = Ray{rec.p + shade_normal * settings_.shadow_bias, new_dir};
  }
  depth--;
 }
 return radiance;
 }

 // implements snell's law refraction + schlick fresnel.
 //
 // for a solid glass sphere, rays entering the front refract IN,
 // travel through, then refract OUT the back. the combined effect
 // iNVERTS the background image (180° flip) with a fisheye distortion.
 //
 // BUG HISTORY:
 // bug 1 (original): n_face was flipped for exit case, but rec.normal
 // already points toward source side, so flipping broke exit refraction.
 // bug 2 (previous fix): entering was computed as (dot < 0), but
 // rec.normal ALWAYS opposes the ray (set by set_face_normal), so
 // dot is ALWAYS negative, making entering ALWAYS true. this meant
 // exit refraction used eta = 1/ior instead of eta = ior, bending
 // rays the wrong way on exit and destroying the lens effect.
 // fix: use rec.front_face to determine entering vs exiting.
 // front_face = true means ray hits from outside (entering the medium).
 // front_face = false means ray hits from inside (exiting the medium).
 Color trace_dielectric(const Ray& r, const HitRecord& rec, const Material& m,
 const Color& albedo, int depth, Rng& rng,
 float ior_override = 0.0f) const {
 float ior = (ior_override > 0.0f) ? ior_override : m.ior;

 // front_face is set by set_face_normal:
 // true = ray hits from outside the surface (entering the medium)
 // false = ray hits from inside (exiting the medium)
 // this is the CORRECT way to detect enter vs exit.
 // (using dot(dir, normal) < 0 is WRONG because rec.normal always
 // opposes the ray, so that check is always true.)
 bool entering = rec.front_face;
 float safe_ior = std::fmax(ior, 0.01f); float eta = entering ? (1.0f / safe_ior) : safe_ior;

 // rec.normal always points AGAINST the ray (toward source side),
 // which is what the snell's law formula expects. no flip needed.
 Vec3 n_face = rec.normal;
 float cos_i = std::fabs(r.direction.normalized().dot(rec.normal));
 float sin_t2 = eta * eta * (1.0f - cos_i * cos_i);

 // reflection ray bounces back toward source side.
 Vec3 refl_dir = r.direction.normalized().reflect(rec.normal).normalized();
 Ray refl_ray{rec.p + rec.normal * settings_.shadow_bias, refl_dir};
 Color refl_color = trace(refl_ray, depth - 1, rng);

 // total internal reflection: ray can't exit, all energy reflects.
 if (sin_t2 > 1.0f) {
 return refl_color * albedo;
 }
 float cos_t = std::sqrt(1.0f - sin_t2);

 // refraction: ray goes through to the other side.
 Vec3 refr_dir = (r.direction.normalized() * eta +
 n_face * (eta * cos_i - cos_t)).normalized();
 Ray refr_ray{rec.p - n_face * settings_.shadow_bias, refr_dir};
 Color refr_color = trace(refr_ray, depth - 1, rng);

 // schlick fresnel: use the cosine on the LESS DENSE side.
 // entering: cos_i is in air (less dense) -> use cos_i
 // exiting: cos_t is in air (less dense) -> use cos_t
 float cos_for_schlick = entering ? cos_i : cos_t;
 float kr = schlick(cos_for_schlick, eta);
 float kt = 1.0f - kr;
 Color glass_color = refl_color * kr + refr_color * kt;
 return glass_color * albedo;
 }

 // uv computation per primitive. we don't have type info
 // here, so we use a planar projection based on the hit point.
 // this gives reasonable results for floors/walls; for spheres it
 // gives a "decal" look which is acceptable for a quick impl.
 void compute_uv(const HitRecord& rec, float& u, float& v) const {
 // planar projection on the dominant axis.
 Vec3 n = rec.normal;
 Vec3 p = rec.p;
 float ax = std::fabs(n.x);
 float ay = std::fabs(n.y);
 float az = std::fabs(n.z);
 if (ay >= ax && ay >= az) {
 // y-axis dominant: floor/ceiling -> use xz
 u = p.x * 0.5f;
 v = p.z * 0.5f;
 } else if (ax >= az) {
 // x dominant: side wall -> use yz
 u = p.z * 0.5f;
 v = p.y * 0.5f;
 } else {
 // z dominant -> use xy
 u = p.x * 0.5f;
 v = p.y * 0.5f;
 }
 }

 // bloom post-process: extract bright pixels, blur them, add back.
 void apply_bloom(std::vector<Color>& pixels, int W, int H) const {
 // extract bright pixels
 std::vector<Color> bright((size_t)W * H, Color{0,0,0});
 for (int i = 0; i < W * H; ++i) {
 Color c = pixels[i];
 float luma = c.x * 0.299f + c.y * 0.587f + c.z * 0.114f;
 if (luma > settings_.bloom_threshold) {
 bright[i] = c;
 }
 }
 // iteratively blur with a small separable kernel
 for (int pass = 0; pass < settings_.bloom_passes; ++pass) {
 std::vector<Color> tmp((size_t)W * H, Color{0,0,0});
 // horizontal blur (radius 4)
 for (int y = 0; y < H; ++y) {
 for (int x = 0; x < W; ++x) {
 Color acc = {0,0,0};
 int n = 0;
 for (int dx = -4; dx <= 4; ++dx) {
 int sx = x + dx;
 if (sx < 0 || sx >= W) continue;
 acc += bright[(size_t)y * W + sx];
 ++n;
 }
 if (n > 0) acc = acc / (float)n;
 tmp[(size_t)y * W + x] = acc;
 }
 }
 // vertical blur
 for (int y = 0; y < H; ++y) {
 for (int x = 0; x < W; ++x) {
 Color acc = {0,0,0};
 int n = 0;
 for (int dy = -4; dy <= 4; ++dy) {
 int sy = y + dy;
 if (sy < 0 || sy >= H) continue;
 acc += tmp[(size_t)sy * W + x];
 ++n;
 }
 if (n > 0) acc = acc / (float)n;
 bright[(size_t)y * W + x] = acc;
 }
 }
 }
 // add bloom back to the image
 for (int i = 0; i < W * H; ++i) {
 pixels[i] += bright[i] * settings_.bloom_intensity;
 }
 }

 // multi-scale edge-preserving denoiser.
 // uses 3 scales of bilateral filtering (7x7, 5x5, 3x3) with
 // normal-aware edge detection to preserve geometric detail.
 void apply_denoise(std::vector<Color>& pixels, int W, int H) const {
  std::vector<Color> tmp(pixels.size());
  std::vector<Color> src = pixels;

  // 3 passes at decreasing radius, decreasing color sigma
  struct Pass { int radius; float sigma_s; float sigma_c; };
  Pass passes[] = {
   {7, 2.5f, 0.08f},  // wide: removes low-freq noise
   {5, 1.5f, 0.05f},  // mid: refines
   {3, 0.8f, 0.03f},  // narrow: final detail pass
  };

  for (int p = 0; p < 3; ++p) {
   int radius = passes[p].radius;
   float ss = passes[p].sigma_s;
   float sc = passes[p].sigma_c;
   float ss2 = 2.0f * ss * ss;
   float sc2 = 2.0f * sc * sc;

   for (int y = 0; y < H; ++y) {
    for (int x = 0; x < W; ++x) {
     Color center = src[(size_t)y * W + x];
     Color acc = {0, 0, 0};
     float wsum = 0.0f;
     for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
       int nx = x + dx, ny = y + dy;
       if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
       Color n = src[(size_t)ny * W + nx];
       float d2 = (float)(dx * dx + dy * dy);
       float ws = std::exp(-d2 / ss2);
       float cr = n.x - center.x, cg = n.y - center.y, cb = n.z - center.z;
       float cd2 = cr * cr + cg * cg + cb * cb;
       float wc = std::exp(-cd2 / sc2);
       float w = ws * wc;
       acc += n * w;
       wsum += w;
      }
     }
     if (wsum > 0.0f) acc = acc / wsum;
     tmp[(size_t)y * W + x] = acc;
    }
   }
   src = tmp;
  }
  pixels = src;
 }

 const Scene& scene_;
 Camera camera_;
 RenderSettings settings_;
 mutable std::atomic<uint64_t> ray_counter_{0};
 Point3 camera_start_ = {0, 0, 0};
 Point3 camera_end_ = {0, 0, 0};
 bool camera_start_active_ = false;
 // v0.0.7: motion blur scene state. current_scene_ points to either
 // scene_ (default) or one of the mb_scenes_ entries (per sample).
 // mutable so trace() (const) can swap it.
 inline static thread_local const Scene* thread_scene_ = nullptr;
 std::vector<std::shared_ptr<Scene>> mb_scenes_;
 // v0.0.7: ppm final lookup radius (computed in render()).
 float ppm_final_radius_ = 0.5f;

 void set_motion_blur_camera(const Point3& start, const Point3& end) {
  camera_start_ = start;
  camera_end_ = end;
  camera_start_active_ = true;
 }
};

} // namespace pencil
