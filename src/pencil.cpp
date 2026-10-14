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

#if defined(__linux__) || defined(_WIN32)
#include "distributed.hpp"
#endif
// pencil.cpp - entry point for the pencil ray tracer, v0.0.3 beta

#include "camera.hpp"
#include "image_io.hpp"
#include "pecl_parser.hpp"
#include "renderer.hpp"
#include "scene.hpp"
#include "vec3.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "portable.hpp"
#include "zipwriter.hpp"

namespace pencil {

static const char* PENCIL_VERSION = "v0.0.7 beta";

// built-in demo scene: 3 spheres + plane + 2 lights
static ParsedScene build_demo_scene() {
    ParsedScene s;
    s.eye = {0.0f, 1.5f, -5.0f};
    s.look = {0.0f, 0.0f, 0.0f};
    s.up = {0.0f, 1.0f, 0.0f};
    s.fov = 60.0f;
    s.aspect = 4.0f / 3.0f;
    s.width = 480;
    s.height = 360;
    s.samples = 4;
    s.depth = 8;

    s.scene.background = {0.05f, 0.06f, 0.09f};
    s.scene.ambient    = {0.08f, 0.08f, 0.10f};

    // floor
    s.scene.add_material(make_diffuse({0.5f, 0.5f, 0.55f}, 0.2f, 32.0f));
    int mat_floor = s.scene.material_count() - 1;
    auto tris = make_quad(
        Point3{-3.0f, -1.0f, -3.0f},
        Point3{ 3.0f, -1.0f, -3.0f},
        Point3{ 3.0f, -1.0f,  3.0f},
        Point3{-3.0f, -1.0f,  3.0f},
        mat_floor);
    for (auto& t : tris) s.scene.add_primitive(std::move(t));

    // red diffuse sphere
    s.scene.add_material(make_diffuse({0.7f, 0.15f, 0.15f}, 0.3f, 64.0f));
    int mat_red = s.scene.material_count() - 1;
    s.scene.add_primitive(std::make_shared<Sphere>(Point3{-1.1f, 0.0f, 0.0f}, 1.0f, mat_red));

    // glass sphere
    s.scene.add_material(make_dielectric(1.5f, {0.95f, 0.97f, 1.0f}));
    int mat_glass = s.scene.material_count() - 1;
    s.scene.add_primitive(std::make_shared<Sphere>(Point3{1.1f, 0.0f, 0.0f}, 1.0f, mat_glass));

    // mirror sphere on top
    s.scene.add_material(make_metal({0.95f, 0.95f, 0.95f}, 0.02f));
    int mat_metal = s.scene.material_count() - 1;
    s.scene.add_primitive(std::make_shared<Sphere>(Point3{0.0f, 1.5f, 0.0f}, 0.5f, mat_metal));

    // lights
    s.scene.add_light(make_point_light({4.0f, 6.0f, -4.0f}, {1.0f, 1.0f, 0.95f}));
    s.scene.add_light(make_point_light({-4.0f, 3.0f, -3.0f}, {0.3f, 0.3f, 0.45f}));

    s.finalize();
    return s;
}

} // namespace pencil

// format an integer with thousands separators
static std::string with_commas(uint64_t n) {
    std::string s = std::to_string(n);
    std::string out;
    int len = (int)s.size();
    for (int i = 0; i < len; ++i) {
        if (i > 0 && (len - i) % 3 == 0) out.push_back(',');
        out.push_back(s[i]);
    }
    return out;
}

static void print_help() {
    std::cout <<
        "pencil " << pencil::PENCIL_VERSION << " - cpu ray tracer\n"
        "\n"
        "usage:\n"
        "  pencil <scene.pecl> [output.{ppm,png}] [options]\n"
        "  pencil --demo [output.{ppm,png}] [options]\n"
        "  pencil --benchmark <scene.pecl> [options]\n"
        "  pencil <scene.pecl> output.{mp4,gif,zip} --animate [options]\n"
        "  pencil --help\n"
        "\n"
        "output format auto-detected from extension (.ppm, .png, .mp4, .gif, .zip).\n"
        "\n"
        "options:\n"
        "  --width N      override output width.\n"
        "  --height N     override output height.\n"
        "  --samples N    override AA samples per pixel (default 4, so 4x4=16 spp).\n"
        "  --depth N      override max reflection/refraction depth (default 8).\n"
        "  --threads N    override worker thread count (default: auto).\n"
        "  --no-bvh       disable BVH, use brute-force linear intersection.\n"
        "  --benchmark    run scene twice (BVH + no-BVH), print stats, no output.\n"
        "  --animate      render all frames as an animation (mp4/gif/zip output).\n"
        "  --frames N     override total animation frame count.\n"
        "  --fps N        override animation FPS (default 30).\n"
        "  --normals      output normal-map (AOV) instead of shaded image.\n"
        "  --bloom        apply bloom post-process (glow on bright pixels).\n"
        "  --denoise     apply bilateral denoise (edge-preserving smoothing).\n"
        "  --pathtrace   use path tracing mode (soft GI, color bleeding).\n"
        "  --progress      show a progress bar with ETA on stderr.\n"
        "  --server       run as render server (distributes tiles to workers).\n"
        "  --worker IP    run as render worker, connect to server at IP.\n"
        "  --port N       TCP port for distributed rendering (default 8080).\n"
        "  --quiet        don't print progress.\n"
        "  --demo         render built-in demo scene.\n"
        "  --version, -v  print version and exit.\n"
        "  --help, -h     show this help.\n";
}

struct CliArgs {
    std::string scene_path;
    std::string out_path;
    bool quiet      = false;
    bool demo       = false;
    bool benchmark  = false;
    bool animate    = false;
    bool no_bvh     = false;
    bool normals    = false;
    bool bloom      = false;
    bool progress   = false;
    bool denoise    = false;
    bool pathtrace  = false;
    bool bdpt       = false; // v0.0.7: bidirectional path tracing
    bool spectral   = false; // v0.0.7: spectral rendering (cauchy ior)
    bool mis        = false; // v0.0.7: enable mis in path tracer
    bool volume     = false; // v0.0.7: full volumetric light transport
    float exposure  = 0.0f;    // v0.0.5: exposure in stops
    int   tonemap   = 1;        // v0.0.5: 0=none, 1=ACES (default), 2=Reinhard
    bool  god_rays  = false;    // v0.0.5: volumetric god rays
    bool  motion_blur = false;  // v0.0.5: motion blur
    bool  adaptive  = false;    // v0.0.5: adaptive sampling
    int   bokeh_blades = 0;     // v0.0.5: bokeh aperture shape
    bool  caustics  = false;    // v0.0.5: caustics (photon mapping)
    bool  ppm       = false;   // v0.0.7: progressive photon mapping
    // v0.0.6: distributed rendering
    bool  is_server = false;
    bool  is_worker = false;
    std::string server_ip;
    int   port = 8080;
    int  override_w = 0;
    int  override_h = 0;
    int  override_s = 0;
    int  override_d = 0;
    int  override_t = 0;
    int  override_frames = 0;
    int  override_fps    = 0;
};

static CliArgs parse_args(int argc, char** argv) {
    CliArgs a;
    // first pass: detect --demo, --benchmark, --animate so we know whether the
    // first positional arg is a scene path or an output path.
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--demo")      a.demo = true;
        else if (s == "--benchmark") a.benchmark = true;
        else if (s == "--animate")  a.animate = true;
    }
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        if (s == "--help" || s == "-h")            { print_help(); std::exit(0); }
        else if (s == "--version" || s == "-v")    { std::cout << pencil::PENCIL_VERSION << "\n"; std::exit(0); }
        else if (s == "--quiet")                   { a.quiet = true; }
        else if (s == "--demo")                     { /* already handled */ }
        else if (s == "--benchmark")                { /* already handled */ }
        else if (s == "--animate")                  { /* already handled */ }
        else if (s == "--no-bvh" || s == "--nobvh"){ a.no_bvh = true; }
        else if (s == "--normals" || s == "--normal" || s == "--aov") { a.normals = true; }
        else if (s == "--bloom")                    { a.bloom = true; }
        else if (s == "--denoise" || s == "--deno") { a.denoise = true; }
        else if (s == "--pathtrace" || s == "--pt") { a.pathtrace = true; }
        else if (s == "--bdpt") { a.bdpt = true; }
        else if (s == "--spectral") { a.spectral = true; }
        else if (s == "--mis") { a.mis = true; }
        else if (s == "--volume" || s == "--volumetric") { a.volume = true; }
        else if (s == "--exposure" && i+1 < argc) { a.exposure = std::strtof(argv[++i], nullptr); }
        else if (s == "--tonemap" && i+1 < argc) { a.tonemap = std::atoi(argv[++i]); }
        else if (s == "--godrays" || s == "--god_rays") { a.god_rays = true; }
        else if (s == "--motionblur" || s == "--motion_blur") { a.motion_blur = true; }
        else if (s == "--adaptive" || s == "--adapt") { a.adaptive = true; }
        else if (s == "--bokeh" && i+1 < argc) { a.bokeh_blades = std::atoi(argv[++i]); }
        else if (s == "--caustics") { a.caustics = true; }
        else if (s == "--ppm") { a.ppm = true; }
        else if (s == "--server") { a.is_server = true; }
        else if (s == "--worker" && i+1 < argc) { a.is_worker = true; a.server_ip = argv[++i]; }
        else if (s == "--port" && i+1 < argc) { a.port = std::atoi(argv[++i]); }
        else if (s == "--progress")                  { a.progress = true; }
        else if (s == "--width"   && i+1 < argc)   { a.override_w = std::atoi(argv[++i]); }
        else if (s == "--height"  && i+1 < argc)   { a.override_h = std::atoi(argv[++i]); }
        else if (s == "--samples" && i+1 < argc)   { a.override_s = std::atoi(argv[++i]); }
        else if (s == "--depth"   && i+1 < argc)   { a.override_d = std::atoi(argv[++i]); }
        else if (s == "--threads" && i+1 < argc)   { a.override_t = std::atoi(argv[++i]); }
        else if (s == "--frames"  && i+1 < argc)   { a.override_frames = std::atoi(argv[++i]); }
        else if (s == "--fps"     && i+1 < argc)   { a.override_fps = std::atoi(argv[++i]); }
        // positional: if --demo, the first positional is the OUTPUT path.
        // otherwise the first positional is the SCENE path, second is output.
        else if (a.demo) {
            if (a.out_path.empty()) a.out_path = s;
            else { std::cerr << "unknown argument: " << s << "\n"; std::exit(2); }
        }
        else if (a.scene_path.empty())             { a.scene_path = s; }
        else if (a.out_path.empty())               { a.out_path = s; }
        else { std::cerr << "unknown argument: " << s << "\n"; std::exit(2); }
    }
    return a;
}

// load + finalize a scene from path or built-in demo
static pencil::ParsedScene load_scene(const CliArgs& a) {
    pencil::ParsedScene ps;
    if (a.demo) {
        ps = pencil::build_demo_scene();
    } else {
        std::vector<pencil::ParseError> errs;
        if (!pencil::parse_pecl(a.scene_path, ps, errs)) {
            std::cerr << "failed to parse scene: " << a.scene_path << "\n";
            for (const auto& e : errs)
                std::cerr << "  line " << e.line << ": " << e.what << "\n";
            std::exit(3);
        }
        for (const auto& e : errs)
            std::cerr << "  warning line " << e.line << ": " << e.what << "\n";
    }

    if (a.override_w > 0) { ps.width = a.override_w; ps.aspect = (float)ps.width / (float)ps.height; ps.finalize(); }
    if (a.override_h > 0) { ps.height = a.override_h; ps.aspect = (float)ps.width / (float)ps.height; ps.finalize(); }
    if (a.override_s > 0) ps.samples = a.override_s;
    if (a.override_d > 0) ps.depth = a.override_d;
    if (a.no_bvh)         ps.scene.use_bvh = false;

    return ps;
}

// run one render, return timing + ray count
struct RunResult {
    double seconds;
    uint64_t rays;
    int threads;
};

static RunResult run_once(const pencil::ParsedScene& ps, int threads_override,
                          const CliArgs& a) {
    pencil::RenderSettings rs;
    rs.width     = ps.width;
    rs.height    = ps.height;
    rs.samples   = ps.samples;
    rs.max_depth = ps.depth;
    rs.threads   = threads_override;
    rs.normals_mode = a.normals;
    rs.bloom     = a.bloom;
    rs.denoise   = a.denoise;
    rs.pathtrace = a.pathtrace || ps.enable_pathtrace;
    rs.bdpt = a.bdpt || ps.enable_bdpt;
    rs.spectral = a.spectral || ps.enable_spectral;
    rs.mis = a.mis || ps.enable_mis;
    rs.volume = a.volume || ps.enable_volume;
    rs.sss_enabled = ps.enable_sss;
    if (ps.enable_bdpt && !ps.bdpt_bidirectional) {
        // bdpt: uni means unidirectional (use path tracer, not bdpt)
        rs.bdpt = false;
        rs.pathtrace = true;
    }
    rs.god_rays = a.god_rays;
    rs.motion_blur = a.motion_blur;
    rs.adaptive = a.adaptive;
    rs.bokeh_blades = a.bokeh_blades;
    rs.caustics = a.caustics || ps.enable_caustics; rs.ppm = a.ppm || ps.enable_ppm;
    rs.show_progress = a.progress;

    pencil::Renderer renderer(ps.scene, ps.camera, rs);
    std::vector<pencil::Color> pixels;
    pencil::RenderStats stats;

    auto t0 = std::chrono::steady_clock::now();
    renderer.render(pixels, &stats);
    auto t1 = std::chrono::steady_clock::now();

    RunResult r;
    r.seconds = std::chrono::duration<double>(t1 - t0).count();
    r.rays    = stats.rays_traced;
    r.threads = (threads_override > 0) ? threads_override
               : (int)std::thread::hardware_concurrency();
    return r;
}

// print the standard pre-render summary
static void print_summary(const CliArgs& a, const pencil::ParsedScene& ps) {
    int threads = (a.override_t > 0) ? a.override_t : (int)std::thread::hardware_concurrency();
    std::cout << "pencil " << pencil::PENCIL_VERSION << " - cpu ray tracer\n"
              << "  scene:   " << (a.demo ? "<built-in demo>" : a.scene_path) << "\n"
              << "  output:  " << a.out_path << "\n"
              << "  size:    " << ps.width << " x " << ps.height << "\n"
              << "  aa:      " << ps.samples << "x" << ps.samples
                              << " spp (" << ps.samples * ps.samples << " samples/pixel)\n"
              << "  depth:   " << ps.depth << "\n"
              << "  lights:  " << ps.scene.light_count() << "\n"
              << "  prims:   " << ps.scene.primitive_count() << "\n"
              << "  bvh:     " << (ps.scene.use_bvh ? "on" : "off (linear)") << "\n"
              << "  threads: " << threads << "\n";
}

// print the benchmark comparison: BVH vs no-BVH
static void print_benchmark(const CliArgs& a, const pencil::ParsedScene& ps_template) {
    std::cout << "pencil " << pencil::PENCIL_VERSION << " - benchmark\n";
    std::cout << "  scene:   " << (a.demo ? "<built-in demo>" : a.scene_path) << "\n";
    std::cout << "  size:    " << ps_template.width << " x " << ps_template.height << "\n";
    std::cout << "  aa:      " << ps_template.samples << "x" << ps_template.samples
                              << " spp (" << ps_template.samples * ps_template.samples << " spp)\n";
    std::cout << "  depth:   " << ps_template.depth << "\n";
    std::cout << "  lights:  " << ps_template.scene.light_count() << "\n";
    std::cout << "  prims:   " << ps_template.scene.primitive_count() << "\n";
    int threads = (a.override_t > 0) ? a.override_t : (int)std::thread::hardware_concurrency();
    std::cout << "  threads: " << threads << "\n";
    std::cout << "\n";

    // run 1: BVH on
    pencil::ParsedScene ps1 = ps_template;
    ps1.scene.use_bvh = true;
    ps1.scene.build_bvh();
    std::cout << "running with BVH..." << std::flush;
    RunResult bvh = run_once(ps1, a.override_t, a);
    std::cout << " done.\n";

    // run 2: BVH off
    pencil::ParsedScene ps2 = ps_template;
    ps2.scene.use_bvh = false;
    std::cout << "running without BVH..." << std::flush;
    RunResult lin = run_once(ps2, a.override_t, a);
    std::cout << " done.\n\n";

    // print comparison table
    double speedup = (bvh.seconds > 1e-9) ? (lin.seconds / bvh.seconds) : 0.0;

    std::cout << "================ benchmark results ================\n";
    std::cout << "  BVH:       " << std::fixed << std::setprecision(3) << bvh.seconds << " s\n";
    std::cout << "  No BVH:    " << std::fixed << std::setprecision(3) << lin.seconds << " s\n";
    std::cout << "  Speedup:   " << std::fixed << std::setprecision(1) << speedup << "x\n";
    std::cout << "  Rays:      " << with_commas(bvh.rays) << " (BVH)\n";
    std::cout << "             " << with_commas(lin.rays) << " (no BVH)\n";
    double mrays_bvh = (bvh.seconds > 1e-9) ? (bvh.rays / bvh.seconds / 1.0e6) : 0.0;
    double mrays_lin = (lin.seconds > 1e-9) ? (lin.rays / lin.seconds / 1.0e6) : 0.0;
    std::cout << "  Mrays/s:   " << std::fixed << std::setprecision(2) << mrays_bvh << " (BVH)\n";
    std::cout << "             " << std::fixed << std::setprecision(2) << mrays_lin << " (no BVH)\n";
    std::cout << "  Threads:   " << threads << "\n";
    std::cout << "  Primitives:" << ps_template.scene.primitive_count() << "\n";
    std::cout << "====================================================\n";
}

// v0.0.3: render an animation.
// for each frame, re-parse the scene with _anim_frame set, render, and save.
// output format determined by out_path extension: .mp4, .gif, .zip, or dir.
static int render_animation(const CliArgs& a, const pencil::ParsedScene& base_ps,
                            int total_frames, int fps) {
    namespace fs = std::filesystem;

    if (!a.quiet) {
        std::cout << "pencil " << pencil::PENCIL_VERSION << " - animation\n"
                  << "  scene:   " << (a.demo ? "<built-in demo>" : a.scene_path) << "\n"
                  << "  output:  " << a.out_path << "\n"
                  << "  frames:  " << total_frames << "\n"
                  << "  fps:     " << fps << "\n"
                  << "  size:    " << base_ps.width << " x " << base_ps.height << "\n"
                  << "  aa:      " << base_ps.samples << "x" << base_ps.samples << " spp\n"
                  << "  depth:   " << base_ps.depth << "\n";
    }

    // determine output format by extension
    std::string out_ext;
    auto dot_pos = a.out_path.find_last_of('.');
    if (dot_pos != std::string::npos) {
        out_ext = a.out_path.substr(dot_pos);
        for (auto& c : out_ext) c = (char)std::tolower(c);
    }

    // create temp dir for frames. v0.0.6: was hardcoded "/tmp/pencil_anim_<pid>"
    // which only worked on Linux. now uses portable_temp_dir() which
    // returns %TEMP% on Windows and $TMPDIR or /tmp on POSIX. note:
    // forward-slash path separator works on both Linux and Windows.
    std::string frames_dir = pencil::portable_temp_dir() + "/pencil_anim_" +
                             std::to_string(PENCIL_GETPID());
    pencil::portable_mkdir_p(frames_dir);

    // strip extension to get the base name for frame files
    std::string base_name = a.scene_path;
    size_t slash = base_name.find_last_of('/');
    if (slash != std::string::npos) base_name = base_name.substr(slash + 1);
    dot_pos = base_name.find_last_of('.');
    if (dot_pos != std::string::npos) base_name = base_name.substr(0, dot_pos);
    if (base_name.empty()) base_name = "frame";

    auto t_start = std::chrono::steady_clock::now();

    for (int frame = 1; frame <= total_frames; ++frame) {
        // re-parse the scene with _anim_frame = frame
        pencil::ParsedScene ps;
        ps._anim_frame = frame;
        if (a.demo) {
            // demo scene doesn't support animation, just copy
            ps = pencil::build_demo_scene();
        } else {
            std::vector<pencil::ParseError> errs;
            pencil::parse_pecl(a.scene_path, ps, errs);
        }

        // apply CLI overrides
        if (a.override_w > 0) { ps.width = a.override_w; ps.aspect = (float)ps.width / (float)ps.height; }
        if (a.override_h > 0) { ps.height = a.override_h; ps.aspect = (float)ps.width / (float)ps.height; }
        if (a.override_s > 0) ps.samples = a.override_s;
        if (a.override_d > 0) ps.depth = a.override_d;
        if (a.no_bvh) ps.scene.use_bvh = false;

        // render
        pencil::RenderSettings rs;
        rs.width = ps.width; rs.height = ps.height;
        rs.samples = ps.samples; rs.max_depth = ps.depth;
        rs.threads = a.override_t;
        rs.normals_mode = a.normals;
        rs.bloom = a.bloom;
    rs.denoise   = a.denoise;
    rs.pathtrace = a.pathtrace || ps.enable_pathtrace;
    rs.bdpt = a.bdpt || ps.enable_bdpt;
    rs.spectral = a.spectral || ps.enable_spectral;
    rs.mis = a.mis || ps.enable_mis;
    rs.volume = a.volume || ps.enable_volume;
    rs.sss_enabled = ps.enable_sss;
    if (ps.enable_bdpt && !ps.bdpt_bidirectional) {
        // bdpt: uni means unidirectional (use path tracer, not bdpt)
        rs.bdpt = false;
        rs.pathtrace = true;
    }
    rs.god_rays = a.god_rays;
    rs.motion_blur = a.motion_blur;
    rs.adaptive = a.adaptive;
    rs.bokeh_blades = a.bokeh_blades;
    rs.caustics = a.caustics || ps.enable_caustics; rs.ppm = a.ppm || ps.enable_ppm;

        pencil::Renderer renderer(ps.scene, ps.camera, rs);

        // v0.0.6: motion blur. compute camera positions at frame start and end.
        // v0.0.7: also pre-build sub-frame scenes for OBJECT motion blur.
        //   the renderer picks a random sub-scene per pixel sample, which
        //   blurs moving objects (e.g. the bouncing sphere in bounce.pecl).
        //   the old v0.0.6 code only blurred the camera, so scenes with
        //   moving objects + static camera showed no blur at all.
        if (a.motion_blur) {
            pencil::ParsedScene ps_prev, ps_next;
            ps_prev._anim_frame = frame;
            ps_next._anim_frame = frame + 1;
            std::vector<pencil::ParseError> errs;
            pencil::parse_pecl(a.scene_path, ps_prev, errs);
            pencil::parse_pecl(a.scene_path, ps_next, errs);
            ps_prev.finalize();
            ps_next.finalize();
            renderer.set_motion_blur_camera(ps_prev.camera.origin(),
                                            ps_next.camera.origin());
            // build sub-frame scenes. we use 4 sub-frames spanning the
            // shutter interval [frame, frame+1]. each is a full parse of
            // the .pecl with _anim_frame set to frame or frame+1.
            // (parser only accepts integer frames, so we approximate by
            // alternating between frame and frame+1 scenes.) a higher
            // count gives smoother blur but costs more parse + trace time.
            const int MB_SUBFRAMES = 4;
            std::vector<std::shared_ptr<pencil::Scene>> mb_scenes;
            mb_scenes.reserve(MB_SUBFRAMES);
            for (int i = 0; i < MB_SUBFRAMES; ++i) {
                int sub_frame = (i < MB_SUBFRAMES / 2) ? frame : frame + 1;
                auto ps_sub = std::make_shared<pencil::ParsedScene>();
                ps_sub->_anim_frame = sub_frame;
                std::vector<pencil::ParseError> sub_errs;
                if (pencil::parse_pecl(a.scene_path, *ps_sub, sub_errs)) {
                    ps_sub->finalize();
                    // copy the scene into a shared_ptr so the renderer
                    // can hold it beyond ps_sub's lifetime.
                    mb_scenes.push_back(
                        std::make_shared<pencil::Scene>(ps_sub->scene));
                }
            }
            renderer.set_motion_blur_scenes(std::move(mb_scenes));
        }

        std::vector<pencil::Color> pixels;
        renderer.render(pixels);

        // save frame
        char frame_num[8];
        snprintf(frame_num, sizeof(frame_num), "%04d", frame);
        std::string frame_path = frames_dir + "/" + base_name + "_" + frame_num + ".png";
        std::string err;
        pencil::ToneMap tm = (a.tonemap == 0) ? pencil::ToneMap::None :
                              (a.tonemap == 2) ? pencil::ToneMap::Reinhard :
                                                  pencil::ToneMap::ACES;
        if (!pencil::write_image(frame_path, pixels, rs.width, rs.height, err, tm, a.exposure)) {
            std::cerr << "error writing frame " << frame << ": " << err << "\n";
            return 4;
        }

        if (!a.quiet) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - t_start).count();
            double per_frame = elapsed / frame;
            double eta = per_frame * (total_frames - frame);
            int pct = frame * 100 / total_frames;
            std::cerr << "\r  frame " << frame << "/" << total_frames
                      << " (" << pct << "%) ETA " << (int)eta << "s    " << std::flush;
        }
    }
    if (!a.quiet) std::cerr << "\n";

    auto t_end = std::chrono::steady_clock::now();
    double total_sec = std::chrono::duration<double>(t_end - t_start).count();
    if (!a.quiet) {
        std::cout << "rendered " << total_frames << " frames in "
                  << std::fixed << std::setprecision(1) << total_sec << " s ("
                  << std::setprecision(1) << total_sec / total_frames << " s/frame)\n";
    }

    // encode output
    if (out_ext == ".zip") {
        // v0.0.6: build the zip with the built-in writer. the OLD code
        // shelled out to `zip -j` via std::system(), which (a) only
        // worked if the `zip` binary was installed (yes on linux, NO
        // on windows), (b) used `cd ... &&` unix shell syntax, and
        // (c) failed silently on Windows.
        //
        // walk the frames_dir, read every PNG, and add it to the
        // archive. filenames inside the archive use the base_name +
        // frame number + .png so they sort cleanly when extracted.
        std::vector<std::pair<std::string, std::vector<uint8_t>>> entries;
        for (int frame = 1; frame <= total_frames; ++frame) {
            char num[8];
            snprintf(num, sizeof(num), "%04d", frame);
            std::string frame_path = frames_dir + "/" + base_name + "_" + num + ".png";
            std::ifstream f(frame_path, std::ios::binary);
            if (!f) {
                std::cerr << "  warn: cannot open " << frame_path << " for zipping\n";
                continue;
            }
            std::vector<uint8_t> data((std::istreambuf_iterator<char>(f)),
                                       std::istreambuf_iterator<char>());
            f.close();
            std::string arc_name = base_name + "_" + num + ".png";
            entries.push_back({arc_name, std::move(data)});
        }
        std::string ziperr;
        if (!pencil::write_zip_archive(a.out_path, entries, ziperr)) {
            std::cerr << "warning: zip creation failed (" << ziperr
                      << "), frames left in " << frames_dir << "\n";
        } else if (!a.quiet) {
            std::cout << "wrote " << a.out_path
                      << " (" << entries.size() << " frames)\n";
        }
        // clean up frames on success
        fs::remove_all(frames_dir);
    } else if (out_ext == ".mp4" || out_ext == ".gif") {
        // use ffmpeg to encode. ffmpeg is a user-installed binary on
        // both linux and windows; we shell out to it but quote paths so
        // the call works on both shells.
        std::string frame_pattern = frames_dir + "/" + base_name + "_%04d.png";
        std::string cmd;
        if (out_ext == ".mp4") {
            cmd = "ffmpeg -y -framerate " + std::to_string(fps)
                + " -i \"" + frame_pattern + "\""
                + " -c:v libx264 -pix_fmt yuv420p -crf 18"
                + " \"" + a.out_path + "\" 2>&1";
        } else {
            // gif
            cmd = "ffmpeg -y -framerate " + std::to_string(fps)
                + " -i \"" + frame_pattern + "\""
                + " -vf \"scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse\""
                + " \"" + a.out_path + "\" 2>&1";
        }
        if (!a.quiet) std::cout << "encoding " << out_ext.substr(1) << "...\n";
        int rc = std::system(cmd.c_str());
        if (rc != 0) {
            std::cerr << "warning: ffmpeg encoding failed (rc=" << rc << "),\n"
                      << "  frames left in " << frames_dir << "\n"
                      << "  you can encode manually with:\n"
                      << "  ffmpeg -framerate " << fps << " -i \""
                      << frame_pattern << "\" \"" << a.out_path << "\"\n";
            return 5;
        } else if (!a.quiet) {
            std::cout << "wrote " << a.out_path << "\n";
        }
        // clean up frames
        fs::remove_all(frames_dir);
    } else {
        // no recognized video extension: just leave the frames in the temp dir
        // and tell the user where they are.
        std::cout << "frames written to: " << frames_dir << "\n"
                  << "encode manually with:\n"
                  << "  ffmpeg -framerate " << fps << " -i \""
                  << frames_dir << "/" << base_name << "_%04d.png\" output.mp4\n";
    }

    return 0;
}

int main(int argc, char** argv) {
    CliArgs a = parse_args(argc, argv);

    // v0.0.6: distributed rendering
    if (a.is_worker) {
        return pencil::run_worker(a.server_ip, a.port);
    }
    if (a.is_server) {
        if (a.scene_path.empty()) {
            std::cerr << "--server requires a scene file: pencil --server scene.pecl out.png\n";
            return 2;
        }
        std::string out = a.out_path.empty() ? "distributed_output.png" : a.out_path;
        // v0.0.6: pass 0 when the user didn't override on the CLI; the
        // server will fall back to the .pecl's render{} values, then to
        // the 480/360/2/4 hard defaults.
        int w = a.override_w > 0 ? a.override_w : 0;
        int h = a.override_h > 0 ? a.override_h : 0;
        int s = a.override_s > 0 ? a.override_s : 0;
        int d = a.override_d > 0 ? a.override_d : 0;
        // v0.0.6: ship the user's CLI flags through to the worker.
        // without this, the worker would silently render with
        // caustics/bloom/pathtrace/etc. all disabled.
        // v0.0.7: parse the .pecl to get render-block syntax settings
        // (cautype, spctrl, sss, bdpt, volume, mis, pt) and OR them
        // with CLI flags. without this, the distributed server would
        // ignore everything in the render {} block except size/spp/depth.
        pencil::ParsedScene ps_dist;
        std::vector<pencil::ParseError> dist_errs;
        pencil::parse_pecl(a.scene_path, ps_dist, dist_errs);
        pencil::RenderSettings user_rs;
        user_rs.samples = s;
        user_rs.max_depth = d;
        user_rs.bloom = a.bloom;
        user_rs.denoise = a.denoise;
        user_rs.pathtrace = a.pathtrace || ps_dist.enable_pathtrace;
        user_rs.bdpt = a.bdpt || ps_dist.enable_bdpt;
        user_rs.spectral = a.spectral || ps_dist.enable_spectral;
        user_rs.mis = a.mis || ps_dist.enable_mis;
        user_rs.volume = a.volume || ps_dist.enable_volume;
        user_rs.sss_enabled = ps_dist.enable_sss;
        user_rs.god_rays = a.god_rays;
        user_rs.motion_blur = a.motion_blur;
        user_rs.adaptive = a.adaptive;
        user_rs.caustics = a.caustics || ps_dist.enable_caustics;
        user_rs.ppm = a.ppm || ps_dist.enable_ppm;
        // v0.0.7: bdpt: uni means unidirectional (use path tracer, not bdpt)
        if (ps_dist.enable_bdpt && !ps_dist.bdpt_bidirectional) {
            user_rs.bdpt = false;
            user_rs.pathtrace = true;
        }
        user_rs.bokeh_blades = a.bokeh_blades;
        user_rs.normals_mode = a.normals;
        // v0.0.6: pass tonemap + exposure too so the server's final
        // write_image matches the local render's tone curve.
        pencil::ToneMap tm = (a.tonemap == 0) ? pencil::ToneMap::None :
                              (a.tonemap == 2) ? pencil::ToneMap::Reinhard :
                                                  pencil::ToneMap::ACES;
        return pencil::run_server(a.scene_path, out, a.port, w, h, s, d, &user_rs, tm, a.exposure);
    }

    if (!a.demo && !a.benchmark && a.scene_path.empty()) {
        print_help();
        return 0;
    }
    if (a.benchmark && a.scene_path.empty() && !a.demo) {
        std::cerr << "--benchmark requires a scene path (or --demo).\n";
        return 2;
    }

    pencil::ParsedScene ps = load_scene(a);

    // default output path
    if (a.out_path.empty()) {
        if (a.demo) {
            a.out_path = "pencil_demo.ppm";
        } else {
            size_t dot = a.scene_path.find_last_of('.');
            a.out_path = (dot == std::string::npos ? a.scene_path
                                                    : a.scene_path.substr(0, dot)) + ".ppm";
        }
    }

    if (a.benchmark) {
        print_benchmark(a, ps);
        return 0;
    }

    // v0.0.3: animation rendering.
    // if --animate flag or render { frames: N } is set, render all frames.
    int total_frames = (a.override_frames > 0) ? a.override_frames : ps.frames;
    int fps = (a.override_fps > 0) ? a.override_fps : ps.fps;
    if (a.animate || total_frames > 0) {
        if (total_frames <= 0) {
            std::cerr << "animation requested but no frame count set.\n"
                      << "use --frames N or add 'frames: N' to the render block.\n";
            return 2;
        }
        return render_animation(a, ps, total_frames, fps);
    }

    if (!a.quiet) print_summary(a, ps);

    // render once and keep the pixels
    pencil::RenderSettings rs;
    rs.width = ps.width; rs.height = ps.height;
    rs.samples = ps.samples; rs.max_depth = ps.depth;
    rs.threads = a.override_t;
    rs.normals_mode = a.normals;
    rs.bloom     = a.bloom;
    rs.denoise   = a.denoise;
    rs.pathtrace = a.pathtrace || ps.enable_pathtrace;
    rs.bdpt = a.bdpt || ps.enable_bdpt;
    rs.spectral = a.spectral || ps.enable_spectral;
    rs.mis = a.mis || ps.enable_mis;
    rs.volume = a.volume || ps.enable_volume;
    rs.sss_enabled = ps.enable_sss;
    if (ps.enable_bdpt && !ps.bdpt_bidirectional) {
        // bdpt: uni means unidirectional (use path tracer, not bdpt)
        rs.bdpt = false;
        rs.pathtrace = true;
    }
    rs.god_rays = a.god_rays;
    rs.motion_blur = a.motion_blur;
    rs.adaptive = a.adaptive;
    rs.bokeh_blades = a.bokeh_blades;
    rs.caustics = a.caustics || ps.enable_caustics; rs.ppm = a.ppm || ps.enable_ppm;
    rs.show_progress = a.progress;
    pencil::Renderer renderer(ps.scene, ps.camera, rs);
    std::vector<pencil::Color> pixels;
    pencil::RenderStats stats;

    auto t0 = std::chrono::steady_clock::now();
    renderer.render(pixels, &stats);
    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();

    if (!a.quiet) {
        std::cout << "rendered in " << std::fixed << std::setprecision(3) << sec << " s"
                  << " (" << with_commas(stats.rays_traced) << " rays, "
                  << std::fixed << std::setprecision(2)
                  << (sec > 1e-9 ? stats.rays_traced / sec / 1.0e6 : 0.0)
                  << " Mrays/s)\n";
    }

    std::string err;
    pencil::ToneMap tm = (a.tonemap == 0) ? pencil::ToneMap::None :
                          (a.tonemap == 2) ? pencil::ToneMap::Reinhard :
                                              pencil::ToneMap::ACES;
    if (!pencil::write_image(a.out_path, pixels, rs.width, rs.height, err, tm, a.exposure)) {
        std::cerr << "error: " << err << "\n";
        return 4;
    }
    if (!a.quiet) std::cout << "wrote " << a.out_path << "\n";
    return 0;
}
