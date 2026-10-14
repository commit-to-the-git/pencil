// distributed.hpp - distributed rendering for pencil
// SPDX-License-Identifier: Apache-2.0
//
// usage:
//   pencil --server scene.pecl out.png --port 8080
//   pencil --worker 192.168.1.100 --port 8080
//
// v0.0.6 hardening:
//   * Server now bundles EVERY asset the .pecl references (mesh .obj,
//     skybox images, textures, includes) and ships them to the worker
//     before tile dispatch. Worker writes them under a working dir and
//     parses the .pecl from there, so relative paths resolve correctly
//     on either side.
//   * Server now ships the user's RenderSettings (caustics, bloom,
//     pathtrace, god_rays, motion_blur, adaptive, denoise, spp, depth,
//     bokeh_blades) to the worker. Worker applies them. Previously the
//     worker silently stripped ALL of these to false/0, which meant
//     distributed renders never had caustics, bloom, or pathtracing
//     even if the user asked for them.
//   * Worker auto-detects aspect mismatch (scene says 512x640 portrait,
//     CLI says --width 320 --height 240 landscape) and calls
//     Camera::set_aspect(target_aspect). Fixes the mesh_dist horizontal
//     stretch bug. Prints "adjusting camera aspect: 0.8 -> 1.33333".
//   * Worker accept timeout: 30 s for the first worker, 3 s for any
//     additional worker after the first. lets the server pick up
//     late-joining workers without hanging forever.
//   * strip the LLM's forced `rs.denoise=false; rs.bloom=false; ...`
//     block in render_tile() -- those were silently disabling features.
//     render_tile() now respects whatever RenderSettings the worker
//     passes in (which is what the server shipped over the wire).

#pragma once

#include "image_io.hpp"
#include "pecl_parser.hpp"
#include "portable.hpp"
#include "renderer.hpp"
#include "vec3.hpp"

#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

// v0.0.6: portable.hpp owns all platform-specific includes (winsock2,
// ws2tcpip, windows.h, process.h, direct.h on Windows; arpa/inet,
// netinet/in, sys/socket, sys/stat, unistd on POSIX) and exposes
// PENCIL_CLOSESOCK / PENCIL_MKDIR_ONE / PENCIL_CHDIR / PENCIL_GETCWD
// / PENCIL_PATHSEP / socklen_t typedef. we just use them here so
// distributed.hpp has no #ifdef _WIN32 of its own to maintain.

#define CLOSE_SOCKET PENCIL_CLOSESOCK
#define MKDIR(p)     PENCIL_MKDIR_ONE(p)
#define CHDIR(p)     PENCIL_CHDIR(p)
#define GETCWD(buf, sz) PENCIL_GETCWD(buf, sz)

namespace pencil {

enum class MsgType : uint32_t {
    TileRequest = 1,
    TileResult  = 2,
    SceneData   = 3,
    Done        = 4,
    WorkerReady = 5,
    // v0.0.6: asset + settings passthrough.
    AssetFile   = 6,   // one referenced file (mesh/texture/include/...)
    AssetsDone  = 7,   // sentinel: no more assets, worker can now parse
    RenderSettings = 8, // serialized user-side RenderSettings
};

#pragma pack(push, 1)
struct TileRequestMsg {
    uint32_t x, y, w, h;
    uint32_t frame;
    uint32_t full_w, full_h;
};
struct TileResultHeader {
    uint32_t x, y, w, h;
};
// v0.0.6: header for an asset file sent server -> worker.
//   path_len bytes of relative path follow, then data_len bytes of file.
//   relative path uses forward slashes; worker rewrites to native if needed.
struct AssetFileHeader {
    uint32_t path_len;
    uint64_t data_len;
};
// v0.0.6: mirror of RenderSettings over the wire. we ship a curated subset
// (the ints/floats/bools the user actually configures); tile_size and
// threads are re-derived on the worker side. bools are uint32_t for
// portable alignment.
struct RenderSettingsMsg {
    uint32_t samples;
    uint32_t max_depth;
    uint32_t pathtrace_bounces;
    uint32_t soft_shadow_samples;
    uint32_t god_ray_steps;
    uint32_t caustic_photons;
    uint32_t bokeh_blades;
    uint32_t tile_size;
    uint32_t threads;
    uint32_t adaptive;        // bool
    uint32_t caustics;        // bool
    uint32_t ppm;             // bool (v0.0.7)
    uint32_t pathtrace;       // bool
    uint32_t bdpt;            // bool (v0.0.7)
    uint32_t spectral;        // bool (v0.0.7)
    uint32_t mis;             // bool (v0.0.7)
    uint32_t volume;          // bool (v0.0.7)
    uint32_t sss_enabled;     // bool (v0.0.7)
    uint32_t normals_mode;   // bool
    uint32_t motion_blur;     // bool
    uint32_t god_rays;        // bool
    uint32_t bloom;           // bool
    uint32_t denoise;         // bool
    uint32_t show_progress;   // bool
    float    adaptive_threshold;
    float    shadow_bias;
    float    exposure;
};
#pragma pack(pop)

inline bool net_init() {
#ifdef _WIN32
    WSADATA wsa;
    return WSAStartup(MAKEWORD(2, 2), &wsa) == 0;
#else
    return true;
#endif
}

inline void net_cleanup() {
#ifdef _WIN32
    WSACleanup();
#endif
}

inline bool send_all(int sock, const void* data, size_t len) {
    const char* p = (const char*)data;
    while (len > 0) {
        int sent = send(sock, p, (int)len, 0);
        if (sent <= 0) return false;
        p += sent;
        len -= sent;
    }
    return true;
}

inline bool recv_all(int sock, void* data, size_t len) {
    char* p = (char*)data;
    while (len > 0) {
        int recvd = recv(sock, p, (int)len, 0);
        if (recvd <= 0) return false;
        p += recvd;
        len -= recvd;
    }
    return true;
}

inline bool send_msg(int sock, MsgType type, const void* payload, size_t len) {
    uint32_t t = (uint32_t)type;
    uint32_t l = (uint32_t)len;
    if (!send_all(sock, &t, sizeof(t))) return false;
    if (!send_all(sock, &l, sizeof(l))) return false;
    if (len > 0 && !send_all(sock, payload, len)) return false;
    return true;
}

inline bool recv_msg_header(int sock, MsgType& type, uint32_t& len) {
    uint32_t t;
    if (!recv_all(sock, &t, sizeof(t))) return false;
    if (!recv_all(sock, &len, sizeof(len))) return false;
    type = (MsgType)t;
    return true;
}

// ---------------------------------------------------------------------------
// v0.0.6: asset discovery + bundling helpers.
//
// Resolution rules (must match what the parser does on the worker side):
//   * mesh / texture / skybox paths are resolved CWD-relative by the
//     parser (load_obj() uses std::ifstream with the raw path). The
//     worker chdir()s to its working dir before parsing, so the worker
//     writes these files at <working_dir>/<raw_path> and the parser
//     finds them via CWD. wire rel_path = raw_path verbatim.
//   * include paths are resolved by the parser via _base_dir, which
//     starts as the directory of the scene file passed to parse_pecl().
//     On the worker, scene.pecl is at <working_dir>/scene.pecl, so
//     _base_dir = <working_dir>. For nested includes, _base_dir is
//     the directory of the parent .pecl. So wire rel_path for an
//     include = (parent's wire_rel_dir) + "/" + raw_path, where
//     wire_rel_dir starts as "" and grows by dir_of(raw_path) on
//     each recursion.
//   * Server-side reading mirrors the parser's resolution: read mesh
//     paths CWD-relative (i.e., just open raw_path), read include
//     paths base_dir-relative.
//
// We scan recursively over the include graph; a visited set prevents
// loops.
// ---------------------------------------------------------------------------

// scan_pecl_for_refs: walk a .pecl source and collect asset references.
//   src              - the .pecl text to scan
//   base_dir         - the directory used to resolve include paths
//                      (server-side mirror of the parser's _base_dir)
//   wire_rel_dir     - the directory of THIS .pecl relative to the
//                      worker's working_dir (used to compute the
//                      wire rel_path for nested includes)
//   out              - appended (server_abs_path, wire_rel_path) pairs
//   visited          - set of wire_rel_paths already shipped (loop guard)
inline void scan_pecl_for_refs(const std::string& src,
                               const std::string& base_dir,
                               const std::string& wire_rel_dir,
                               std::vector<std::pair<std::string,std::string>>& out,
                               std::set<std::string>& visited) {
    static const std::vector<std::string> keys = {
        "file", "path", "obj", "tex", "texture", "nmap", "normal",
        "sky", "env", "diffuse_map", "specular_map", "roughness_map",
        "emission_map", "albedo_map"
    };
    size_t i = 0;
    while (i < src.size()) {
        // skip # comments to end of line
        if (src[i] == '#') {
            while (i < src.size() && src[i] != '\n') i++;
            continue;
        }
        // try to match a keyword followed by ':'
        bool matched = false;
        for (const auto& k : keys) {
            if (src.compare(i, k.size(), k) == 0) {
                size_t after = i + k.size();
                size_t j = after;
                while (j < src.size() && (src[j] == ' ' || src[j] == '\t')) j++;
                if (j < src.size() && src[j] == ':') {
                    // left boundary check: prev char must not be identifier
                    bool left_ok = (i == 0);
                    if (i > 0) {
                        char pc = src[i - 1];
                        left_ok = !(std::isalnum((unsigned char)pc) || pc == '_');
                    }
                    if (left_ok) {
                        i = j + 1;
                        matched = true;
                        break;
                    }
                }
            }
        }
        if (!matched) { i++; continue; }

        // skip whitespace after ':'
        while (i < src.size() && (src[i] == ' ' || src[i] == '\t')) i++;

        // read the path: either quoted or unquoted-until-terminator
        std::string raw;
        if (i < src.size() && src[i] == '"') {
            i++;
            while (i < src.size() && src[i] != '"') {
                raw.push_back(src[i++]);
            }
            if (i < src.size()) i++; // closing quote
        } else {
            while (i < src.size()) {
                char c = src[i];
                if (c == '\n' || c == ',' || c == '}' || c == '#') break;
                raw.push_back(c);
                i++;
            }
            while (!raw.empty() && (raw.back() == ' ' || raw.back() == '\t')) raw.pop_back();
        }

        if (raw.empty()) continue;
        // ignore non-file values (numbers, vectors, true/false)
        if (raw[0] == '<') continue;
        if (raw == "true" || raw == "false") continue;
        bool is_number = true;
        for (char c : raw) {
            if (!std::isdigit((unsigned char)c) && c != '.' && c != '-' && c != '+' && c != 'e' && c != 'E') {
                is_number = false; break;
            }
        }
        if (is_number) continue;

        // The .pecl syntax does not distinguish "this is a mesh path"
        // from "this is an include path" at the token level -- we infer
        // it from the file extension. Anything ending in .pecl is treated
        // as a nested scene (resolved via base_dir); everything else is
        // treated as a binary asset (resolved CWD-relative).
        bool is_include = (raw.size() >= 5 &&
                           raw.compare(raw.size() - 5, 5, ".pecl") == 0);

        std::string wire_rel_path;
        std::string server_abs_path;
        if (is_include) {
            // base_dir-relative on server side
            if (!raw.empty() && (raw[0] == '/' || (raw.size() >= 2 && raw[1] == ':'))) {
                server_abs_path = raw;
            } else {
                server_abs_path = base_dir.empty() ? raw : (base_dir + "/" + raw);
            }
            // wire rel_path grows under the parent's wire_rel_dir
            if (wire_rel_dir.empty()) wire_rel_path = raw;
            else wire_rel_path = wire_rel_dir + "/" + raw;
        } else {
            // CWD-relative on server side (mirrors load_obj())
            server_abs_path = raw;
            // wire rel_path is just the raw path (parser uses CWD on worker)
            wire_rel_path = raw;
        }

        if (wire_rel_path.empty() || visited.count(wire_rel_path)) {
            // continue scanning even if visited -- there might be more
            // refs further down the source
            continue;
        }
        visited.insert(wire_rel_path);
        out.push_back({server_abs_path, wire_rel_path});

        // if it's an include, recurse with new base_dir and wire_rel_dir
        if (is_include) {
            std::ifstream f(server_abs_path);
            if (f) {
                std::string sub((std::istreambuf_iterator<char>(f)),
                                std::istreambuf_iterator<char>());
                f.close();
                // new base_dir = directory of server_abs_path
                std::string new_base;
                size_t slash = server_abs_path.find_last_of('/');
                if (slash != std::string::npos) new_base = server_abs_path.substr(0, slash);
                // new wire_rel_dir = dir part of raw (joined with parent's wire_rel_dir)
                std::string raw_dir;
                size_t rslash = raw.find_last_of('/');
                if (rslash != std::string::npos) raw_dir = raw.substr(0, rslash);
                std::string new_wire_rel_dir;
                if (wire_rel_dir.empty()) new_wire_rel_dir = raw_dir;
                else new_wire_rel_dir = raw_dir.empty() ? wire_rel_dir : (wire_rel_dir + "/" + raw_dir);
                scan_pecl_for_refs(sub, new_base, new_wire_rel_dir, out, visited);
            }
        }
    }
}

// normalize a path's slashes for cross-platform (always use '/')
inline std::string normalize_slashes(std::string p) {
    for (char& c : p) if (c == '\\') c = '/';
    return p;
}

// make all parent directories of `file_path` (forward-slash separated).
// used by the worker to recreate the asset tree under its working dir.
inline bool mkdir_parents(const std::string& file_path) {
    std::string dir;
    size_t pos = 0;
    while ((pos = file_path.find('/', pos + 1)) != std::string::npos) {
        dir = file_path.substr(0, pos);
        if (dir.empty()) continue;
        MKDIR(dir.c_str()); // ignore EEXIST; if it fails for other reasons,
                            // the open() call below will report it.
    }
    return true;
}

// ---------------------------------------------------------------------------
// render a tile of the image. only traces rays for the tile region,
// not the full image. uses camera UV offset.
//
// v0.0.6: caller is responsible for calling camera.set_aspect() before
// invoking this if the worker's target aspect differs from the scene's.
// We no longer force rs.denoise/bloom/pathtrace/caustics/etc to false --
// we respect whatever the caller passed in (which is what the server
// shipped over the wire via RenderSettingsMsg).
// ---------------------------------------------------------------------------

inline void render_tile(const Scene& scene, const Camera& camera,
                        int tile_x, int tile_y, int tile_w, int tile_h,
                        int full_w, int full_h, int spp, int depth,
                        std::vector<Color>& out_colors,
                        const RenderSettings* rs_override = nullptr) {
    out_colors.assign(tile_w * tile_h, {0, 0, 0});
    // v0.0.6 bugfix: seed per-pixel like the local renderer
    // (render_tile_region in renderer.hpp does
    //    Rng rng(i * 97861 + j * 1973 + 1)
    // per pixel). The OLD distributed code used ONE shared Rng(12345)
    // for the whole tile, which (a) made every tile's noise pattern
    // identical, producing a visible tiling artifact at tile
    // boundaries, and (b) made distributed renders differ from local
    // renders of the same scene. Now both paths agree pixel-for-pixel.
    RenderSettings rs;
    rs.width = full_w; rs.height = full_h;
    rs.samples = spp; rs.max_depth = depth;
    if (rs_override) rs = *rs_override;  // v0.0.6: respect caller's settings
    rs.width = full_w; rs.height = full_h;  // always reset these to the
                                            // full image dims (the
                                            // server may have shipped a
                                            // 0 here, that's fine for
                                            // per-tile rendering)
    if (rs.samples <= 0) rs.samples = spp;
    if (rs.max_depth <= 0) rs.max_depth = depth;
    Renderer renderer(scene, camera, rs);

    for (int j = 0; j < tile_h; ++j) {
        for (int i = 0; i < tile_w; ++i) {
            int full_px = tile_x + i;
            int full_py = tile_y + j;
            // v0.0.6: per-pixel Rng seed identical to the local renderer,
            // so distributed and local renders produce the same image.
            Rng rng((uint32_t)(full_px * 97861u + full_py * 1973u + 1u));
            Color acc = {0, 0, 0};
            int n = rs.samples > 0 ? rs.samples : spp;
            for (int sy = 0; sy < n; ++sy) {
                for (int sx = 0; sx < n; ++sx) {
                    float u = (full_px + (sx + 0.5f) / n) / (float)full_w;
                    float v = 1.0f - (full_py + (sy + 0.5f) / n) / (float)full_h;
                    u += rng.sym() * (0.5f / full_w);
                    v += rng.sym() * (0.5f / full_h);
                    Ray r = camera.ray_at(u, v, &rng);
                    // v0.0.6: use rs.max_depth (server's setting) not the
                    // .pecl-derived `depth` param. before this fix, the
                    // server's --depth override was silently ignored by
                    // the worker.
                    acc += renderer.trace(r, rs.max_depth > 0 ? rs.max_depth : depth, rng);
                }
            }
            acc = acc / (float)(n * n);
            out_colors[j * tile_w + i] = acc;
        }
    }
}

// ---------------------------------------------------------------------------
// Server: discover assets, send scene + assets + settings, dispatch tiles.
// ---------------------------------------------------------------------------

inline int run_server(const std::string& scene_path, const std::string& out_path,
                      int port, int img_w, int img_h, int spp, int depth,
                      const RenderSettings* user_rs = nullptr,
                      ToneMap tonemap = ToneMap::ACES,
                      float exposure = 0.0f) {
    if (!net_init()) {
        std::cerr << "network init failed\n";
        return 1;
    }

    // v0.0.6 bugfix: if the caller passed spp<=0 or depth<=0 (i.e. no
    // --spp / --depth on the CLI), fall back to the .pecl's render{} block
    // values, then to 2/4. Before this fix the server ignored the .pecl's
    // spp/depth entirely and always used 2/4 unless the user passed CLI
    // overrides.
    if (spp <= 0 || depth <= 0) {
        ParsedScene peek;
        std::vector<ParseError> perrs;
        if (parse_pecl(scene_path, peek, perrs)) {
            // peek.finalize(); // not needed; we only read width/height/samples/depth
            if (spp    <= 0 && peek.samples > 0) spp    = peek.samples;
            if (depth  <= 0 && peek.depth    > 0) depth  = peek.depth;
            // also pick up width/height from the .pecl if caller didn't
            // pass them (img_w/img_h == 0 means "no override").
            if (img_w <= 0 && peek.width  > 0) img_w = peek.width;
            if (img_h <= 0 && peek.height > 0) img_h = peek.height;
        }
    }
    if (spp   <= 0) spp   = 2;
    if (depth <= 0) depth = 4;
    if (img_w <= 0) img_w = 480;
    if (img_h <= 0) img_h = 360;

    // 1. read the .pecl text
    std::ifstream f(scene_path);
    if (!f) { std::cerr << "cannot open scene: " << scene_path << "\n"; return 1; }
    std::string scene_text((std::istreambuf_iterator<char>(f)),
                            std::istreambuf_iterator<char>());
    f.close();

    // 2. discover all asset references (mesh .obj, skybox images, textures,
    // included .pecl files, etc.). The main .pecl's wire_rel_dir is ""
    // because the worker will write scene.pecl at the working dir root.
    std::string base_dir;
    {
        size_t slash = scene_path.find_last_of("/\\");
        if (slash != std::string::npos) base_dir = scene_path.substr(0, slash);
    }
    std::vector<std::pair<std::string,std::string>> assets; // (server_abs, wire_rel)
    std::set<std::string> visited;
    scan_pecl_for_refs(scene_text, base_dir, "", assets, visited);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { std::cerr << "socket failed\n"; return 1; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "bind failed on port " << port << "\n";
        CLOSE_SOCKET(srv);
        return 1;
    }

    if (listen(srv, 8) < 0) {
        std::cerr << "listen failed\n";
        CLOSE_SOCKET(srv);
        return 1;
    }

    std::cout << "pencil server on port " << port << "\n"
              << "  scene: " << scene_path << "\n"
              << "  output: " << out_path << " (" << img_w << "x" << img_h << ")\n"
              << "  spp: " << spp << " depth: " << depth << "\n"
              << "  discovered " << assets.size() << " asset(s)\n";
    for (const auto& [abs, rel] : assets) {
        std::cout << "    - " << rel << "  (" << abs << ")\n";
    }
    std::cout << "  waiting for workers...\n";

    std::vector<int> workers;
    // accept workers with a 3-second timeout after the first one connects
    bool got_first = false;
    while (true) {
        // set socket timeout
        timeval tv;
        tv.tv_sec = got_first ? 3 : 30;  // 30s for first, 3s for more
        tv.tv_usec = 0;
        setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

        sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client = accept(srv, (sockaddr*)&client_addr, &client_len);
        if (client < 0) {
            if (got_first) break;  // timeout after first worker
            continue;  // keep waiting for first
        }

        // clear timeout
        tv.tv_sec = 0; tv.tv_usec = 0;
        setsockopt(srv, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

        char ip[32];
        inet_ntop(AF_INET, &client_addr.sin_addr, ip, sizeof(ip));
        std::cout << "worker: " << ip << "\n";

        MsgType type;
        uint32_t len;
        if (!recv_msg_header(client, type, len) || type != MsgType::WorkerReady) {
            CLOSE_SOCKET(client);
            continue;
        }

        // 1. send the .pecl text
        if (!send_msg(client, MsgType::SceneData, scene_text.data(), scene_text.size())) {
            CLOSE_SOCKET(client);
            continue;
        }

        // 2. send each asset file
        bool send_ok = true;
        for (const auto& [abs, rel] : assets) {
            std::ifstream af(abs, std::ios::binary);
            if (!af) {
                std::cerr << "  warn: cannot open asset " << abs << ", skipping\n";
                continue;
            }
            std::string content((std::istreambuf_iterator<char>(af)),
                                std::istreambuf_iterator<char>());
            af.close();
            std::string rel_norm = normalize_slashes(rel);
            AssetFileHeader hdr = {(uint32_t)rel_norm.size(), (uint64_t)content.size()};
            // single message: header + path + data
            std::string payload;
            payload.reserve(sizeof(hdr) + rel_norm.size() + content.size());
            payload.append((const char*)&hdr, sizeof(hdr));
            payload.append(rel_norm);
            payload.append(content);
            if (!send_msg(client, MsgType::AssetFile, payload.data(), payload.size())) {
                send_ok = false;
                break;
            }
        }
        if (!send_ok) { CLOSE_SOCKET(client); continue; }

        // 3. signal end of assets
        if (!send_msg(client, MsgType::AssetsDone, nullptr, 0)) {
            CLOSE_SOCKET(client);
            continue;
        }

        // 4. ship the user's RenderSettings
        RenderSettingsMsg rsm{};
        {
            RenderSettings rs;
            if (user_rs) rs = *user_rs;
            rs.samples = spp; rs.max_depth = depth;
            rsm.samples = (uint32_t)rs.samples;
            rsm.max_depth = (uint32_t)rs.max_depth;
            rsm.pathtrace_bounces = (uint32_t)rs.pathtrace_bounces;
            rsm.soft_shadow_samples = (uint32_t)rs.soft_shadow_samples;
            rsm.god_ray_steps = (uint32_t)rs.god_ray_steps;
            rsm.caustic_photons = (uint32_t)rs.caustic_photons;
            rsm.bokeh_blades = (uint32_t)rs.bokeh_blades;
            rsm.tile_size = (uint32_t)rs.tile_size;
            rsm.threads = (uint32_t)rs.threads;
            rsm.adaptive = rs.adaptive ? 1 : 0;
            rsm.caustics = rs.caustics ? 1 : 0; rsm.ppm = rs.ppm ? 1 : 0;
            rsm.pathtrace = rs.pathtrace ? 1 : 0; rsm.bdpt = rs.bdpt ? 1 : 0; rsm.spectral = rs.spectral ? 1 : 0; rsm.mis = rs.mis ? 1 : 0; rsm.volume = rs.volume ? 1 : 0; rsm.sss_enabled = rs.sss_enabled ? 1 : 0;
            rsm.normals_mode = rs.normals_mode ? 1 : 0;
            rsm.motion_blur = rs.motion_blur ? 1 : 0;
            rsm.god_rays = rs.god_rays ? 1 : 0;
            rsm.bloom = rs.bloom ? 1 : 0;
            rsm.denoise = rs.denoise ? 1 : 0;
            rsm.show_progress = false ? 1 : 0; // never show progress on worker
            rsm.adaptive_threshold = rs.adaptive_threshold;
            rsm.shadow_bias = rs.shadow_bias;
            rsm.exposure = 0.0f;
        }
        if (!send_msg(client, MsgType::RenderSettings, &rsm, sizeof(rsm))) {
            CLOSE_SOCKET(client);
            continue;
        }

        workers.push_back(client);
        got_first = true;
        std::cout << "  " << workers.size() << " worker(s) connected. "
                  << "Waiting 3s for more...\n";
    }

    if (workers.empty()) {
        std::cerr << "no workers\n";
        CLOSE_SOCKET(srv);
        net_cleanup();
        return 1;
    }

    // 64x64 tiles
    int tile_size = 64;
    struct Tile { int x, y, w, h; };
    std::vector<Tile> tiles;
    for (int y = 0; y < img_h; y += tile_size) {
        for (int x = 0; x < img_w; x += tile_size) {
            Tile t;
            t.x = x; t.y = y;
            t.w = std::min(tile_size, img_w - x);
            t.h = std::min(tile_size, img_h - y);
            tiles.push_back(t);
        }
    }
    std::cout << "  " << tiles.size() << " tiles (" << tile_size << "x" << tile_size << ")\n";

    std::vector<uint8_t> image(img_w * img_h * 3, 0);  // unused, kept for ABI compat
    (void)image;
    // v0.0.6: assemble HDR Color buffer; tonemap/gamma/8-bit happen
    // only at the end via write_image, AFTER bloom/denoise.
    std::vector<Color> image_hdr((size_t)img_w * img_h, Color{0,0,0});
    std::mutex image_mutex;
    std::atomic<int> tiles_done(0);
    std::atomic<int> next_tile(0);
    int total_tiles = (int)tiles.size();

    auto handle_worker = [&](int sock) {
        while (true) {
            int tile_idx = next_tile.fetch_add(1);
            if (tile_idx >= total_tiles) {
                send_msg(sock, MsgType::Done, nullptr, 0);
                break;
            }

            Tile& t = tiles[tile_idx];
            TileRequestMsg req = {(uint32_t)t.x, (uint32_t)t.y,
                                  (uint32_t)t.w, (uint32_t)t.h, 0,
                                  (uint32_t)img_w, (uint32_t)img_h};

            if (!send_msg(sock, MsgType::TileRequest, &req, sizeof(req))) break;

            MsgType type;
            uint32_t len;
            if (!recv_msg_header(sock, type, len) || type != MsgType::TileResult) break;

            TileResultHeader hdr;
            if (!recv_all(sock, &hdr, sizeof(hdr))) break;

            // v0.0.6: worker now ships HDR Color (3x float) per pixel
            // instead of LDR RGB8. the server assembles the full HDR
            // image, then runs bloom/denoise/tonemap/gamma on the
            // assembled buffer -- so distributed renders get the same
            // post-processing pipeline as local renders.
            std::vector<Color> tile_hdr(hdr.w * hdr.h);
            if (!recv_all(sock, tile_hdr.data(), tile_hdr.size() * sizeof(Color))) break;

            {
                std::lock_guard<std::mutex> lock(image_mutex);
                for (uint32_t row = 0; row < hdr.h; ++row) {
                    for (uint32_t col = 0; col < hdr.w; ++col) {
                        int dst = (hdr.y + row) * img_w + (hdr.x + col);
                        image_hdr[dst] = tile_hdr[row * hdr.w + col];
                    }
                }
            }

            int done = tiles_done.fetch_add(1) + 1;
            std::cerr << "\r  " << done << "/" << total_tiles << " ("
                      << (done * 100 / total_tiles) << "%)   " << std::flush;
        }
        CLOSE_SOCKET(sock);
    };

    std::vector<std::thread> handler_threads;
    for (int w : workers) {
        handler_threads.emplace_back(handle_worker, w);
    }
    for (auto& t : handler_threads) t.join();
    std::cerr << "\n";

    // v0.0.6: apply bloom/denoise on the assembled HDR buffer BEFORE
    // tonemap+gamma. this matches the local render() pipeline so
    // distributed renders with --bloom / --denoise produce the same
    // pixels as local renders.
    if (user_rs && (user_rs->bloom || user_rs->denoise)) {
        // build a dummy renderer just to access post_process. the
        // camera/scene are unused for post-processing. we feed the
        // user's actual RenderSettings so bloom_threshold etc. are
        // correct.
        Scene dummy_scene;
        Camera dummy_cam;
        Renderer post(dummy_scene, dummy_cam, *user_rs);
        post.post_process(image_hdr, img_w, img_h);
    }

    // tonemap -> gamma 2.2 -> 8-bit -> PNG/PPM (same as local path).
    std::string err;
    if (!write_image(out_path, image_hdr, img_w, img_h, err, tonemap, exposure)) {
        std::cerr << "failed: " << err << "\n";
    } else {
        std::cout << "wrote " << out_path << "\n";
    }

    CLOSE_SOCKET(srv);
    net_cleanup();
    return 0;
}

// ---------------------------------------------------------------------------
// Worker: connect, receive scene + assets + settings, render tiles.
// ---------------------------------------------------------------------------

inline int run_worker(const std::string& server_ip, int port) {
    if (!net_init()) {
        std::cerr << "network init failed\n";
        return 1;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) { std::cerr << "socket failed\n"; return 1; }

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, server_ip.c_str(), &addr.sin_addr);

    std::cout << "connecting to " << server_ip << ":" << port << "...\n";
    if (connect(sock, (sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "connect failed\n";
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    std::cout << "connected\n";

    if (!send_msg(sock, MsgType::WorkerReady, nullptr, 0)) {
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }

    // 1. receive scene text
    MsgType type;
    uint32_t len;
    if (!recv_msg_header(sock, type, len) || type != MsgType::SceneData) {
        std::cerr << "expected scene\n";
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    std::vector<char> scene_buf(len);
    if (!recv_all(sock, scene_buf.data(), len)) {
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    std::string scene_text(scene_buf.data(), len);

    // 2. receive asset files until AssetsDone. write each to a relative
    // path under the worker's working directory so the parser can find
    // them when resolving mesh/texture/skybox references.
    std::string worker_dir = "pencil_worker_assets";
    MKDIR(worker_dir.c_str());
    std::string scene_path = worker_dir + "/scene.pecl";

    int assets_received = 0;
    while (true) {
        if (!recv_msg_header(sock, type, len)) break;
        if (type == MsgType::AssetsDone) break;
        if (type != MsgType::AssetFile) {
            std::cerr << "expected AssetFile or AssetsDone, got type=" << (int)type << "\n";
            CLOSE_SOCKET(sock);
            net_cleanup();
            return 1;
        }
        std::vector<char> af_buf(len);
        if (!recv_all(sock, af_buf.data(), len)) break;
        if (len < sizeof(AssetFileHeader)) break;
        AssetFileHeader hdr;
        std::memcpy(&hdr, af_buf.data(), sizeof(hdr));
        if (len < sizeof(hdr) + hdr.path_len + hdr.data_len) {
            std::cerr << "asset payload truncated\n";
            break;
        }
        std::string rel_path(af_buf.data() + sizeof(hdr), hdr.path_len);
        std::string out_path = worker_dir + "/" + rel_path;
        mkdir_parents(out_path);
        std::ofstream of(out_path, std::ios::binary);
        if (!of) {
            std::cerr << "  warn: cannot write " << out_path << "\n";
        } else {
            of.write(af_buf.data() + sizeof(hdr) + hdr.path_len, hdr.data_len);
            of.close();
        }
        assets_received++;
    }
    if (assets_received > 0) {
        std::cout << "  received " << assets_received << " asset file(s)\n";
    }

    // write the scene .pecl last so the parser can find it + its assets
    {
        std::ofstream sf(scene_path);
        sf << scene_text;
        sf.close();
    }

    // 3. receive RenderSettings
    if (!recv_msg_header(sock, type, len) || type != MsgType::RenderSettings) {
        std::cerr << "expected RenderSettings\n";
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    RenderSettingsMsg rsm;
    if (len < sizeof(rsm)) {
        std::cerr << "RenderSettings payload too small\n";
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    if (!recv_all(sock, &rsm, sizeof(rsm))) {
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    RenderSettings rs;
    rs.samples = (int)rsm.samples;
    rs.max_depth = (int)rsm.max_depth;
    rs.pathtrace_bounces = (int)rsm.pathtrace_bounces;
    rs.soft_shadow_samples = (int)rsm.soft_shadow_samples;
    rs.god_ray_steps = (int)rsm.god_ray_steps;
    rs.caustic_photons = (int)rsm.caustic_photons;
    rs.bokeh_blades = (int)rsm.bokeh_blades;
    rs.tile_size = (int)rsm.tile_size;
    rs.threads = (int)rsm.threads;
    rs.adaptive = rsm.adaptive != 0;
    rs.caustics = rsm.caustics != 0; rs.ppm = rsm.ppm != 0;
    rs.pathtrace = rsm.pathtrace != 0; rs.bdpt = rsm.bdpt != 0; rs.spectral = rsm.spectral != 0; rs.mis = rsm.mis != 0; rs.volume = rsm.volume != 0; rs.sss_enabled = rsm.sss_enabled != 0;
    rs.normals_mode = rsm.normals_mode != 0;
    rs.motion_blur = rsm.motion_blur != 0;
    rs.god_rays = rsm.god_rays != 0;
    rs.bloom = rsm.bloom != 0;
    rs.denoise = rsm.denoise != 0;
    rs.show_progress = false; // never on worker
    rs.adaptive_threshold = rsm.adaptive_threshold;
    rs.shadow_bias = rsm.shadow_bias;

    // 4. parse the scene. chdir to worker_dir first so that mesh/texture
    // paths (which the parser resolves CWD-relative) point at the assets
    // we just wrote. include paths still resolve via _base_dir which
    // starts as the directory of scene_path == worker_dir.
    //
    // We save the original CWD so we can restore it for error messages.
    std::string saved_cwd;
    {
        char buf[4096];
        if (GETCWD(buf, sizeof(buf))) saved_cwd = buf;
    }
    if (CHDIR(worker_dir.c_str()) != 0) {
        std::cerr << "  warn: cannot chdir to " << worker_dir << "\n";
    }
    ParsedScene ps;
    std::vector<ParseError> errs;
    if (!parse_pecl("scene.pecl", ps, errs)) {
        std::cerr << "failed to parse scene\n";
        for (const auto& e : errs) std::cerr << "  line " << e.line << ": " << e.what << "\n";
        if (!saved_cwd.empty()) CHDIR(saved_cwd.c_str());
        CLOSE_SOCKET(sock);
        net_cleanup();
        return 1;
    }
    ps.finalize();
    if (!saved_cwd.empty()) CHDIR(saved_cwd.c_str());
    std::cout << "scene: " << ps.scene.primitive_count() << " prims, "
              << ps.scene.light_count() << " lights\n";

    // v0.0.6 bugfix: build the photon map ONCE here, before the tile
    // loop. The distributed render_tile() calls renderer.trace()
    // directly (bypassing Renderer::render(), which is what builds
    // the photon map in the local path). Without this, distributed
    // renders silently dropped caustics even when --caustics was set
    // on the server -- the renderer's trace() saw caustics_built=false
    // and skipped the photon-map lookup entirely. Building it here
    // once per worker (instead of once per tile) also avoids the
    // O(tiles * caustic_photons) blowup you'd get if you put this
    // inside render_tile.
    // v0.0.7: progressive photon mapping - do ppm_passes rounds.
    if (rs.caustics && !ps.scene.caustics_built) {
        ps.scene.caustics.photons.clear();
        int n_lights = std::max(1, (int)ps.scene.light_count());
        int photons_per_light = rs.caustic_photons / n_lights;
        int passes = rs.ppm ? rs.ppm_passes : 1;
        for (int pass = 0; pass < passes; ++pass) {
            for (const auto& light : ps.scene.lights()) {
                trace_photons_impl(ps.scene, ps.scene.caustics,
                                   light, photons_per_light);
            }
        }
        ps.scene.caustics.build();
        ps.scene.caustics_built = true;
        std::cout << "  built photon map: "
                  << ps.scene.caustics.size() << " photons ("
                  << photons_per_light << "/light across "
                  << n_lights << " light(s))\n";
    }

    // 5. v0.0.6: detect aspect mismatch. the scene's `size: w h` bakes
    // the camera aspect at parse time. if the server tells us to render
    // a different aspect (via TileRequest's full_w/full_h), we re-bake
    // the camera aspect to match the target. this fixes the
    // mesh_dist horizontal stretch bug.
    bool aspect_adjusted = false;

    // render loop: receive tile request, render ONLY that tile, send back
    while (true) {
        if (!recv_msg_header(sock, type, len)) break;

        if (type == MsgType::Done) {
            std::cout << "done\n";
            break;
        }

        if (type != MsgType::TileRequest) break;

        TileRequestMsg req;
        if (!recv_all(sock, &req, sizeof(req))) break;

        // adjust aspect on the FIRST tile we receive
        if (!aspect_adjusted) {
            float target_aspect = (float)req.full_w / (float)req.full_h;
            float scene_aspect = ps.camera.aspect();
            if (scene_aspect <= 0.0f) scene_aspect = (float)ps.width / (float)ps.height;
            const float EPS = 1e-3f;
            if (std::fabs(scene_aspect - target_aspect) > EPS) {
                std::cout << "adjusting camera aspect: " << scene_aspect
                          << " -> " << target_aspect << "\n";
                ps.camera.set_aspect(target_aspect);
            }
            aspect_adjusted = true;
        }

        // render ONLY the tile region (not the full image!)
        std::vector<Color> tile_colors;
        render_tile(ps.scene, ps.camera,
                    req.x, req.y, req.w, req.h,
                    req.full_w, req.full_h,
                    ps.samples, ps.depth, tile_colors,
                    &rs);

        // v0.0.6: send HDR Color (3x float) back to the server, NOT
        // tonemapped 8-bit RGB. the server assembles the full HDR image,
        // applies bloom/denoise, then tonemap+gamma+8-bit in one pass.
        // before this change, the worker tonemapped+8-bit per tile,
        // which (a) skipped the gamma 2.2 step (darker than local),
        // (b) made bloom impossible (it needs HDR), and (c) lost
        // dynamic range for denoise bilateral filtering.
        TileResultHeader hdr = {req.x, req.y, req.w, req.h};
        if (!send_msg(sock, MsgType::TileResult, &hdr, sizeof(hdr))) break;
        if (!send_all(sock, tile_colors.data(), tile_colors.size() * sizeof(Color))) break;
    }

    CLOSE_SOCKET(sock);
    net_cleanup();
    return 0;
}

} // namespace pencil
