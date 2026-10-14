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

// pecl_parser.hpp - Parser for Pencil's .pecl scene format (v2)
#pragma once

#include "animation.hpp"
#include "camera.hpp"
#include "expr.hpp"
#include "light.hpp"
#include "material.hpp"
#include "mesh.hpp"
#include "noise.hpp"
#include "presets.hpp"
#include "primitives.hpp"
#include "scene.hpp"

#include <cctype>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace pencil {

struct ParsedScene {
    Scene  scene;
    Camera camera;

    Point3 eye   = {0.0f, 1.5f, -5.0f};
    Point3 look  = {0.0f, 0.0f, 0.0f};
    Vec3   up    = {0.0f, 1.0f, 0.0f};
    float  fov   = 60.0f;
    float  aspect = 4.0f / 3.0f;
    float  aperture   = 0.0f;  // v0.0.2: DOF aperture
    float  focus_dist = 1.0f;  // v0.0.2: DOF focal distance
    int    bokeh_blades = 0;  // v0.0.5: bokeh aperture shape

    int width   = 400;
    int height  = 300;
    int samples = 4;
    int depth   = 8;
    int fps     = 30;    // v0.0.3: animation FPS
    int frames  = 0;     // v0.0.3: total animation frames (0 = no animation)
    // v0.0.7: render-block feature toggles. each can be set to
    // true/false, 1/0, or a specific value (e.g. cautype: ppm).
    bool enable_caustics = false;       // cautype: rglr/ppm
    bool enable_ppm = false;             // cautype: ppm (sub-type of caustics)
    bool enable_spectral = false;        // spctrl: true/false or 1/0
    bool enable_sss = false;             // sss: true/false or 1/0 (global)
    bool enable_bdpt = false;            // bdpt: bi/uni/true/false
    bool enable_volume = false;          // volume: true/false or 1/0
    bool bdpt_bidirectional = true;       // bdpt: bi (true) or uni (false)
    bool enable_mis = false;             // mis: true/false or 1/0
    bool enable_pathtrace = false;       // pt: true/false or 1/0

    // v0.0.3: animation state. set by the caller before parse_pecl.
    // when > 0, anim { } blocks compute interpolated transforms.
    int _anim_frame = 0;
    Vec3 _anim_scale = {1, 1, 1};
    Vec3 _anim_rot   = {0, 0, 0};
    bool _anim_has_pos   = false;
    bool _anim_has_scale = false;
    bool _anim_has_rot   = false;

    // scratch fields reused while parsing each primitive block
    Vec3   _tmp_v1 = {0,0,0};   // pos / v0 / a / center / apex
    Vec3   _tmp_v2 = {0,0,0};   // v1 / b / normal / surf min / axis
    Vec3   _tmp_v3 = {0,0,0};   // v2 / c / surf max / mesh translate
    Vec3   _tmp_v4 = {0,0,0};   // d
    float  _tmp_f1 = 1.0f;       // rad / size / half_extent / mesh scale
    float  _tmp_f2 = 1.0f;       // cyl/cone height
    float  _tmp_f4 = 0.0f;       // surf step_size
    int    _tmp_i1 = 0;          // surf max_steps / terrain octaves
    int    _tmp_i2 = 32;         // terrain grid resolution
    std::string _pending_surf_func; // surf { func { ... } } body
    std::string _pending_mesh_file; // mesh { file: ... } path
    std::string _pending_include;   // include { file: ... } path
    std::string _pending_seed;      // tern { seed: ... } seed string
    // v0.0.6: physics
    bool  _has_physics = false;
    float _phys_mass = 1.0f;
    float _phys_gravity = 9.81f;
    float _phys_bounce = 0.5f;
    // v0.0.6: initial physics position (from the object's pos)
    Vec3  _phys_initial_pos = {0, 0, 0};
    Vec3  _phys_velocity = {0, 0, 0};
    std::string _base_dir;          // directory of the parent scene file (for includes)
    bool _skip_finalize = false;    // when true, parse_pecl won't call finalize (used by includes)
    Material _current_mat = make_diffuse({0.7f, 0.7f, 0.7f}, 0.0f, 32.0f);

    // v0.0.6: group transform stack.
    // when inside a grp { } block, all child primitives get offset
    // by the accumulated group transforms.
    struct GroupTransform {
        Vec3 offset = {0, 0, 0};
        Vec3 scale = {1, 1, 1};
        bool active = false;
    };
    std::vector<GroupTransform> group_stack;

    // compute total offset from all active group transforms
    Vec3 group_offset() const {
        Vec3 total = {0, 0, 0};
        for (const auto& g : group_stack) {
            if (g.active) total += g.offset;
        }
        return total;
    }
    float group_scale() const {
        float s = 1.0f;
        for (const auto& g : group_stack) {
            if (g.active) s *= g.scale.x;
        }
        return s;
    }
    bool in_group() const {
        for (const auto& g : group_stack) {
            if (g.active) return true;
        }
        return false;
    }

    void finalize() {
        if (focus_dist <= 0.0f) {
            // default to distance from eye to look
            focus_dist = (eye - look).length();
        }
        camera = Camera(eye, look, up, fov, aspect, aperture, focus_dist);
        camera.set_bokeh_blades(bokeh_blades);
        scene.build_bvh();
    }
};

struct ParseError {
    int         line = 0;
    std::string what;
};

// --- Tiny tokenizer --------------------------------------------------------- ...

enum class TokKind {
    LBrace, RBrace, Colon, Comma,
    AngleOpen, AngleClose,
    Ident, Number, End,
};

struct Token {
    TokKind    kind = TokKind::End;
    std::string text;
    int        line = 0;
};

class Lexer {
public:
    Lexer(const std::string& src) : src_(src), pos_(0), line_(1) {}

    Token next() {
        skip_ws_and_comments();
        if (pos_ >= src_.size()) return {TokKind::End, "", line_};

        char c = src_[pos_];

        if (c == '{') { pos_++; return {TokKind::LBrace, "{", line_}; }
        if (c == '}') { pos_++; return {TokKind::RBrace, "}", line_}; }
        if (c == ':') { pos_++; return {TokKind::Colon,  ":", line_}; }
        if (c == ',') { pos_++; return {TokKind::Comma,  ",", line_}; }
        if (c == '<') { pos_++; return {TokKind::AngleOpen, "<", line_}; }
        if (c == '>') { pos_++; return {TokKind::AngleClose,">", line_}; }

        // numbers: digits, leading '-', optional '.'
        if (std::isdigit((unsigned char)c) || c == '.' ||
            (c == '-' && pos_ + 1 < src_.size() &&
             (std::isdigit((unsigned char)src_[pos_+1]) || src_[pos_+1] == '.'))) {
            return read_number();
        }

        // identifiers: letters, digits, underscore
        if (std::isalpha((unsigned char)c) || c == '_') {
            return read_ident();
        }

        // unknown character; emit as identifier so caller can complain
        pos_++;
        return {TokKind::Ident, std::string(1, c), line_};
    }

    // grab everything between matching braces, used for `func {
    std::string read_braced() {
        skip_ws_and_comments();
        std::string out;
        if (pos_ >= src_.size() || src_[pos_] != '{') return out;
        pos_++; // consume '{'
        int depth = 1;
        while (pos_ < src_.size() && depth > 0) {
            char c = src_[pos_++];
            if (c == '{') depth++;
            else if (c == '}') { depth--; if (depth == 0) break; }
            else if (c == '\n') line_++;
            out.push_back(c);
        }
        return out;
    }

    int line() const { return line_; }

    // v0.0.4: save/restore for peek-ahead parsing.
    size_t peek_pos() const { return pos_; }
    void restore_pos(size_t p) { pos_ = p; }

    // read a path-like string: either a quoted string ("...") or raw chars until ne...
    std::string read_path() {
        skip_ws_and_comments();
        if (pos_ >= src_.size()) return "";
        // skip optional opening quote
        bool quoted = false;
        if (src_[pos_] == '"') { quoted = true; pos_++; }
        std::string out;
        while (pos_ < src_.size()) {
            char c = src_[pos_];
            if (quoted && c == '"') { pos_++; break; }
            if (!quoted && (c == '\n' || c == ',' || c == '}')) break;
            if (c == '\r') { pos_++; continue; }
            out.push_back(c);
            pos_++;
        }
        // trim trailing whitespace
        while (!out.empty() && (out.back() == ' ' || out.back() == '\t')) out.pop_back();
        return out;
    }

private:
    const std::string& src_;
    size_t pos_;
    int    line_;

    void skip_ws_and_comments() {
        while (pos_ < src_.size()) {
            char c = src_[pos_];
            if (c == ' ' || c == '\t' || c == '\r') { pos_++; }
            else if (c == '\n') { pos_++; line_++; }
            else if (c == '#') {
                while (pos_ < src_.size() && src_[pos_] != '\n') pos_++;
            } else break;
        }
    }

    Token read_number() {
        size_t start = pos_;
        if (src_[pos_] == '-') pos_++;
        bool seenDot = false, seenExp = false;
        while (pos_ < src_.size()) {
            char c = src_[pos_];
            if (std::isdigit((unsigned char)c)) { pos_++; }
            else if (c == '.' && !seenDot) { seenDot = true; pos_++; }
            else if ((c == 'e' || c == 'E') && !seenExp) {
                seenExp = true; pos_++;
                if (pos_ < src_.size() && (src_[pos_] == '+' || src_[pos_] == '-')) pos_++;
            } else break;
        }
        return {TokKind::Number, src_.substr(start, pos_ - start), line_};
    }

    Token read_ident() {
        size_t start = pos_;
        while (pos_ < src_.size() &&
               (std::isalnum((unsigned char)src_[pos_]) || src_[pos_] == '_')) {
            pos_++;
        }
        return {TokKind::Ident, src_.substr(start, pos_ - start), line_};
    }
};

// helper: parse "<x, y, z>" into a Vec3
inline bool parse_vec3(Lexer& lx, Vec3& out, ParseError& err) {
    Token t = lx.next();
    if (t.kind != TokKind::AngleOpen) {
        err = {t.line, "expected '<' for vector, got '" + t.text + "'"};
        return false;
    }
    float v[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        Token num = lx.next();
        if (num.kind != TokKind::Number) {
            err = {num.line, "expected number in vector"};
            return false;
        }
        v[i] = std::stof(num.text);
        if (i < 2) {
            Token sep = lx.next();
            if (sep.kind != TokKind::Comma) {
                // allow either comma or no separator; if it's not a comma, we have to back up s...
                if (sep.kind != TokKind::Number) {
                    err = {sep.line, "expected ',' between vector components"};
                    return false;
                }
                // it was actually the next number; put it back as v[i+1]
                v[++i] = std::stof(sep.text);
                if (i < 3) {
                    Token s2 = lx.next();
                    if (s2.kind != TokKind::Comma && s2.kind != TokKind::Number) {
                        err = {s2.line, "expected ',' or number in vector"};
                        return false;
                    }
                    if (s2.kind == TokKind::Number && i == 2) {
                        // already at 3 components, but got another, back up
                    }
                }
            }
        }
    }
    Token close = lx.next();
    if (close.kind != TokKind::AngleClose) {
        err = {close.line, "expected '>' to close vector"};
        return false;
    }
    out = {v[0], v[1], v[2]};
    return true;
}

// vec3 parser used in production paths
inline bool parse_vec3_strict(Lexer& lx, Vec3& out, ParseError& err) {
    Token t = lx.next();
    if (t.kind != TokKind::AngleOpen) {
        err = {t.line, "expected '<' for vector, got '" + t.text + "'"};
        return false;
    }
    float v[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
        Token num = lx.next();
        if (num.kind != TokKind::Number) {
            err = {num.line, "expected number in vector"};
            return false;
        }
        v[i] = std::stof(num.text);
        if (i < 2) {
            Token sep = lx.next();
            if (sep.kind != TokKind::Comma) {
                err = {sep.line, "expected ',' between vector components"};
                return false;
            }
        }
    }
    Token close = lx.next();
    if (close.kind != TokKind::AngleClose) {
        err = {close.line, "expected '>' to close vector"};
        return false;
    }
    out = {v[0], v[1], v[2]};
    return true;
}

// read a single number
inline bool parse_scalar(Lexer& lx, float& out, ParseError& err) {
    Token t = lx.next();
    if (t.kind != TokKind::Number) {
        err = {t.line, "expected number, got '" + t.text + "'"};
        return false;
    }
    out = std::stof(t.text);
    return true;
}

// read two numbers on one line: "640 480"
inline bool parse_size2(Lexer& lx, int& a, int& b, ParseError& err) {
    Token t1 = lx.next();
    if (t1.kind != TokKind::Number) {
        err = {t1.line, "expected two numbers, got '" + t1.text + "'"};
        return false;
    }
    Token t2 = lx.next();
    if (t2.kind != TokKind::Number) {
        err = {t2.line, "expected second number, got '" + t2.text + "'"};
        return false;
    }
    a = (int)std::stof(t1.text);
    b = (int)std::stof(t2.text);
    return true;
}

// read an identifier
inline bool parse_ident(Lexer& lx, std::string& out, ParseError& err) {
    Token t = lx.next();
    if (t.kind != TokKind::Ident) {
        err = {t.line, "expected identifier, got '" + t.text + "'"};
        return false;
    }
    out = t.text;
    return true;
}

// convert a 0-100 color literal into a 0-1 color
inline Color color_from_100(const Vec3& v) {
    return { clampf(v.x, 0.0f, 100.0f) / 100.0f,
             clampf(v.y, 0.0f, 100.0f) / 100.0f,
             clampf(v.z, 0.0f, 100.0f) / 100.0f };
}

// v0.0.5: emission can exceed 1.0 (HDR) for bright light sources.
// regular colors are clamped to [0,1], but emission is NOT clamped
// so that emissive surfaces can be brighter than 1.0.
// this is critical for god rays, bloom, and tone mapping to work.
inline Color emission_from_100(const Vec3& v) {
    return { v.x / 100.0f,
             v.y / 100.0f,
             v.z / 100.0f };
}

// material overrides: applies a known material property key
struct MaterialPatch {
    bool   has_clr  = false; Color clr;
    bool   has_emt  = false; Color emt;
    bool   has_rfl  = false; float rfl = 0;
    bool   has_rfr  = false; float rfr = 0; // 0-100, mapped to ior
    bool   has_ior  = false; float ior = 0;
    bool   has_spc  = false; float spc = 0;
    bool   has_shn  = false; float shn = 0;
    bool   has_fuz  = false; float fuz = 0;
    bool   has_typ  = false; std::string typ;
    bool   has_tex  = false; std::string tex_path;
    bool   has_refdis = false; int refdis = 0;
    bool   has_shdw  = false; std::string shdw;
    bool   has_radius = false; float radius = 0;
    // v0.0.4: procedural texture params
    std::string proc_type = "checker";
    float proc_scale = 4.0f;
    Color proc_c1 = {1, 1, 1};
    Color proc_c2 = {0, 0, 0};
    // v0.0.4: normal map
    bool has_nrml = false;
    bool has_prst = false;
    std::string prst_name;
    std::string nrml_path;
    // v0.0.5: subsurface scattering
    bool has_sss = false;
    float sss_amount = 0.0f;
    float sss_radius = 0.5f;
    Color sss_color = {1.0f, 0.5f, 0.3f};
    // v0.0.7: true displacement map
    bool has_disp = false;
    std::string disp_path;
    float disp_amount = 0.1f;
    std::string disp_proc; // "noise", "marble", "wood", "checker"
    // v0.0.7: principled bsdf parameters
    bool has_prst_bsdf = false;
    float metallic = 0.0f;
    float roughness = 0.5f;
    float specular_amt = 0.5f;
    float specular_tint = 0.0f;
    float sheen = 0.0f;
    float sheen_tint = 0.5f;
    float clearcoat = 0.0f;
    float clearcoat_gloss = 1.0f;
    float transmission = 0.0f;
};

// v0.0.4: helper to create a texture from a MaterialPatch.
// handles both file paths and procedural textures (path == "__proc__").
inline TexturePtr make_texture_from_patch(const MaterialPatch& patch) {
    auto tex = std::make_shared<Texture>();
    if (patch.tex_path == "__proc__") {
        ProcType pt = ProcType::Checker;
        if (patch.proc_type == "noise")       pt = ProcType::Noise;
        else if (patch.proc_type == "marble") pt = ProcType::Marble;
        else if (patch.proc_type == "wood")   pt = ProcType::Wood;
        tex->set_procedural(pt, patch.proc_scale, patch.proc_c1, patch.proc_c2);
    } else {
        std::string terr;
        if (!tex->load(patch.tex_path, terr)) {
            return nullptr;
        }
    }
    return tex;
}

// try to parse a material property
inline bool try_parse_material_key(const std::string& key, Lexer& lx,
                                    MaterialPatch& patch, ParseError& err) {
    if (key == "clr" || key == "color") {
        Vec3 v; if (!parse_vec3_strict(lx, v, err)) return false;
        patch.has_clr = true; patch.clr = color_from_100(v);
        return true;
    }
    if (key == "emt" || key == "emission") {
        Vec3 v; if (!parse_vec3_strict(lx, v, err)) return false;
        patch.has_emt = true; patch.emt = emission_from_100(v);
        return true;
    }
    if (key == "rfl" || key == "reflectivity" || key == "reflection") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_rfl = true; patch.rfl = clampf(f, 0.0f, 100.0f) / 100.0f;
        return true;
    }
    if (key == "rfr") {
        // rfr in 0-100 maps to a glassness; treat 100 as ior=1.5
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_rfr = true; patch.rfr = clampf(f, 0.0f, 100.0f) / 100.0f;
        return true;
    }
    if (key == "ior") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_ior = true; patch.ior = f;
        return true;
    }
    if (key == "spc" || key == "specular") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_spc = true; patch.spc = clampf(f, 0.0f, 100.0f) / 100.0f;
        return true;
    }
    if (key == "shn" || key == "shininess") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        // shininess exponent: keep raw value (Phong exponents are 1..256+)
        patch.has_shn = true; patch.shn = f;
        return true;
    }
    if (key == "fuz" || key == "fuzz") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_fuz = true; patch.fuz = clampf(f, 0.0f, 100.0f) / 100.0f;
        return true;
    }
    if (key == "typ" || key == "type") {
        std::string s; if (!parse_ident(lx, s, err)) return false;
        patch.has_typ = true; patch.typ = s;
        return true;
    }
    if (key == "tex" || key == "texture") {
        // v0.0.4: check if next token is "procedural" keyword.
        // otherwise treat as file path.
        // peek at next token by saving/restoring pos.
        size_t saved_pos = lx.peek_pos();
        Token next = lx.next();
        if (next.kind == TokKind::Ident && next.text == "procedural") {
            // expect { type: ..., scale: ..., c1: ..., c2: ... }
            Token lb = lx.next();
            if (lb.kind != TokKind::LBrace) {
                err = {lb.line, "expected '{' after procedural"};
                return false;
            }
            std::string ptype = "checker";
            float pscale = 4.0f;
            Vec3 pc1 = {100, 100, 100}, pc2 = {0, 0, 0};
            while (true) {
                Token pk = lx.next();
                if (pk.kind == TokKind::RBrace) break;
                if (pk.kind == TokKind::End) break;
                if (pk.kind == TokKind::Comma) continue;
                Token pcol = lx.next();
                if (pcol.kind != TokKind::Colon) {
                    err = {pcol.line, "expected ':' in procedural"};
                    return false;
                }
                ParseError pe;
                if (pk.text == "type" || pk.text == "kind") {
                    if (!parse_ident(lx, ptype, pe)) return false;
                } else if (pk.text == "scale" || pk.text == "s") {
                    if (!parse_scalar(lx, pscale, pe)) return false;
                } else if (pk.text == "c1" || pk.text == "color1") {
                    if (!parse_vec3_strict(lx, pc1, pe)) return false;
                } else if (pk.text == "c2" || pk.text == "color2") {
                    if (!parse_vec3_strict(lx, pc2, pe)) return false;
                }
            }
            // store as a special path prefix that the block action recognizes
            patch.has_tex = true;
            patch.tex_path = "__proc__";
            patch.proc_type = ptype;
            patch.proc_scale = pscale;
            patch.proc_c1 = color_from_100(pc1);
            patch.proc_c2 = color_from_100(pc2);
            return true;
        }
        // not procedural - restore pos and read as path
        lx.restore_pos(saved_pos);
        std::string s = lx.read_path();
        if (s.empty()) {
            err = {0, "expected path after tex"};
            return false;
        }
        patch.has_tex = true; patch.tex_path = s;
        return true;
    }
    if (key == "refdis" || key == "dispersion") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_refdis = true; patch.refdis = (int)f;
        return true;
    }
    if (key == "prst" || key == "preset") {
        std::string s; if (!parse_ident(lx, s, err)) return false;
        patch.has_prst = true; patch.prst_name = s;
        return true;
    }
    if (key == "nrml" || key == "normal" || key == "normal_map" || key == "bump") {
        // v0.0.4: normal map texture path
        std::string s = lx.read_path();
        if (s.empty()) {
            err = {0, "expected path after nrml"};
            return false;
        }
        patch.has_nrml = true; patch.nrml_path = s;
        return true;
    }
    // v0.0.5: subsurface scattering
    if (key == "sss") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_sss = true; patch.sss_amount = clampf(f, 0.0f, 100.0f) / 100.0f;
        return true;
    }
    if (key == "sss_r" || key == "sss_radius") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_sss = true; patch.sss_radius = f;
        return true;
    }
    if (key == "sss_c" || key == "sss_color") {
        Vec3 v; if (!parse_vec3_strict(lx, v, err)) return false;
        patch.has_sss = true; patch.sss_color = color_from_100(v);
        return true;
    }
    if (key == "shdw" || key == "shadow") {
        std::string s; if (!parse_ident(lx, s, err)) return false;
        patch.has_shdw = true; patch.shdw = s;
        return true;
    }
    // v0.0.7: principled bsdf parameters
    if (key == "metallic" || key == "metal") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.metallic = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "rough" || key == "roughness") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.roughness = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "spec_amt" || key == "specular") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.specular_amt = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "spec_tint" || key == "specular_tint") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.specular_tint = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "sheen") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.sheen = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "sheen_tint") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.sheen_tint = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "clearcoat" || key == "cc") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.clearcoat = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "cc_gloss" || key == "clearcoat_gloss") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.clearcoat_gloss = clampf(f, 0.0f, 1.0f);
        return true;
    }
    if (key == "transmission" || key == "transmit") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_prst_bsdf = true; patch.transmission = clampf(f, 0.0f, 1.0f);
        return true;
    }
    // v0.0.7: true displacement map. disp: path to a height texture.
    // disp_amt: max displacement in world units (default 0.1).
    // disp_proc: procedural type (noise/marble/wood/checker) instead of file.
    if (key == "disp" || key == "displacement" || key == "disp_map") {
        std::string s = lx.read_path();
        if (s.empty()) { err = {0, "expected path after disp"}; return false; }
        patch.has_disp = true; patch.disp_path = s;
        return true;
    }
    if (key == "disp_amt" || key == "disp_amount" || key == "disp_scale") {
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_disp = true; patch.disp_amount = f;
        return true;
    }
    if (key == "disp_proc") {
        std::string s = lx.read_path();
        if (s.empty()) { err = {0, "expected procedural type after disp_proc"}; return false; }
        patch.has_disp = true; patch.disp_proc = s;
        return true;
    }
    if (key == "radius" || key == "rad_l") {
        // for lights: soft shadow radius
        float f; if (!parse_scalar(lx, f, err)) return false;
        patch.has_radius = true; patch.radius = f;
        return true;
    }
    return false;
}

// apply a MaterialPatch onto a base material, returning the new material
inline Material apply_patch(const Material& base, const MaterialPatch& p) {
    Material m = base;
    if (p.has_typ) {
        if (p.typ == "metal")      m.kind = MaterialKind::Metal;
        else if (p.typ == "glass" ||
                 p.typ == "dielectric") m.kind = MaterialKind::Dielectric;
        else if (p.typ == "principled" ||
                 p.typ == "pbr")       m.kind = MaterialKind::Principled;
        else if (p.typ == "emissive" ||
                 p.typ == "emit")      m.kind = MaterialKind::Diffuse; // emission via emt
        else                          m.kind = MaterialKind::Diffuse;
    }
    // v0.0.7: principled bsdf. if any principled parameter is set,
    // switch material kind to principled.
    if (p.has_prst_bsdf) {
        m.kind = MaterialKind::Principled;
        m.metallic = p.metallic;
        m.roughness = p.roughness;
        m.specular_amt = p.specular_amt;
        m.specular_tint = p.specular_tint;
        m.sheen = p.sheen;
        m.sheen_tint = p.sheen_tint;
        m.clearcoat = p.clearcoat;
        m.clearcoat_gloss = p.clearcoat_gloss;
        m.transmission = p.transmission;
    }
    if (p.has_clr) m.albedo = p.clr;
    if (p.has_emt) m.emission = p.emt;
    if (p.has_rfl) {
        // rfl 0-100 controls reflectivity
        if (p.rfl > 0.0f && m.kind == MaterialKind::Diffuse) {
            m.kind = MaterialKind::Metal;
        }
        m.reflectivity = p.rfl;
    }
    if (p.has_rfr) {
        // rfr > 0 means dielectric glass
        if (p.rfr > 0.0f) {
            m.kind = MaterialKind::Dielectric;
            if (!p.has_ior) m.ior = 1.5f;
        }
    }
    if (p.has_ior) {
        m.kind = MaterialKind::Dielectric;
        m.ior = p.ior;
    }
    if (p.has_spc) m.specular = p.spc;
    if (p.has_shn) m.shininess = p.shn;
    if (p.has_fuz) {
        if (m.kind != MaterialKind::Metal) m.kind = MaterialKind::Metal;
        m.reflectivity = 1.0f - p.fuz;
    }
    if (p.has_refdis) m.refdis = p.refdis;
    if (p.has_shdw) {
        if (p.shdw == "soft") m.shadow = ShadowMode::Soft;
        else if (p.shdw == "hard") m.shadow = ShadowMode::Hard;
    }
    // v0.0.5: SSS
    if (p.has_sss) {
        m.sss = p.sss_amount;
        m.sss_radius = p.sss_radius;
        m.sss_color = p.sss_color;
    }
    if (p.has_tex) {
        // load texture lazily; the patch just stores the path.
        // the scene-application code below will load and attach it.
        // we mark this for the caller by setting a sentinel.
        // (texture load is deferred to the calling site.)
    }
    return m;
}

// parse a .pecl file into a ParsedScene
inline bool parse_pecl(const std::string& path, ParsedScene& out,
                       std::vector<ParseError>& errors) {
    std::ifstream f(path);
    if (!f) {
        errors.push_back({0, "Cannot open scene file: " + path});
        return false;
    }
    std::string src((std::istreambuf_iterator<char>(f)),
                     std::istreambuf_iterator<char>());
    f.close();

    // set base dir for relative include resolution
    size_t slash = path.find_last_of('/');
    if (slash != std::string::npos) out._base_dir = path.substr(0, slash);

    Lexer lx(src);
    bool ok = true;

    // the "current" material starts as a default diffuse gray
    // out._current_mat now lives on ParsedScene as out._current_mat

    while (true) {
        Token t = lx.next();
        if (t.kind == TokKind::End) break;

        if (t.kind != TokKind::Ident) {
            errors.push_back({t.line, "unexpected token '" + t.text + "'"});
            // skip to next reasonable boundary and continue
            while (t.kind != TokKind::End && t.kind != TokKind::RBrace) t = lx.next();
            ok = false;
            continue;
        }

        const std::string& block_name = t.text;

        // top-level scalars (no braces)
        if (block_name == "bg" || block_name == "background") {
            Token colon = lx.next();
            if (colon.kind != TokKind::Colon) {
                errors.push_back({colon.line, "expected ':' after bg"});
                ok = false; continue;
            }
            Vec3 v; ParseError e;
            if (parse_vec3_strict(lx, v, e)) {
                out.scene.background = color_from_100(v);
            } else errors.push_back(e);
            continue;
        }
        if (block_name == "amb" || block_name == "ambient") {
            Token colon = lx.next();
            if (colon.kind != TokKind::Colon) {
                errors.push_back({colon.line, "expected ':' after amb"});
                ok = false; continue;
            }
            Vec3 v; ParseError e;
            if (parse_vec3_strict(lx, v, e)) {
                out.scene.ambient = color_from_100(v);
            } else errors.push_back(e);
            continue;
        }

        // all other top-level directives are blocks: name {
        Token lb = lx.next();
        if (lb.kind != TokKind::LBrace) {
            errors.push_back({lb.line, "expected '{' after " + block_name});
            ok = false; continue;
        }

        // parse key/value pairs inside the block
        MaterialPatch patch; // captured for primitives that need material overrides

        while (true) {
            Token kt = lx.next();
            if (kt.kind == TokKind::RBrace) break;
            if (kt.kind == TokKind::End) {
                errors.push_back({kt.line, "unexpected end of file inside " + block_name});
                ok = false; break;
            }
            // tolerate stray commas between key/value pairs (people like to write `pos: <0,...
            if (kt.kind == TokKind::Comma) continue;
            if (kt.kind != TokKind::Ident) {
                errors.push_back({kt.line, "expected key, got '" + kt.text + "'"});
                ok = false;
                // skip to next '}' or end
                while (kt.kind != TokKind::End && kt.kind != TokKind::RBrace) kt = lx.next();
                if (kt.kind == TokKind::RBrace) break;
                continue;
            }

            std::string key = kt.text;

            // special-case `func {
            if (key == "func" || key == "function") {
                // skip optional colon
                Token maybe_colon = lx.next();
                if (maybe_colon.kind == TokKind::Colon) {
                    // func: <expression to end of line> Read until newline
                    std::string expr_src;
                    while (maybe_colon.line == lx.line()) {
                        // peek by reading tokens
                        break;
                    }
                    // read the rest of the line by re-reading from source
                    errors.push_back({kt.line, "func: requires { } form, e.g. func { x*x + y*y - 1 }"});
                    ok = false;
                } else if (maybe_colon.kind == TokKind::LBrace) {
                    // we've consumed '{'; read until matching '}'
                    std::string expr_src;
                    int depth = 1;
                    while (true) {
                        Token inner = lx.next();
                        if (inner.kind == TokKind::End) { errors.push_back({inner.line, "unterminated func { }"}); ok = false; break; }
                        if (inner.kind == TokKind::LBrace) { depth++; expr_src += "{ "; continue; }
                        if (inner.kind == TokKind::RBrace) {
                            depth--;
                            if (depth == 0) break;
                            expr_src += "} ";
                            continue;
                        }
                        if (!expr_src.empty() && expr_src.back() != ' ')
                            expr_src += " ";
                        expr_src += inner.text;
                    }
                    // stash the expression on the patch using clr's slot (we'll need a side channel)
                    if (block_name == "surf") {
                        // we'll attach it via a special key on out
                        out._pending_surf_func = expr_src;
                    }
                } else {
                    errors.push_back({maybe_colon.line, "expected '{' or ':' after func"});
                    ok = false;
                }
                continue;
            }

            // special-case `anim { kf { ... } }` for keyframe animation.
            // when frame > 0, the interpolated transform overrides the
            // parent object's position/scale.
            if (key == "anim" || key == "animation") {
                Token lb = lx.next();
                if (lb.kind != TokKind::LBrace) {
                    errors.push_back({lb.line, "expected '{' after anim"});
                    ok = false; continue;
                }
                std::vector<Keyframe> kfs;
                while (true) {
                    Token kt2 = lx.next();
                    if (kt2.kind == TokKind::RBrace) break;
                    if (kt2.kind == TokKind::End) break;
                    if (kt2.kind == TokKind::Comma) continue;
                    if (kt2.text != "kf") {
                        errors.push_back({kt2.line, "expected 'kf' in anim, got '" + kt2.text + "'"});
                        while (kt2.kind != TokKind::End && kt2.kind != TokKind::RBrace) kt2 = lx.next();
                        if (kt2.kind == TokKind::RBrace) break;
                        continue;
                    }
                    Token klb = lx.next();
                    if (klb.kind != TokKind::LBrace) {
                        errors.push_back({klb.line, "expected '{' after kf"});
                        ok = false; continue;
                    }
                    Keyframe kf;
                    while (true) {
                        Token kkt = lx.next();
                        if (kkt.kind == TokKind::RBrace) break;
                        if (kkt.kind == TokKind::End) break;
                        if (kkt.kind == TokKind::Comma) continue;
                        Token kcolon = lx.next();
                        if (kcolon.kind != TokKind::Colon) {
                            errors.push_back({kcolon.line, "expected ':' in kf"});
                            ok = false; break;
                        }
                        ParseError ke;
                        if (kkt.text == "f") {
                            float f; if (parse_scalar(lx, f, ke)) { kf.frame = (int)f; }
                        }
                        else if (kkt.text == "p" || kkt.text == "pos") {
                            if (parse_vec3_strict(lx, kf.pos, ke)) kf.has_pos = true;
                        }
                        else if (kkt.text == "r" || kkt.text == "rot") {
                            if (parse_vec3_strict(lx, kf.rot, ke)) kf.has_rot = true;
                        }
                        else if (kkt.text == "s" || kkt.text == "scale") {
                            if (parse_vec3_strict(lx, kf.scale, ke)) kf.has_scale = true;
                        }
                        else if (kkt.text == "e" || kkt.text == "ease") {
                            std::string s; if (parse_ident(lx, s, ke)) {
                                kf.ease = parse_ease(s);
                            }
                        }
                    }
                    kfs.push_back(kf);
                }
                // if we have a frame number, compute the interpolated transform
                if (out._anim_frame > 0 && !kfs.empty()) {
                    TransformSample ts = sample_animation(kfs, out._anim_frame);
                    // apply to scratch fields (overriding what was parsed before)
                    if (ts.has_pos) {
                        out._tmp_v1 = ts.pos;
                        out._anim_has_pos = true;
                    }
                    if (ts.has_scale) {
                        out._anim_scale = ts.scale;
                        out._anim_has_scale = true;
                    }
                    if (ts.has_rot) {
                        out._anim_rot = ts.rot;
                        out._anim_has_rot = true;
                    }
                }
                continue;
            }

            // v0.0.6: physics sub-block (phys { typ: rigid, mass: 1.0, grv: 9.81, bnce: 80 })
            if (key == "phys" || key == "physics") {
                Token lb = lx.next();
                if (lb.kind != TokKind::LBrace) {
                    errors.push_back({lb.line, "expected '{' after phys"});
                    ok = false; continue;
                }
                // parse physics properties
                while (true) {
                    Token pkt = lx.next();
                    if (pkt.kind == TokKind::RBrace) break;
                    if (pkt.kind == TokKind::End) break;
                    if (pkt.kind == TokKind::Comma) continue;
                    Token pcolon = lx.next();
                    if (pcolon.kind != TokKind::Colon) {
                        errors.push_back({pcolon.line, "expected ':' in phys"});
                        ok = false; break;
                    }
                    ParseError pe;
                    if (pkt.text == "typ" || pkt.text == "type") {
                        std::string s; parse_ident(lx, s, pe);
                    }
                    else if (pkt.text == "mass") {
                        float f; parse_scalar(lx, f, pe);
                        out._phys_mass = f;
                    }
                    else if (pkt.text == "grv" || pkt.text == "gravity") {
                        float f; parse_scalar(lx, f, pe);
                        out._phys_gravity = f;
                    }
                    else if (pkt.text == "bnce" || pkt.text == "bounce") {
                        float f; parse_scalar(lx, f, pe);
                        out._phys_bounce = clampf(f, 0.0f, 100.0f) / 100.0f;
                    }
                    out._has_physics = true;
                }
                continue;
            }

            // for all other keys, expect ':' next
            Token colon = lx.next();
            if (colon.kind != TokKind::Colon) {
                errors.push_back({colon.line, "expected ':' after " + key + " in " + block_name + ", got '" + colon.text + "'"});
                ok = false; continue;
            }

            ParseError e;
            bool consumed = false;

            // v0.0.5 fix: for fog/sky/include blocks, skip try_parse_material_key
            // because it would intercept "clr" and store it in the material
            // patch instead of the fog/sky color. for all other blocks,
            // material keys are checked first (inline overrides on primitives).
            bool is_fog_or_sky = (block_name == "fog" || block_name == "sky" ||
                                   block_name == "skybox" || block_name == "env" ||
                                   block_name == "include" ||
                                   block_name == "tern" || block_name == "terrain" ||
                                   block_name == "gsky" || block_name == "gen_sky" ||
                                   block_name == "grp" || block_name == "group" ||
                                   block_name == "render");

            if (!is_fog_or_sky && try_parse_material_key(key, lx, patch, e)) {
                consumed = true;
            }
            // block-specific keys
            if (!consumed && block_name == "cmra") {
                if      (key == "pos")  consumed = parse_vec3_strict(lx, out.eye, e);
                else if (key == "aim" || key == "look" || key == "at")
                        consumed = parse_vec3_strict(lx, out.look, e);
                else if (key == "up")   consumed = parse_vec3_strict(lx, out.up, e);
                else if (key == "fov")  consumed = parse_scalar(lx, out.fov, e);
                else if (key == "aspect") consumed = parse_scalar(lx, out.aspect, e);
                else if (key == "aperture" || key == "ap") {
                    float f; if (parse_scalar(lx, f, e)) { out.aperture = f; consumed = true; }
                }
                else if (key == "focus" || key == "focal" || key == "fdist") {
                    float f; if (parse_scalar(lx, f, e)) { out.focus_dist = f; consumed = true; }
                }
                else if (key == "bd" || key == "bokeh" || key == "blades") {
                    float f; if (parse_scalar(lx, f, e)) { out.bokeh_blades = (int)f; consumed = true; }
                }
            }
            if (!consumed && block_name == "mat") {
                // all keys are material keys; already handled above
                if (!consumed) {
                    // unknown key inside mat block
                    errors.push_back({kt.line, "unknown material key: " + key});
                    ok = false;
                    // skip value
                    Token skip = lx.next();
                }
                continue;
            }
            if (!consumed && block_name == "sphere") {
                if      (key == "pos")  consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "rad")  consumed = parse_scalar(lx, out._tmp_f1, e);
            }
            if (!consumed && block_name == "cube") {
                if      (key == "pos")  consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "size") consumed = parse_scalar(lx, out._tmp_f1, e);
            }
            if (!consumed && block_name == "plane") {
                if      (key == "pos")  consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "nrm" || key == "normal" || key == "dir")
                        consumed = parse_vec3_strict(lx, out._tmp_v2, e);
            }
            if (!consumed && (block_name == "tri" || block_name == "triangle")) {
                if      (key == "v0") consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "v1") consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "v2") consumed = parse_vec3_strict(lx, out._tmp_v3, e);
            }
            if (!consumed && block_name == "quad") {
                if      (key == "a") consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "b") consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "c") consumed = parse_vec3_strict(lx, out._tmp_v3, e);
                else if (key == "d") consumed = parse_vec3_strict(lx, out._tmp_v4, e);
            }
            if (!consumed && (block_name == "cyl" || block_name == "cylinder")) {
                // pos: center, ax/axis: axis direction, rad: radius, h/height: total length
                if      (key == "pos" || key == "center") consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "ax"  || key == "axis")   consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "rad" || key == "radius") consumed = parse_scalar(lx, out._tmp_f1, e);
                else if (key == "h"   || key == "height") consumed = parse_scalar(lx, out._tmp_f2, e);
            }
            if (!consumed && block_name == "cone") {
                // apex: tip position, ax/axis: axis direction, rad: base radius, h/height: tota...
                if      (key == "apex")                    consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "ax"  || key == "axis")    consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "rad" || key == "radius")  consumed = parse_scalar(lx, out._tmp_f1, e);
                else if (key == "h"   || key == "height")  consumed = parse_scalar(lx, out._tmp_f2, e);
            }
            if (!consumed && block_name == "mesh") {
                // file: path to obj, scale: target longest-axis size, pos: translate
                if      (key == "file" || key == "path" || key == "obj") {
                    out._pending_mesh_file = lx.read_path();
                    consumed = true;
                }
                else if (key == "scale" || key == "fit") { float f; if (parse_scalar(lx, f, e)) { out._tmp_f1 = f; consumed = true; } }
                else if (key == "pos"   || key == "translate")  consumed = parse_vec3_strict(lx, out._tmp_v3, e);
            }
            if (!consumed && (block_name == "sky" || block_name == "skybox" || block_name == "env")) {
                if (key == "file" || key == "path" || key == "tex" || key == "texture") {
                    out._pending_mesh_file = lx.read_path();  // reuse field as scratch
                    consumed = true;
                }
            }
            if (!consumed && block_name == "include") {
                if (key == "file" || key == "path") {
                    out._pending_include = lx.read_path();
                    consumed = true;
                }
            }
            if (!consumed && block_name == "fog") {
                // v0.0.4: volumetric fog
                // v0.0.5: gradient fog (color_top, grad_min, grad_max)
                if (key == "density") {
                    float f; if (parse_scalar(lx, f, e)) { out.scene.fog.density = f; consumed = true; }
                }
                else if (key == "clr" || key == "color") {
                    Vec3 v; if (parse_vec3_strict(lx, v, e)) {
                        out.scene.fog.color = color_from_100(v); consumed = true;
                    }
                }
                else if (key == "height") {
                    float f; if (parse_scalar(lx, f, e)) { out.scene.fog.height = f; consumed = true; }
                }
                else if (key == "clr_top" || key == "color_top") {
                    Vec3 v; if (parse_vec3_strict(lx, v, e)) {
                        out.scene.fog.color_top = color_from_100(v);
                        out.scene.fog.has_gradient = true;
                        consumed = true;
                    }
                }
                else if (key == "grad_min") {
                    float f; if (parse_scalar(lx, f, e)) { out.scene.fog.gradient_min = f;
                        out.scene.fog.has_gradient = true; consumed = true; }
                }
                else if (key == "grad_max") {
                    float f; if (parse_scalar(lx, f, e)) { out.scene.fog.gradient_max = f;
                        out.scene.fog.has_gradient = true; consumed = true; }
                }
            }
            // v0.0.6: terrain generation
            if (!consumed && (block_name == "tern" || block_name == "terrain")) {
                if (key == "size") {
                    int w, h; if (parse_size2(lx, w, h, e)) {
                        out._tmp_v1 = {(float)w, 0, (float)h}; consumed = true;
                    }
                }
                else if (key == "height" || key == "h") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_f1 = f; consumed = true; }
                }
                else if (key == "scale" || key == "freq") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_f2 = f; consumed = true; }
                }
                else if (key == "octaves" || key == "oct") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_i1 = (int)f; consumed = true; }
                }
                else if (key == "trgrid" || key == "grid") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_i2 = (int)f; consumed = true; }
                }
                else if (key == "seed") {
                    // seed can be a string or number
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out._pending_seed = next.text;
                    } else if (next.kind == TokKind::Number) {
                        out._pending_seed = next.text;
                    }
                    consumed = true;
                }
            }
            // v0.0.6: physical sky
            if (!consumed && (block_name == "gsky" || block_name == "gen_sky")) {
                if (key == "sun" || key == "sun_dir") {
                    consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                }
                else if (key == "turbidity" || key == "turb") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_f1 = f; consumed = true; }
                }
            }
            // v0.0.6: group block
            if (!consumed && (block_name == "grp" || block_name == "group")) {
                if (key == "pos") { consumed = parse_vec3_strict(lx, out._tmp_v1, e); }
                else if (key == "scale" || key == "s") {
                    float f; if (parse_scalar(lx, f, e)) { out._tmp_f1 = f; consumed = true; }
                }
            }
            if (!consumed && block_name == "surf") {
                if      (key == "min") consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "max") consumed = parse_vec3_strict(lx, out._tmp_v3, e);
                else if (key == "bbox") {
                    // bbox: 6 numbers in angle brackets: <min_x, min_y, min_z, max_x, max_y, max_z>...
                    Vec3 lo, hi;
                    if (parse_vec3_strict(lx, lo, e)) {
                        Token sep = lx.next();
                        if (sep.kind == TokKind::Comma) {
                            if (parse_vec3_strict(lx, hi, e)) {
                                out._tmp_v2 = lo;
                                out._tmp_v3 = hi;
                                consumed = true;
                            }
                        } else {
                            e = {sep.line, "expected ',' between min and max in bbox"};
                        }
                    }
                }
                else if (key == "steps") { float f; if (parse_scalar(lx, f, e)) { out._tmp_i1 = (int)f; consumed = true; } }
                else if (key == "step")  { float f; if (parse_scalar(lx, f, e)) { out._tmp_f4 = f; consumed = true; } }
            }
            if (!consumed && block_name == "light") {
                if (key == "typ" || key == "type") {
                    std::string s; if (parse_ident(lx, s, e)) {
                        if (s == "point" || s == "dir" || s == "directional" || s == "sun") {
                            // set typ so we know which kind
                            patch.has_typ = true; patch.typ = s;
                            consumed = true;
                        } else {
                            e = {kt.line, "unknown light type: " + s};
                        }
                    }
                }
                else if (key == "pos")  consumed = parse_vec3_strict(lx, out._tmp_v1, e);
                else if (key == "dir")  consumed = parse_vec3_strict(lx, out._tmp_v2, e);
                else if (key == "clr" || key == "color" || key == "intensity") {
                    Vec3 v; if (parse_vec3_strict(lx, v, e)) {
                        patch.has_clr = true; patch.clr = color_from_100(v);
                        consumed = true;
                    }
                }
            }
            if (!consumed && block_name == "render") {
                if (key == "size") {
                    int w, h; if (parse_size2(lx, w, h, e)) {
                        out.width = w; out.height = h;
                        out.aspect = (float)w / (float)h;
                        consumed = true;
                    }
                }
                else if (key == "spp" || key == "samples") {
                    float f; if (parse_scalar(lx, f, e)) { out.samples = (int)f; consumed = true; }
                }
                else if (key == "depth") {
                    float f; if (parse_scalar(lx, f, e)) { out.depth = (int)f; consumed = true; }
                }
                else if (key == "no_bvh" || key == "nobvh") {
                    std::string s; if (parse_ident(lx, s, e)) {
                        out.scene.use_bvh = !(s == "true" || s == "yes" || s == "1");
                        consumed = true;
                    }
                }
                else if (key == "fps") {
                    float f; if (parse_scalar(lx, f, e)) { out.fps = (int)f; consumed = true; }
                }
                else if (key == "frames" || key == "anim_frames") {
                    float f; if (parse_scalar(lx, f, e)) { out.frames = (int)f; consumed = true; }
                }
                // v0.0.7: render-block feature toggles.
                // cautype: rglr (regular photon map) or ppm (progressive).
                // both enable caustics; ppm additionally enables progressive.
                else if (key == "cautype" || key == "caustics_type") {
                    std::string s; if (parse_ident(lx, s, e)) {
                        if (s == "ppm") {
                            out.enable_caustics = true;
                            out.enable_ppm = true;
                        } else if (s == "rglr" || s == "regular") {
                            out.enable_caustics = true;
                            out.enable_ppm = false;
                        } else if (s == "off" || s == "false" || s == "0") {
                            out.enable_caustics = false;
                        }
                        consumed = true;
                    }
                }
                // spctrl: true/false or 1/0 (spectral rendering)
                else if (key == "spctrl" || key == "spectral") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out.enable_spectral = (next.text == "true" || next.text == "yes");
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_spectral = (std::atof(next.text.c_str()) != 0.0);
                        consumed = true;
                    }
                }
                // sss: true/false or 1/0 (global SSS enable)
                else if (key == "sss") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out.enable_sss = (next.text == "true" || next.text == "yes");
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_sss = (std::atof(next.text.c_str()) != 0.0);
                        consumed = true;
                    }
                }
                // bdpt: bi (bidirectional) or uni (unidirectional)
                // also accepts true/false/1/0 where true=bi.
                else if (key == "bdpt") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        if (next.text == "bi" || next.text == "bidirectional" ||
                            next.text == "true" || next.text == "yes") {
                            out.enable_bdpt = true;
                            out.bdpt_bidirectional = true;
                        } else if (next.text == "uni" || next.text == "unidirectional") {
                            out.enable_bdpt = true;
                            out.bdpt_bidirectional = false;
                        } else if (next.text == "off" || next.text == "false" || next.text == "no") {
                            out.enable_bdpt = false;
                        }
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_bdpt = (std::atof(next.text.c_str()) != 0.0);
                        out.bdpt_bidirectional = true;
                        consumed = true;
                    }
                }
                // volume: true/false or 1/0
                else if (key == "volume" || key == "volumetric") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out.enable_volume = (next.text == "true" || next.text == "yes");
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_volume = (std::atof(next.text.c_str()) != 0.0);
                        consumed = true;
                    }
                }
                // mis: true/false or 1/0
                else if (key == "mis") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out.enable_mis = (next.text == "true" || next.text == "yes");
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_mis = (std::atof(next.text.c_str()) != 0.0);
                        consumed = true;
                    }
                }
                // pt: true/false or 1/0 (path tracing enable)
                else if (key == "pt" || key == "pathtrace") {
                    Token next = lx.next();
                    if (next.kind == TokKind::Ident) {
                        out.enable_pathtrace = (next.text == "true" || next.text == "yes");
                        consumed = true;
                    } else if (next.kind == TokKind::Number) {
                        out.enable_pathtrace = (std::atof(next.text.c_str()) != 0.0);
                        consumed = true;
                    }
                }
            }

            if (!consumed) {
                if (e.what.empty()) {
                    e = {kt.line, "unknown key '" + key + "' in " + block_name + " block"};
                }
                errors.push_back(e);
                ok = false;
                // try to skip the value token to keep parsing
                Token skip = lx.next();
            }
        }

        // block closed
        if (block_name == "cmra") {
            // v0.0.3: apply animation overrides for camera.
            // p overrides eye, r rotates the look direction, s scales fov.
            if (out._anim_has_pos) {
                out.eye = out._tmp_v1;
            }
            if (out._anim_has_rot) {
                // rotate the view direction by Euler angles (degrees)
                Vec3 dir = (out.look - out.eye).normalized();
                float rx = out._anim_rot.x * 3.14159265f / 180.0f;
                float ry = out._anim_rot.y * 3.14159265f / 180.0f;
                float rz = out._anim_rot.z * 3.14159265f / 180.0f;
                // yaw (Y rotation)
                float cy = std::cos(ry), sy = std::sin(ry);
                Vec3 dy = {dir.x * cy + dir.z * sy, dir.y, -dir.x * sy + dir.z * cy};
                // pitch (X rotation)
                float cx = std::cos(rx), sx = std::sin(rx);
                Vec3 dx = {dy.x, dy.y * cx - dy.z * sx, dy.y * sx + dy.z * cx};
                float dist = (out.look - out.eye).length();
                out.look = out.eye + dx * dist;
                // roll (Z rotation) rotates the up vector
                if (rz != 0.0f) {
                    float cz = std::cos(rz), sz = std::sin(rz);
                    Vec3 up_rot = {out.up.x * cz - out.up.y * sz,
                                   out.up.x * sz + out.up.y * cz,
                                   out.up.z};
                    out.up = up_rot.normalized();
                }
            }
            if (out._anim_has_scale) {
                // scale on camera = FOV zoom (s.x < 1 = zoom in, s.x > 1 = zoom out)
                out.fov = out.fov / out._anim_scale.x;
            }
            out.aspect = (out.aspect == 0.0f) ? 1.0f : out.aspect;
        }
        else if (block_name == "mat") {
            // a new mat {} block should RESET the material to defaults,
            // then apply the patch. this prevents properties from previous
            // materials (especially emission) from leaking into the new one.
            // without this, `mat { clr: <0,100,0> }` after an emissive mat
            // would inherit the emission, making the "green" material also
            // glow white.
            // v0.0.6: start with preset material if specified, else defaults
            Material fresh;
            if (patch.has_prst) {
                fresh = get_preset(patch.prst_name);
            } else {
                fresh = make_diffuse({0.7f, 0.7f, 0.7f}, 0.0f, 32.0f);
            }
            out._current_mat = apply_patch(fresh, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) out._current_mat.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            // v0.0.4: normal map
            if (patch.has_nrml) {
                auto ntex = std::make_shared<Texture>();
                std::string nerr;
                if (ntex->load(patch.nrml_path, nerr)) out._current_mat.normal_map = ntex;
                else { errors.push_back({0, "normal map load failed: " + nerr}); ok = false; }
            }
            // v0.0.7: true displacement map. either load from file or
            // use a procedural texture (noise/marble/wood/checker).
            if (patch.has_disp) {
                auto dtex = std::make_shared<Texture>();
                if (!patch.disp_proc.empty()) {
                    ProcType pt = ProcType::Noise;
                    if (patch.disp_proc == "marble")      pt = ProcType::Marble;
                    else if (patch.disp_proc == "wood")  pt = ProcType::Wood;
                    else if (patch.disp_proc == "checker") pt = ProcType::Checker;
                    else if (patch.disp_proc == "ridged") pt = ProcType::Ridged;
                    else if (patch.disp_proc == "fbm")    pt = ProcType::Fbm;
                    dtex->set_procedural(pt, 8.0f,
                        Color{1, 1, 1}, Color{0, 0, 0});
                } else if (!patch.disp_path.empty()) {
                    std::string derr;
                    if (!dtex->load(patch.disp_path, derr)) {
                        errors.push_back({0, "displacement map load failed: " + derr});
                        ok = false;
                        dtex.reset();
                    }
                }
                if (dtex && dtex->valid()) {
                    out._current_mat.displacement_map = dtex;
                    out._current_mat.displacement_amount = patch.disp_amount;
                }
            }
        }
        else if (block_name == "sky" || block_name == "skybox" || block_name == "env") {
            // sky { file: "..." }
            if (!out._pending_mesh_file.empty()) {
                auto tex = std::make_shared<Texture>();
                std::string terr;
                if (tex->load(out._pending_mesh_file, terr)) {
                    out.scene.skybox = tex;
                } else {
                    errors.push_back({0, "skybox load failed: " + terr + " (" + out._pending_mesh_file + ")"});
                    ok = false;
                }
                out._pending_mesh_file.clear();
            }
        }
        else if (block_name == "include") {
            // include { file: "..." } - parsed inline after this block.
            // _pending_include is set, the main loop handles it.
            if (out._pending_include.empty() && !out._pending_mesh_file.empty()) {
                out._pending_include = out._pending_mesh_file;
                out._pending_mesh_file.clear();
            }
        }
        // v0.0.6: group   push transform, children get offset
        else if (block_name == "grp" || block_name == "group") {
            ParsedScene::GroupTransform gt;
            gt.offset = out._tmp_v1;
            gt.scale = {out._tmp_f1, out._tmp_f1, out._tmp_f1};
            gt.active = true;
            out.group_stack.push_back(gt);
            // nOTE: the group's children are parsed AFTER this block action.
            // but pecl blocks are flat, not nested! we need a different approach.
            // solution: the group block's pos is applied to ALL subsequent
            // primitives until another grp resets it. but that's not how
            // nested blocks work in .pecl...
            //
            // actually .pecl DOES support nesting because the parser reads
            // blocks recursively (each block reads its own keys until }).
            // but the current parser flattens everything. so groups work
            // as a "current offset" that applies to subsequent objects.
            //
            // for now, groups set a global offset that's applied to all
            // subsequent primitives. to "close" a group, just don't use
            // the offset.
        }
        else if (block_name == "fog") {
            if (out.scene.fog.density > 0.0f) {
                out.scene.fog.enabled = true;
            }
        }
        // v0.0.6: terrain generation using perlin noise
        else if (block_name == "tern" || block_name == "terrain") {
            Material m = apply_patch(out._current_mat, patch);
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            float terrain_w = out._tmp_v1.x > 0 ? out._tmp_v1.x : 20.0f;
            float terrain_d = out._tmp_v1.z > 0 ? out._tmp_v1.z : 20.0f;
            float terrain_h = out._tmp_f1 > 0 ? out._tmp_f1 : 3.0f;
            float freq = out._tmp_f2 > 0 ? out._tmp_f2 : 0.1f;
            int octaves = out._tmp_i1 > 0 ? out._tmp_i1 : 4;
            // hash seed string to uint32
            uint32_t seed = 0;
            if (!out._pending_seed.empty()) {
                seed = PerlinNoise::hash_seed(out._pending_seed);
            } else {
                seed = (uint32_t)(12345 + mid * 7);
            }
            PerlinNoise perlin(seed);
            // generate grid of triangles
            int grid = out._tmp_i2 > 0 ? out._tmp_i2 : 32; // terrain grid resolution
            for (int zi = 0; zi < grid; ++zi) {
                for (int xi = 0; xi < grid; ++xi) {
                    float x0 = (float)xi / grid * terrain_w - terrain_w / 2;
                    float z0 = (float)zi / grid * terrain_d - terrain_d / 2;
                    float x1 = (float)(xi + 1) / grid * terrain_w - terrain_w / 2;
                    float z1 = (float)(zi + 1) / grid * terrain_d - terrain_d / 2;
                    float h00 = perlin.fbm(x0 * freq, z0 * freq, octaves) * terrain_h;
                    float h10 = perlin.fbm(x1 * freq, z0 * freq, octaves) * terrain_h;
                    float h01 = perlin.fbm(x0 * freq, z1 * freq, octaves) * terrain_h;
                    float h11 = perlin.fbm(x1 * freq, z1 * freq, octaves) * terrain_h;
                    Point3 v00{x0, h00, z0}, v10{x1, h10, z0}, v01{x0, h01, z1}, v11{x1, h11, z1};
                    out.scene.add_primitive(std::make_shared<Triangle>(v00, v10, v11, mid));
                    out.scene.add_primitive(std::make_shared<Triangle>(v00, v11, v01, mid));
                }
            }
            out._pending_seed.clear();
        }
        // v0.0.6: physical sky (preetham-based)
        else if (block_name == "gsky" || block_name == "gen_sky") {
            Vec3 sun_dir = out._tmp_v1.length_squared() > 0 ? out._tmp_v1.normalized() : Vec3{0.3f, 0.5f, 0.4f};
            float turbidity = out._tmp_f1 > 0 ? out._tmp_f1 : 2.5f;

            // preetham sky model: compute zenith luminance and chromaticity
            float theta_s = std::acos(clampf(sun_dir.y, -1.0f, 1.0f)); // sun zenith angle
            float cos_theta_s = sun_dir.y;
            float T = turbidity;

            // zenith luminance (preetham)
            float chi = (4.0f / 9.0f - T / 120.0f) * (3.14159265f - 2.0f * theta_s);
            float chi_clamped = clampf(chi, -1.5f, 1.5f); float Yz = (4.0453f * T - 4.9710f) * std::tan(chi_clamped) - 0.2155f * T + 2.4192f;
            Yz = clampf(Yz, 0.0f, 10.0f);

            // zenith chromaticity x and y
            float xz = (0.00165f * theta_s * theta_s * theta_s - 0.00375f * theta_s * theta_s + 0.00209f * theta_s + 0.0f) * T
                     + (0.02901f * theta_s * theta_s * theta_s - 0.06377f * theta_s * theta_s + 0.04895f * theta_s + 0.0008f);
            float yz = (0.00275f * theta_s * theta_s * theta_s - 0.00610f * theta_s * theta_s + 0.00394f * theta_s + 0.0f) * T
                     + (0.10678f * theta_s * theta_s * theta_s - 0.23679f * theta_s * theta_s + 0.19798f * theta_s + 0.03202f);
            xz = clampf(xz, 0.0f, 1.0f);
            yz = clampf(yz, 0.0f, 1.0f);

            // convert xyY to RGB
            float Y = clampf(Yz * 0.3f, 0.0f, 2.0f);
            float X = (std::fabs(yz) > 1e-6f) ? (xz / yz) * Y : 0.0f;
            float Z = (std::fabs(yz) > 1e-6f) ? ((1.0f - xz - yz) / yz) * Y : 0.0f;
            // XYZ to linear RGB
            float r = 3.2406f * X - 1.5372f * Y - 0.4986f * Z;
            float g = -0.9689f * X + 1.8758f * Y + 0.0415f * Z;
            float b = 0.0557f * X - 0.2040f * Y + 1.0570f * Z;
            r = clampf(r, 0.0f, 2.0f);
            g = clampf(g, 0.0f, 2.0f);
            b = clampf(b, 0.0f, 2.0f);

            // sun color depends on elevation (redder near horizon)
            float elev = clampf(cos_theta_s, 0.0f, 1.0f);
            Color sun_color = {1.0f, 0.5f + 0.5f * elev, 0.2f + 0.8f * elev};

            out.scene.background = {r, g, b};
            out.scene.add_light(make_directional_light(-sun_dir, sun_color * 3.0f));
        }
        else if (block_name == "sphere") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            // v0.0.3: apply animation scale to radius
            float rad = out._tmp_f1;
            if (out._anim_has_scale) rad *= out._anim_scale.x;
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            Vec3 sphere_pos = out._tmp_v1 + out.group_offset();
            // v0.0.6: physics simulation
            if (out._has_physics && out._anim_frame > 0) {
                float dt = 1.0f / 30.0f;
                float g = out._phys_gravity;
                float bounce = out._phys_bounce;
                float rad_val = rad;
                // ground is at y=-1, ball center should stop at ground_y + rad
                float ground_y = -1.0f + rad_val;
                float y = out._tmp_v1.y;
                float vy = 0.0f;
                // use sub-stepping for stability (4 substeps per frame)
                int substeps = 4;
                float sub_dt = dt / substeps;
                for (int f = 1; f <= out._anim_frame; ++f) {
                    for (int s = 0; s < substeps; ++s) {
                        vy -= g * sub_dt;
                        y += vy * sub_dt;
                        // ground collision with proper restitution
                        if (y < ground_y) {
                            y = ground_y;
                            vy = -vy * bounce;
                            // friction on bounce
                            if (std::fabs(vy) < 0.3f) {
                                vy = 0.0f;
                                y = ground_y;
                            }
                        }
                    }
                }
                sphere_pos.y = y;
            }
            out.scene.add_primitive(std::make_shared<Sphere>(sphere_pos, rad, mid));
            // v0.0.7: true displacement. if the material has a
            // displacement map, also tessellate the sphere into a
            // triangle mesh with displaced vertices and add it to
            // the scene. we keep the analytic sphere too for the BVH
            // (the displaced mesh is the actual renderable surface).
            //
            // actually we need to REPLACE the sphere with the mesh,
            // otherwise both surfaces would render and z-fight. so
            // pop the sphere we just added and add the mesh instead.
            if (m.displacement_map && m.displacement_map->valid()) {
                // remove the analytic sphere we just added
                out.scene.pop_primitive();
                // v0.0.7: 96 segments = ~18000 triangles. higher = more
                // detailed but slower. the old 64 segments gave a "fuzzy"
                // look because the noise was too smooth for the resolution.
                MeshData md = tessellate_displaced_sphere(
                    sphere_pos, rad, mid, m.displacement_map,
                    m.displacement_amount, 64);
                for (auto& tri : md.triangles) {
                    out.scene.add_primitive(tri);
                }
            }
            // v0.0.6: store initial position for physics
            if (out._has_physics) {
                out._phys_initial_pos = sphere_pos;
            }
            if (m.emission.length_squared() > 1e-6f) {
                out.scene.add_light(make_point_light(
                    out._tmp_v1, m.emission, rad * 0.5f, true));
            }
        }
        else if (block_name == "cube") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            // v0.0.3: apply animation scale to size
            float sz = out._tmp_f1;
            if (out._anim_has_scale) sz *= out._anim_scale.x;
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            Vec3 cube_pos = out._tmp_v1 + out.group_offset();
            out.scene.add_primitive(std::make_shared<Cube>(cube_pos, sz, mid));
        }
        else if (block_name == "plane") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            out.scene.add_primitive(std::make_shared<Plane>(out._tmp_v1, out._tmp_v2, mid));
        }
        else if (block_name == "tri" || block_name == "triangle") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            out.scene.add_primitive(std::make_shared<Triangle>(out._tmp_v1, out._tmp_v2, out._tmp_v3, mid));
        }
        else if (block_name == "quad") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            auto tris = make_quad(out._tmp_v1, out._tmp_v2, out._tmp_v3, out._tmp_v4, mid);
            for (auto& t : tris) out.scene.add_primitive(std::move(t));
        }
        else if (block_name == "cyl" || block_name == "cylinder") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            Vec3 axis = (out._tmp_v2.length_squared() < 1e-9f) ? Vec3{0,1,0} : out._tmp_v2;
            out.scene.add_primitive(std::make_shared<Cylinder>(
                out._tmp_v1, axis, out._tmp_f1, out._tmp_f2, mid));
        }
        else if (block_name == "cone") {
            Material m = apply_patch(out._current_mat, patch);
            if (patch.has_tex) {
                auto tex = make_texture_from_patch(patch);
                if (tex) m.texture = tex;
                else { errors.push_back({0, "texture load failed: " + patch.tex_path}); ok = false; }
            }
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            Vec3 axis = (out._tmp_v2.length_squared() < 1e-9f) ? Vec3{0,1,0} : out._tmp_v2;
            out.scene.add_primitive(std::make_shared<Cone>(
                out._tmp_v1, axis, out._tmp_f1, out._tmp_f2, mid));
        }
        else if (block_name == "mesh") {
            Material m = apply_patch(out._current_mat, patch);
            out.scene.add_material(m);
            int mid = out.scene.material_count() - 1;
            if (out._pending_mesh_file.empty()) {
                errors.push_back({0, "mesh block requires a file: ... entry"});
                ok = false;
            } else {
                MeshData md;
                std::string merr;
                if (!load_obj(out._pending_mesh_file, md, mid, merr)) {
                    errors.push_back({0, merr});
                    ok = false;
                } else {
                    float target = (out._tmp_f1 > 0.0f) ? out._tmp_f1 : 2.0f;
                    MeshTransform t = fit_mesh(md, target);
                    Vec3 offset = out._tmp_v3;
                    // bake scale + centering + user offset into each vertex
                    for (auto& prim : md.triangles) {
                        auto tri = std::dynamic_pointer_cast<Triangle>(prim);
                        if (!tri) continue;
                        Point3 v0 = tri->v0() * t.scale + t.offset + offset;
                        Point3 v1 = tri->v1() * t.scale + t.offset + offset;
                        Point3 v2 = tri->v2() * t.scale + t.offset + offset;
                        out.scene.add_primitive(
                            std::make_shared<Triangle>(v0, v1, v2, mid));
                    }
                }
            }
            out._pending_mesh_file.clear();
        }
        else if (block_name == "surf") {
            // compile the pending expression and build an ImplicitSurface
            std::string expr_err;
            auto expr = parse_expression(out._pending_surf_func, expr_err);
            if (!expr) {
                errors.push_back({0, "surf func parse error: " + expr_err});
                ok = false;
            } else {
                Material m = apply_patch(out._current_mat, patch);
                out.scene.add_material(m);
                int mid = out.scene.material_count() - 1;
                Vec3 bmin = out._tmp_v2, bmax = out._tmp_v3;
                // default bounding box if not set
                if (bmin.length_squared() == 0.0f && bmax.length_squared() == 0.0f) {
                    bmin = {-2.0f, -2.0f, -2.0f};
                    bmax = { 2.0f,  2.0f,  2.0f};
                }
                int steps = out._tmp_i1 > 0 ? out._tmp_i1 : 200;
                float step = out._tmp_f4 > 0.0f ? out._tmp_f4 : 0.0f;
                out.scene.add_primitive(std::make_shared<ImplicitSurface>(
                    expr, bmin, bmax, mid, steps, step));
            }
            out._pending_surf_func.clear();
        }
        else if (block_name == "light") {
            Light l;
            if (patch.has_typ && (patch.typ == "dir" || patch.typ == "directional" || patch.typ == "sun")) {
                l = make_directional_light(out._tmp_v2, patch.has_clr ? patch.clr : Color{1,1,1});
            } else {
                // soft shadows: default radius is 0.5 (soft).
                // override with radius: 0 for hard shadows, or a larger value.
                float r = patch.has_radius ? patch.radius : 0.5f;
                l = make_point_light(out._tmp_v1,
                                     patch.has_clr ? patch.clr : Color{1,1,1},
                                     r);
            }
            out.scene.add_light(l);
        }
        else if (block_name == "render") {
            // already consumed into out fields
        }
        else {
            errors.push_back({t.line, "unknown block '" + block_name + "' (skipped)"});
            ok = false;
        }

        // reset scratch fields for the next block
        out._tmp_v1 = {0,0,0}; out._tmp_v2 = {0,0,0};
        out._tmp_v3 = {0,0,0}; out._tmp_v4 = {0,0,0};
        out._tmp_f1 = 1.0f; out._tmp_f2 = 1.0f;
        out._tmp_f4 = 0.0f; out._tmp_i1 = 0;
        // reset animation flags so they don't leak to the next block
        out._anim_has_pos = false;
        out._anim_has_scale = false;
        out._anim_has_rot = false;
        out._anim_scale = {1, 1, 1};
        out._anim_rot = {0, 0, 0};

        // handle include directive: parse the included file inline.
        if (!out._pending_include.empty()) {
            std::string inc_path = out._pending_include;
            out._pending_include.clear();
            std::string full_path = inc_path;
            std::ifstream test(inc_path);
            if (!test && !out._base_dir.empty()) {
                full_path = out._base_dir + "/" + inc_path;
            }
            test.close();
            // save and clear base_dir so the included file uses its own dir
            std::string saved_base = out._base_dir;
            // extract dir of full_path
            size_t slash = full_path.find_last_of('/');
            out._base_dir = (slash != std::string::npos) ? full_path.substr(0, slash) : "";
            // don't re-finalize on includes
            bool saved_skip = out._skip_finalize;
            out._skip_finalize = true;
            std::vector<ParseError> inc_errs;
            parse_pecl(full_path, out, inc_errs);
            for (auto& e : inc_errs) errors.push_back(e);
            // restore
            out._base_dir = saved_base;
            out._skip_finalize = saved_skip;
        }
    }

    // final aspect from render size if size was set
    if (out.width > 0 && out.height > 0) {
        out.aspect = (float)out.width / (float)out.height;
    }
    if (!out._skip_finalize) {
        out.finalize();
    }
    return ok;
}

} // namespace pencil
