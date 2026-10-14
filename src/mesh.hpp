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

// mesh.hpp - obj loader and triangle mesh for pencil SPDX-License-Identifier: A...

#pragma once

#include "primitives.hpp"

#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace pencil {

// parses an .obj file
struct MeshData {
    std::vector<Point3>                       vertices;
    std::vector<std::shared_ptr<Primitive>>   triangles;
    // computed bbox of all verts, used by the bvh
    AABB bbox;
};

inline bool load_obj(const std::string& path, MeshData& out, int material_id,
                     std::string& err) {
    std::ifstream f(path);
    if (!f) { err = "cannot open obj: " + path; return false; }

    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        if (line.empty()) continue;
        // strip comments and trailing whitespace
        if (line[0] == '#') continue;

        std::istringstream ss(line);
        std::string tag;
        ss >> tag;

        if (tag == "v") {
            float x, y, z;
            if (!(ss >> x >> y >> z)) { err = "bad v line " + std::to_string(lineno); return false; }
            out.vertices.push_back({x, y, z});
        }
        else if (tag == "f") {
            // each token is "v", "v/vt", "v//vn", or "v/vt/vn"
            std::vector<int> idx;
            std::string tok;
            while (ss >> tok) {
                int v = 0;
                // parse leading integer
                size_t i = 0;
                while (i < tok.size() && (tok[i] == '-' || std::isdigit((unsigned char)tok[i]))) ++i;
                if (i == 0) { err = "bad face token at line " + std::to_string(lineno); return false; }
                v = std::atoi(tok.c_str());
                if (v < 0) v = (int)out.vertices.size() + v + 1;
                if (v < 1 || v > (int)out.vertices.size()) {
                    err = "invalid vertex index " + std::to_string(v) + " at line " + std::to_string(lineno) +
                          " (only " + std::to_string(out.vertices.size()) + " vertices loaded)";
                    return false;
                }
                idx.push_back(v - 1);
            }
            if (idx.size() < 3) {
                err = "face with < 3 valid vertices at line " + std::to_string(lineno);
                continue;
            }
            // fan triangulate: (0, i, i+1) for i in [1, n-1)
            for (size_t i = 1; i + 1 < idx.size(); ++i) {
                out.triangles.push_back(std::make_shared<Triangle>(
                    out.vertices[idx[0]],
                    out.vertices[idx[i]],
                    out.vertices[idx[i+1]],
                    material_id));
            }
        }
        // silently ignore vt, vn, mtllib, usemtl, o, g, s, etc
    }

    // compute bbox
    AABB box;
    for (const auto& v : out.vertices) {
        box.min = {std::fmin(box.min.x, v.x),
                   std::fmin(box.min.y, v.y),
                   std::fmin(box.min.z, v.z)};
        box.max = {std::fmax(box.max.x, v.x),
                   std::fmax(box.max.y, v.y),
                   std::fmax(box.max.z, v.z)};
    }
    out.bbox = box;
    return true;
}

// helper: compute scale and translate to fit a mesh into a target box
struct MeshTransform {
    float  scale = 1.0f;
    Vec3   offset = {0,0,0};
};

inline MeshTransform fit_mesh(const MeshData& m, float target_size) {
    Vec3 ext = m.bbox.max - m.bbox.min;
    float longest = std::fmax(std::fmax(ext.x, ext.y), ext.z);
    if (longest <= 0.0f) return {};
    MeshTransform t;
    t.scale = target_size / longest;
    Vec3 center = (m.bbox.min + m.bbox.max) * 0.5f;
    t.offset = -center * t.scale;
    return t;
}

// v0.0.7: true displacement. tessellate a sphere into a triangle mesh
// with `segments` latitude bands. each vertex is offset along the
// sphere normal by (displacement_texture(u,v).r * amount). this gives
// real silhouette and self-shadowing changes, unlike a normal map
// which only perturbs the shading normal. the result is added to
// the scene as triangle primitives (replacing the analytic sphere).
//
// segments controls the resolution. 64 gives ~8000 triangles, which
// is enough for fine surface detail at the cost of a slower BVH build
// and more ray-triangle intersections.
//
// the displacement texture's RED channel is used as the height field.
// values in [0,1] -> offset in [0, amount]. (use a negative amount
// for inward displacement.)
inline MeshData tessellate_displaced_sphere(Point3 center, float radius,
                                            int material_id,
                                            TexturePtr disp_tex,
                                            float amount,
                                            int segments = 96) {
    MeshData md;
    if (segments < 4) segments = 4;
    // generate vertices on a unit sphere using spherical coordinates.
    // latitude: theta in [0, pi] (top to bottom)
    // longitude: phi in [0, 2pi)
    std::vector<Point3> verts;
    std::vector<Vec3> normals;
    std::vector<std::pair<float,float>> uvs;
    verts.reserve((size_t)(segments + 1) * (segments + 1));
    normals.reserve(verts.size());
    uvs.reserve(verts.size());
    for (int i = 0; i <= segments; ++i) {
        float theta = 3.14159265358979f * (float)i / (float)segments;
        float st = std::sin(theta), ct = std::cos(theta);
        for (int j = 0; j <= segments; ++j) {
            float phi = 2.0f * 3.14159265358979f * (float)j / (float)segments;
            float sp = std::sin(phi), cp = std::cos(phi);
            Vec3 n = {st * cp, ct, st * sp};
            Vec3 p = center + n * radius;
            // uv from spherical: u = phi / (2pi), v = theta / pi
            float u = (float)j / (float)segments;
            float v = (float)i / (float)segments;
            // sample displacement texture
            float h = 0.0f;
            if (disp_tex && disp_tex->valid()) {
                Color c = disp_tex->sample(u, v);
                h = c.x; // red channel
            }
            // offset along normal
            Vec3 displaced = p + n * (h * amount);
            verts.push_back(displaced);
            normals.push_back(n);
            uvs.push_back({u, v});
        }
    }
    // build triangles: two per grid cell
    int n = segments + 1;
    for (int i = 0; i < segments; ++i) {
        for (int j = 0; j < segments; ++j) {
            int a = i * n + j;
            int b = i * n + j + 1;
            int c = (i + 1) * n + j;
            int d = (i + 1) * n + j + 1;
            md.triangles.push_back(std::make_shared<Triangle>(
                verts[a], verts[b], verts[c], material_id));
            md.triangles.push_back(std::make_shared<Triangle>(
                verts[b], verts[d], verts[c], material_id));
        }
    }
    // bbox
    for (const auto& v : verts) {
        md.bbox.min.x = std::fmin(md.bbox.min.x, v.x);
        md.bbox.min.y = std::fmin(md.bbox.min.y, v.y);
        md.bbox.min.z = std::fmin(md.bbox.min.z, v.z);
        md.bbox.max.x = std::fmax(md.bbox.max.x, v.x);
        md.bbox.max.y = std::fmax(md.bbox.max.y, v.y);
        md.bbox.max.z = std::fmax(md.bbox.max.z, v.z);
    }
    md.vertices = std::move(verts);
    return md;
}

} // namespace pencil
