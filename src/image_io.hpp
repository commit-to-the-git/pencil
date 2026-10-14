// image_io.hpp - image input/output for pencil
// SPDX-License-Identifier: Apache-2.0
//
// supports:
// - reading PNG/JPG/BMP/TGA for textures and skyboxes (via stb_image)
// - writing PNG output (via stb_image_write)
// - writing PPM P6 binary (legacy, no deps)
// - v0.0.5: tone mapping (ACES/Reinhard) + exposure control
// output format auto-detected from file extension.

#pragma once

#include "stb/stb_image.h"
#include "stb/stb_image_write.h"
#include "tonemap.hpp"
#include "vec3.hpp"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <tuple>
#include <vector>

namespace pencil {

// detect output format by extension.
enum class ImageFormat { PPM, PNG, UNKNOWN };

inline ImageFormat detect_format(const std::string& path) {
    auto pos = path.find_last_of('.');
    if (pos == std::string::npos) return ImageFormat::UNKNOWN;
    std::string ext = path.substr(pos + 1);
    for (auto& c : ext) c = (char)std::tolower(c);
    if (ext == "ppm") return ImageFormat::PPM;
    if (ext == "png") return ImageFormat::PNG;
    return ImageFormat::UNKNOWN;
}

// v0.0.6: shared "HDR Color -> 8-bit RGB" pipeline.
// applies tone map -> exposure -> gamma 2.2 -> 8-bit. this is what
// the LOCAL write_image() does internally. the distributed worker
// used to skip the gamma step, producing renders that were visually
// darker than local renders of the same scene. routing both paths
// through this function makes them pixel-identical.
inline void color_to_rgb8(const Color& c, ToneMap tonemap, float exposure,
                          uint8_t& out_r, uint8_t& out_g, uint8_t& out_b) {
    Color mapped = tonemap_color(c, tonemap, exposure);
    float r = clampf(mapped.x, 0.0f, 1.0f);
    float g = clampf(mapped.y, 0.0f, 1.0f);
    float b = clampf(mapped.z, 0.0f, 1.0f);
    // gamma 2.2
    r = std::pow(r, 1.0f / 2.2f);
    g = std::pow(g, 1.0f / 2.2f);
    b = std::pow(b, 1.0f / 2.2f);
    out_r = (uint8_t)(r * 255.0f + 0.5f);
    out_g = (uint8_t)(g * 255.0f + 0.5f);
    out_b = (uint8_t)(b * 255.0f + 0.5f);
}

// write pixels to a file. format auto-detected from path extension.
// v0.0.5: applies tone mapping + exposure, then gamma 2.2.
inline bool write_image(const std::string& path,
                        const std::vector<Color>& pixels,
                        int width, int height,
                        std::string& err_out,
                        ToneMap tonemap = ToneMap::ACES,
                        float exposure = 0.0f) {
    ImageFormat fmt = detect_format(path);
    if (fmt == ImageFormat::UNKNOWN) {
        err_out = "unknown image format: " + path + " (use .ppm or .png)";
        return false;
    }

    // helper: process a color through tone map -> exposure -> gamma -> 8bit
    auto process = [&](const Color& c) -> std::tuple<uint8_t,uint8_t,uint8_t> {
        Color mapped = tonemap_color(c, tonemap, exposure);
        // gamma 2.2
        float r = clampf(mapped.x, 0.0f, 1.0f);
        float g = clampf(mapped.y, 0.0f, 1.0f);
        float b = clampf(mapped.z, 0.0f, 1.0f);
        r = std::pow(r, 1.0f / 2.2f);
        g = std::pow(g, 1.0f / 2.2f);
        b = std::pow(b, 1.0f / 2.2f);
        return { (uint8_t)(r * 255.0f + 0.5f),
                 (uint8_t)(g * 255.0f + 0.5f),
                 (uint8_t)(b * 255.0f + 0.5f) };
    };
    if (fmt == ImageFormat::PPM) {
        std::ofstream f(path, std::ios::binary);
        if (!f) { err_out = "cannot open output: " + path; return false; }
        f << "P6\n" << width << ' ' << height << "\n255\n";
        for (const Color& c : pixels) {
            auto [r, g, b] = process(c);
            uint8_t bytes[3] = {r, g, b};
            f.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
        }
        return true;
    }

    // pNG via stb_image_write.
    std::vector<uint8_t> buf((size_t)width * height * 3);
    for (int i = 0; i < width * height; ++i) {
        auto [r, g, b] = process(pixels[i]);
        buf[i*3 + 0] = r;
        buf[i*3 + 1] = g;
        buf[i*3 + 2] = b;
    }

    int ok = stbi_write_png(path.c_str(), width, height, 3, buf.data(), width * 3);
    if (!ok) { err_out = "stbi_write_png failed: " + path; return false; }
    return true;
}

// write pixels to a file as raw 8-bit RGB, no gamma.
// used for normal/AOV outputs where values are already encoded.
inline bool write_image_raw(const std::string& path,
                            const uint8_t* rgb,
                            int width, int height,
                            std::string& err_out) {
    ImageFormat fmt = detect_format(path);
    if (fmt == ImageFormat::PNG) {
        int ok = stbi_write_png(path.c_str(), width, height, 3, rgb, width * 3);
        if (!ok) { err_out = "stbi_write_png failed: " + path; return false; }
        return true;
    }
    if (fmt == ImageFormat::PPM) {
        std::ofstream f(path, std::ios::binary);
        if (!f) { err_out = "cannot open output: " + path; return false; }
        f << "P6\n" << width << ' ' << height << "\n255\n";
        f.write(reinterpret_cast<const char*>(rgb), (size_t)width * height * 3);
        return true;
    }
    err_out = "unknown image format: " + path;
    return false;
}

// read an image file (PNG/JPG/BMP/TGA) into RGB8 buffer.
// returns true on success, fills out_w/out_h/out_pixels.
// caller owns the pixels buffer (must free with stbi_image_free).
inline bool read_image(const std::string& path, int& out_w, int& out_h,
                       std::vector<uint8_t>& out_pixels, std::string& err_out,
                       int desired_channels = 3) {
    int w, h, n;
    uint8_t* data = stbi_load(path.c_str(), &w, &h, &n, desired_channels);
    if (!data) {
        err_out = std::string("stbi_load failed: ") + stbi_failure_reason();
        return false;
    }
    out_w = w; out_h = h;
    out_pixels.assign(data, data + (size_t)w * h * desired_channels);
    stbi_image_free(data);
    return true;
}

} // namespace pencil
