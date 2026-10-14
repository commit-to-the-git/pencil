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

// ppm.hpp - PPM (P6 binary) writer for Pencil
#pragma once

#include "vec3.hpp"

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace pencil {

inline bool write_ppm(const std::string& path,
                      const std::vector<Color>& pixels,
                      int width, int height,
                      std::string& err_out) {
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        err_out = "Cannot open output file: " + path;
        return false;
    }

    // header
    f << "P6\n" << width << ' ' << height << "\n255\n";

    // body
    for (const Color& c : pixels) {
        // gamma correction: pixel^{1/2.2}
        float r = clampf(c.x, 0.0f, 1.0f);
        float g = clampf(c.y, 0.0f, 1.0f);
        float b = clampf(c.z, 0.0f, 1.0f);
        r = std::pow(r, 1.0f / 2.2f);
        g = std::pow(g, 1.0f / 2.2f);
        b = std::pow(b, 1.0f / 2.2f);
        uint8_t bytes[3] = {
            (uint8_t)(r * 255.0f + 0.5f),
            (uint8_t)(g * 255.0f + 0.5f),
            (uint8_t)(b * 255.0f + 0.5f),
        };
        f.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
    }
    return true;
}

} // namespace pencil
