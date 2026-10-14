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

// ray.hpp - A parametric ray P(t) = origin + t * direction
#pragma once

#include "vec3.hpp"

namespace pencil {

struct Ray {
    Point3 origin;
    Vec3   direction;  // not necessarily normalized; consumers normalize as needed.

    constexpr Ray() = default;
    constexpr Ray(const Point3& o, const Vec3& d) : origin(o), direction(d) {}

    // evaluate at parameter t
    Point3 at(float t) const { return origin + direction * t; }
};

} // namespace pencil
