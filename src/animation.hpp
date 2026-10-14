// animation.hpp - keyframe animation data for pencil
// SPDX-License-Identifier: Apache-2.0
//
// v0.0.4: added easing functions for keyframes.
// supported: linear, ease_in, ease_out, ease_in_out, bounce, elastic, back.

#pragma once

#include "vec3.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace pencil {

enum class Ease {
    Linear,
    EaseIn,
    EaseOut,
    EaseInOut,
    Bounce,
    Elastic,
    Back,
};

inline Ease parse_ease(const std::string& s) {
    if (s == "linear")     return Ease::Linear;
    if (s == "ease_in")    return Ease::EaseIn;
    if (s == "ease_out")   return Ease::EaseOut;
    if (s == "ease_in_out") return Ease::EaseInOut;
    if (s == "bounce")     return Ease::Bounce;
    if (s == "elastic")    return Ease::Elastic;
    if (s == "back")       return Ease::Back;
    return Ease::EaseInOut;  // default
}

// apply an easing function to t in [0, 1].
inline float apply_ease(float t, Ease e) {
    switch (e) {
        case Ease::Linear:
            return t;
        case Ease::EaseIn:
            return t * t;
        case Ease::EaseOut:
            return 1.0f - (1.0f - t) * (1.0f - t);
        case Ease::EaseInOut:
            return t * t * (3.0f - 2.0f * t);  // smoothstep
        case Ease::Bounce: {
            // bounce out: a ball bouncing to rest
            if (t < 1.0f/2.75f) return t * t * 7.5625f;
            else if (t < 2.0f/2.75f) { t -= 1.5f/2.75f; return t * t * 7.5625f + 0.75f; }
            else if (t < 2.5f/2.75f) { t -= 2.25f/2.75f; return t * t * 7.5625f + 0.9375f; }
            else { t -= 2.625f/2.75f; return t * t * 7.5625f + 0.984375f; }
        }
        case Ease::Elastic: {
            // elastic: oscillates with decaying amplitude
            if (t == 0.0f || t == 1.0f) return t;
            float p = 0.3f;
            float s = p / 4.0f;
            return std::pow(2.0f, -10.0f * t) * std::sin((t - s) * (2.0f * 3.14159265f) / p) + 1.0f;
        }
        case Ease::Back: {
            // back: overshoots slightly then settles
            float c1 = 1.70158f;
            float c3 = c1 + 1.0f;
            return 1.0f + c3 * std::pow(t - 1.0f, 3.0f) + c1 * std::pow(t - 1.0f, 2.0f);
        }
    }
    return t;
}

struct Keyframe {
    int  frame = 0;
    Vec3 pos   = {0, 0, 0};
    Vec3 rot   = {0, 0, 0};
    Vec3 scale = {1, 1, 1};
    bool has_pos   = false;
    bool has_rot   = false;
    bool has_scale = false;
    Ease ease = Ease::EaseInOut;  // v0.0.4: per-keyframe easing
};

struct TransformSample {
    Vec3 pos   = {0, 0, 0};
    Vec3 rot   = {0, 0, 0};
    Vec3 scale = {1, 1, 1};
    bool has_pos   = false;
    bool has_rot   = false;
    bool has_scale = false;
};

inline TransformSample sample_animation(const std::vector<Keyframe>& kfs, int frame) {
    TransformSample out;
    if (kfs.empty()) return out;

    if (kfs.size() == 1) {
        const auto& kf = kfs[0];
        out.pos = kf.pos; out.rot = kf.rot; out.scale = kf.scale;
        out.has_pos = kf.has_pos; out.has_rot = kf.has_rot; out.has_scale = kf.has_scale;
        return out;
    }

    const Keyframe* k0 = &kfs[0];
    const Keyframe* k1 = &kfs[kfs.size() - 1];

    for (size_t i = 0; i + 1 < kfs.size(); ++i) {
        if (kfs[i].frame <= frame && kfs[i+1].frame >= frame) {
            k0 = &kfs[i];
            k1 = &kfs[i+1];
            break;
        }
    }

    if (frame <= kfs.front().frame) {
        k0 = &kfs.front();
        k1 = &kfs.front();
    }
    if (frame >= kfs.back().frame) {
        k0 = &kfs.back();
        k1 = &kfs.back();
    }

    float span = (float)(k1->frame - k0->frame);
    float t = (span > 0.0f) ? (float)(frame - k0->frame) / span : 0.0f;

    // v0.0.4: apply easing from k0 (the keyframe we're leaving).
    t = apply_ease(t, k0->ease);

    out.pos   = k0->pos   + (k1->pos   - k0->pos)   * t;
    out.rot   = k0->rot   + (k1->rot   - k0->rot)   * t;
    out.scale = k0->scale + (k1->scale - k0->scale) * t;
    out.has_pos   = k0->has_pos   || k1->has_pos;
    out.has_rot   = k0->has_rot   || k1->has_rot;
    out.has_scale = k0->has_scale || k1->has_scale;
    return out;
}

} // namespace pencil
