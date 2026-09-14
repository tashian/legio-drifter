// src/edge.h — position edge behavior (right switch). Borrowed from the
// LIMIT / FOLD / THRU edge behaviours. All three map any float onto [0, 1].
#pragma once
#include <cmath>
#include "params.h"

namespace legio {

// Clamp to [0, 1]: the image parks at the rail.
inline float edge_clip(float x) {
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

// Triangle map with period 2: reflect about 0 and 1. The image bounces.
inline float edge_fold(float x) {
    float t = std::fmod(x, 2.0f);
    if (t < 0.0f) t += 2.0f;          // now 0 <= t < 2
    return t > 1.0f ? 2.0f - t : t;
}

// x mod 1: the image snaps across the field.
inline float edge_wrap(float x) {
    float t = x - std::floor(x);
    return t >= 1.0f ? 0.0f : t;       // guard float rounding at the seam
}

inline float apply_edge(Edge e, float x) {
    switch (e) {
        case Edge::CLIP: return edge_clip(x);
        case Edge::WRAP: return edge_wrap(x);
        case Edge::FOLD:
        default:         return edge_fold(x);
    }
}

}  // namespace legio
