// src/bezier_random.cpp
#include "bezier_random.h"
#include <cmath>

namespace legio {

namespace {
constexpr int   kMaxSolveIters = 16;
constexpr float kSolveTol      = 1e-6f;

inline float Bez(float p0, float p1, float p2, float p3, float s) {
    float u = 1.0f - s;
    return u * u * u * p0 + 3.0f * u * u * s * p1 + 3.0f * u * s * s * p2 + s * s * s * p3;
}
inline float BezDeriv(float p0, float p1, float p2, float p3, float s) {
    float u = 1.0f - s;
    return 3.0f * u * u * (p1 - p0) + 6.0f * u * s * (p2 - p1) + 3.0f * s * s * (p3 - p2);
}
}  // namespace

void BezierRandom::Init(float sample_rate, uint32_t seed) {
    sample_rate_ = sample_rate;
    rng_         = seed ? seed : 0x9E3779B9u;   // xorshift must never hold 0
    period_      = 10.0f * sample_rate;
    curve_       = 0.0f;
    phase_       = 0.0f;
    s_           = 0.0f;
    v_cur_       = 0.0f;
    v_next_      = NextRandom();
    value_       = 0.0f;
    solve_error_ = 0.0f;
    new_target_  = false;
    synced_      = false;
}

float BezierRandom::NextRandom() {
    uint32_t x = rng_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_ = x;
    // Top 24 bits -> [0, 1) -> [-1, +1).
    return (float)(x >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

void BezierRandom::SetPeriodSamples(float p) { period_ = p < 1.0f ? 1.0f : p; }

void BezierRandom::SetCurve(float c) {
    curve_ = c < -1.0f ? -1.0f : (c > 1.0f ? 1.0f : c);
}

void BezierRandom::SetEndpoints(float v_cur, float v_next) {
    v_cur_  = v_cur;
    v_next_ = v_next;
    value_  = v_cur;
    phase_  = 0.0f;
    s_      = 0.0f;
}

void BezierRandom::DrawTarget() {
    v_cur_  = v_next_;
    v_next_ = NextRandom();
    s_      = 0.0f;
}

void BezierRandom::Sync() {
    v_cur_  = value_;          // continue from where we are: no jump
    v_next_ = NextRandom();
    phase_  = 0.0f;
    s_      = 0.0f;
    synced_ = true;
}

float BezierRandom::Process(int block_size) {
    bool nt = synced_;
    synced_ = false;
    phase_ += (float)block_size / period_;
    if (phase_ >= 1.0f) {
        phase_ -= std::floor(phase_);
        DrawTarget();
        nt = true;
    }
    new_target_ = nt;
    value_      = Evaluate(phase_);
    return value_;
}

void BezierRandom::ControlPoints(float& x1, float& y1, float& x2, float& y2) const {
    float d = v_next_ - v_cur_;
    if (curve_ < 0.0f) {
        // CCW: control points move vertically. Fast-slow-fast, cusps at the targets.
        float k = 0.5f * -curve_;
        x1 = 0.0f; y1 = v_cur_ + k * d;
        x2 = 1.0f; y2 = v_next_ - k * d;
    } else {
        // CW: control points move horizontally. Slow-fast-slow, plateaus at the targets.
        float k = 0.5f * curve_;
        x1 = k;        y1 = v_cur_;
        x2 = 1.0f - k; y2 = v_next_;
    }
}

float BezierRandom::Evaluate(float phi) {
    if (curve_ == 0.0f) {
        s_           = phi;
        solve_error_ = 0.0f;
        return v_cur_ + (v_next_ - v_cur_) * phi;
    }
    float x1, y1, x2, y2;
    ControlPoints(x1, y1, x2, y2);

    // Solve x(s) = phi. x(s) is monotone non-decreasing on [0, 1] because
    // 0 <= x1 <= x2 <= 1, so a bracket [lo, hi] always contains the root.
    // Newton from the previous s (phi only grows within a cycle, so the warm
    // start is already close); bisect whenever Newton would leave the bracket
    // or the slope vanishes (it is exactly 0 at s = 0 and s = 1 for curve < 0).
    float lo = 0.0f, hi = 1.0f;
    float s  = s_ < 0.0f ? 0.0f : (s_ > 1.0f ? 1.0f : s_);
    float f  = Bez(0.0f, x1, x2, 1.0f, s) - phi;
    for (int i = 0; i < kMaxSolveIters && std::fabs(f) > kSolveTol; ++i) {
        if (f < 0.0f) lo = s; else hi = s;
        float d    = BezDeriv(0.0f, x1, x2, 1.0f, s);
        float cand = (d > 1e-6f) ? s - f / d : -1.0f;
        s = (cand > lo && cand < hi) ? cand : 0.5f * (lo + hi);
        f = Bez(0.0f, x1, x2, 1.0f, s) - phi;
    }
    s_           = s;
    solve_error_ = std::fabs(f);
    return Bez(v_cur_, y1, y2, v_next_, s);
}

}  // namespace legio
