// src/bezier_random.cpp
#include "bezier_random.h"
#include <cmath>

namespace legio {

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
    // Filled in by Task 6. Linear placeholder keeps the build green.
    x1 = 0.0f; y1 = v_cur_;
    x2 = 1.0f; y2 = v_next_;
}

float BezierRandom::Evaluate(float phi) {
    // Task 5: linear only. Task 6 adds the Bézier families.
    s_           = phi;
    solve_error_ = 0.0f;
    return v_cur_ + (v_next_ - v_cur_) * phi;
}

}  // namespace legio
