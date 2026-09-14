// src/panner.cpp
#include "panner.h"
#include <cmath>

namespace legio {

namespace {
constexpr float kPosSmoothSec = 0.005f;   // ~5 ms one-pole on position
constexpr float kCvSmoothSec  = 0.002f;   // ~2 ms one-pole on the CV source
constexpr float kHalfPi       = 1.57079632679f;

inline float OnePoleAlpha(float seconds, float sample_rate) {
    return 1.0f - std::exp(-1.0f / (seconds * sample_rate));
}
inline float Clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
}  // namespace

void Panner::Init(float sample_rate) {
    mode_      = Mode::PAN;
    internal_  = false;
    pos_alpha_ = OnePoleAlpha(kPosSmoothSec, sample_rate);
    cv_alpha_  = OnePoleAlpha(kCvSmoothSec, sample_rate);
    SetPositionImmediate(0.5f);
    cv_target_ = 0.0f;
    cv_s_      = 0.0f;
}

void Panner::SetPositionTarget(float pos)    { pos_target_ = Clamp01(pos); }
void Panner::SetPositionImmediate(float pos) { pos_target_ = pos_s_ = Clamp01(pos); }
void Panner::SetCvSource(float source)       { cv_target_ = source; }

void Panner::EqualPowerGains(float pos, float& g_l, float& g_r) {
    float a = pos * kHalfPi;
    g_l = std::cos(a);
    g_r = std::sin(a);
}

void Panner::ProcessBlock(const float* in_l, const float* in_r,
                          float* out_l, float* out_r, int n) {
    for (int i = 0; i < n; ++i) {
        pos_s_ += pos_alpha_ * (pos_target_ - pos_s_);
        cv_s_  += cv_alpha_  * (cv_target_  - cv_s_);
        switch (mode_) {
            case Mode::PAN: {
                float gl, gr;
                EqualPowerGains(pos_s_, gl, gr);
                out_l[i] = in_l[i] * gl;
                out_r[i] = in_r[i] * gr;
                break;
            }
            case Mode::XFADE: {
                float ga, gb;
                EqualPowerGains(pos_s_, ga, gb);
                out_l[i] = in_l[i] * ga + in_r[i] * gb;   // the crossfade
                out_r[i] = in_l[i] * gb + in_r[i] * ga;   // the complementary crossfade
                break;
            }
            case Mode::CV:
            default: {
                // Linear law so the two outputs always sum to the source: an envelope
                // split across two destinations keeps its total "presence".
                float src = internal_ ? 1.0f : cv_s_;   // 1.0 = +5 V at the DC-coupled output
                out_l[i] = src * (1.0f - pos_s_);
                out_r[i] = src * pos_s_;
                break;
            }
        }
    }
}

}  // namespace legio
