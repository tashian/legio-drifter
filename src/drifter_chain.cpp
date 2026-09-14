// src/drifter_chain.cpp
#include "drifter_chain.h"
#include <cmath>
#include "cv_in.h"
#include "edge.h"

namespace legio {

namespace {
constexpr float kFreePeriodMaxSec   = 300.0f;   // 5 minutes per target
constexpr float kFreePeriodMinSec   = 0.05f;    // 20 Hz
constexpr int   kDefaultRateIndex   = 15;       // ~9.7 s
constexpr int   kDefaultRatioIndex  = 3;        // ×1
constexpr int   kDefaultCurveSteps  = 6;        // curve = +0.5, eased
// Index:            ÷8    ÷4    ÷2    ×1    ×2    ×4     ×8
const float kRatioMult[DrifterChain::kRatioCount] = {8.0f, 4.0f, 2.0f, 1.0f, 0.5f, 0.25f, 0.125f};
const int   kRatioDiv [DrifterChain::kRatioCount] = {8,    4,    2,    1,    1,    1,     1};

inline int ClampInt(int x, int lo, int hi) { return x < lo ? lo : (x > hi ? hi : x); }
}  // namespace

float DrifterChain::FreePeriodSeconds(int rate_index) {
    float t = (float)ClampInt(rate_index, 0, kRateSteps) / (float)kRateSteps;
    return kFreePeriodMaxSec * std::pow(kFreePeriodMinSec / kFreePeriodMaxSec, t);
}

float DrifterChain::RatioMultiplier(int ratio_index) {
    return kRatioMult[ClampInt(ratio_index, 0, kRatioCount - 1)];
}

void DrifterChain::Init(float sample_rate, uint32_t seed) {
    sample_rate_     = sample_rate;
    gen_.Init(sample_rate, seed);
    panner_.Init(sample_rate);
    mode_            = Mode::PAN;
    rate_index_      = kDefaultRateIndex;
    ratio_index_     = kDefaultRatioIndex;
    curve_steps_     = kDefaultCurveSteps;
    curve_edit_      = false;
    internal_source_ = false;
    div_counter_     = 0;
    clocked_         = false;
    period_          = FreePeriodSeconds(rate_index_) * sample_rate_;
    center_          = 0.5f;
    depth_           = 0.0f;
    cv_volts_        = 0.0f;
    gen_.SetPeriodSamples(period_);
    gen_.SetCurve((float)curve_steps_ / (float)kCurveSteps);
}

void DrifterChain::ApplyParams(const Params& p, const Clock& clk, int block_size) {
    mode_    = p.mode;
    clocked_ = clk.is_external();

    // --- Encoder events. Tap toggles edit mode; long press toggles the internal
    //     source in CV mode only; rotation edits exactly one of rate / ratio / curve.
    if (p.encoder_tap) curve_edit_ = !curve_edit_;
    if (p.encoder_long_press && mode_ == Mode::CV) internal_source_ = !internal_source_;
    if (p.encoder_increment != 0) {
        if (curve_edit_) {
            curve_steps_ = ClampInt(curve_steps_ + p.encoder_increment, -kCurveSteps, kCurveSteps);
        } else if (clocked_) {
            ratio_index_ = ClampInt(ratio_index_ + p.encoder_increment, 0, kRatioCount - 1);
            div_counter_ = 0;
        } else {
            rate_index_ = ClampInt(rate_index_ + p.encoder_increment, 0, kRateSteps);
        }
    }

    // --- Period: clocked = measured clock period × ratio; free = encoder log scale.
    period_ = clocked_ ? clk.period_samples() * RatioMultiplier(ratio_index_)
                       : FreePeriodSeconds(rate_index_) * sample_rate_;
    gen_.SetPeriodSamples(period_);
    gen_.SetCurve((float)curve_steps_ / (float)kCurveSteps);

    // --- Clock sync: every edge for ×N, every Nth edge for ÷N.
    if (clocked_ && clk.tick()) {
        if (++div_counter_ >= kRatioDiv[ratio_index_]) {
            div_counter_ = 0;
            gen_.Sync();
        }
    }
    if (!clocked_) div_counter_ = 0;

    // --- Position for this block.
    center_ = p.top_knob;
    depth_  = p.bottom_knob;
    float r       = gen_.Process(block_size);
    float pos_raw = center_ + 0.5f * depth_ * r;
    panner_.SetPositionTarget(apply_edge(p.edge, pos_raw));

    // --- CV source and mode.
    cv_volts_ = cv_volts_from_norm(p.cv_norm);
    panner_.SetCvSource(cv_source_from_volts(cv_volts_));
    panner_.SetInternalSource(internal_source_);
    panner_.SetMode(mode_);
}

void DrifterChain::ProcessBlock(const float* in_l, const float* in_r,
                                float* out_l, float* out_r, int n) {
    panner_.ProcessBlock(in_l, in_r, out_l, out_r, n);
}

}  // namespace legio
