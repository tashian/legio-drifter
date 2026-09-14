// src/drifter_chain.h — orchestrator. Owns the generator and the panner, holds the
// rate / ratio / curve / edit-mode / internal-source state, and turns Params into
// a smoothed position and mixed outputs once per block. Pure float, no HAL.
#pragma once
#include <cstdint>
#include "bezier_random.h"
#include "clock.h"
#include "panner.h"
#include "params.h"

namespace legio {

class DrifterChain {
  public:
    static constexpr int kRateSteps  = 38;   // free-running: 5 min (0) .. 20 Hz (38), ~1/3 oct per detent
    static constexpr int kRatioCount = 7;    // clocked: ÷8 ÷4 ÷2 ×1 ×2 ×4 ×8
    static constexpr int kCurveSteps = 12;   // curve = steps / 12, steps in -12..+12

    static float FreePeriodSeconds(int rate_index);
    static float RatioMultiplier(int ratio_index);   // period = clock period * this

    void Init(float sample_rate, uint32_t seed);
    void ApplyParams(const Params& p, const Clock& clk, int block_size);
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, int n);

    // Observers for LEDs / telemetry.
    float position()               const { return panner_.smoothed_position(); }
    bool  new_target()             const { return gen_.new_target(); }
    bool  curve_edit()             const { return curve_edit_; }
    float curve()                  const { return gen_.curve(); }
    bool  internal_source()        const { return internal_source_; }
    bool  internal_source_active() const { return mode_ == Mode::CV && internal_source_; }
    float period_samples()         const { return period_; }
    bool  clocked()                const { return clocked_; }
    int   rate_index()             const { return rate_index_; }
    int   ratio_index()            const { return ratio_index_; }
    float center()                 const { return center_; }
    float depth()                  const { return depth_; }
    float wander()                 const { return gen_.value(); }
    float cv_volts()               const { return cv_volts_; }

  private:
    BezierRandom gen_;
    Panner       panner_;
    float        sample_rate_;
    Mode         mode_;
    int          rate_index_;
    int          ratio_index_;
    int          curve_steps_;
    bool         curve_edit_;
    bool         internal_source_;
    int          div_counter_;      // edges seen since the last ÷N sync
    float        period_;
    bool         clocked_;
    float        center_;
    float        depth_;
    float        cv_volts_;
};

}  // namespace legio
