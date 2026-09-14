// src/bezier_random.h — random Bézier wander generator, one channel.
// Output is in [-1, +1]. Pure float math, no HAL.
#pragma once
#include <cstdint>

namespace legio {

class BezierRandom {
  public:
    void  Init(float sample_rate, uint32_t seed);
    void  SetPeriodSamples(float period_samples);   // read every block; clamped >= 1
    void  SetCurve(float curve);                    // -1 cusped .. 0 linear .. +1 plateau
    void  SetEndpoints(float v_cur, float v_next);  // test hook: fixes the cycle, phase -> 0
    void  Sync();                                   // phase -> 0, restart from current value toward a new target
    float Process(int block_size);                  // advance one block, return value

    bool  new_target()  const { return new_target_; }   // true for the one block a target was drawn
    float value()       const { return value_; }
    float phase()       const { return phase_; }
    float current()     const { return v_cur_; }
    float target()      const { return v_next_; }
    float curve()       const { return curve_; }
    float solve_error() const { return solve_error_; }  // |x(s) - phase| after the last solve

  private:
    float NextRandom();                 // uniform in [-1, +1)
    void  DrawTarget();                 // v_cur <- v_next, v_next <- random
    float Evaluate(float phi);          // value at time fraction phi of the current cycle
    void  ControlPoints(float& x1, float& y1, float& x2, float& y2) const;

    uint32_t rng_;
    float    sample_rate_;
    float    period_;
    float    curve_;
    float    phase_;
    float    s_;             // Bézier parameter, warm start for the solver
    float    v_cur_;
    float    v_next_;
    float    value_;
    float    solve_error_;
    bool     new_target_;
    bool     synced_;        // Sync() happened since the last Process()
};

}  // namespace legio
