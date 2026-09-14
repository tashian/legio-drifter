// src/clock.h — gate-input clock: edge detect, smoothed period, external / fallback state.
// Fallback rule: while external, free-run after max(4 periods, 2 s) of silence; while
// acquiring, a lone first edge is kept for up to the 20 s max period.
#pragma once

namespace legio {

class Clock {
  public:
    void  Init(float sample_rate);
    void  update(bool gate_edge, int block_size);
    float period_samples() const;
    bool  tick() const;          // true on the block where an edge arrived
    bool  is_external() const;   // true while edges keep arriving

  private:
    float sample_rate_;
    float fallback_period_;
    float silence_floor_;        // 2 s in samples
    float min_period_;
    float max_period_;
    float period_;
    int   samples_since_edge_;
    int   samples_to_last_edge_;
    bool  have_first_edge_;
    bool  is_external_;
    bool  tick_;
    int   edge_count_;
};

}  // namespace legio
