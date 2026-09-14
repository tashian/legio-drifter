// src/clock.cpp
#include "clock.h"

namespace legio {

namespace {
constexpr float kFallbackBpm      = 120.0f;
constexpr float kFallbackSeconds  = 60.0f / kFallbackBpm;   // 0.5 s (unused by drifter's chain)
constexpr float kSilenceFloorSec  = 2.0f;                   // minimum time before falling back
constexpr float kFallbackPeriods  = 4.0f;                   // ... or this many measured periods
constexpr float kSmoothingAlpha   = 0.3f;
constexpr float kMinPeriodSec     = 0.05f;                  // 20 Hz ceiling
constexpr float kMaxPeriodSec     = 20.0f;                  // 0.05 Hz floor (slow ambient clocks)
}

void Clock::Init(float sample_rate) {
    sample_rate_          = sample_rate;
    fallback_period_      = kFallbackSeconds * sample_rate_;
    silence_floor_        = kSilenceFloorSec * sample_rate_;
    min_period_           = kMinPeriodSec * sample_rate_;
    max_period_           = kMaxPeriodSec * sample_rate_;
    period_               = fallback_period_;
    samples_since_edge_   = 0;
    samples_to_last_edge_ = 0;
    have_first_edge_      = false;
    is_external_          = false;
    tick_                 = false;
    edge_count_           = 0;
}

void Clock::update(bool gate_edge, int block_size) {
    tick_ = gate_edge;
    if (gate_edge) {
        if (have_first_edge_) {
            float interval = (float)samples_to_last_edge_;
            if (interval >= min_period_ && interval <= max_period_) {
                if (edge_count_ == 0) {
                    period_ = interval;       // first measured interval = direct
                } else {
                    period_ += kSmoothingAlpha * (interval - period_);
                }
                ++edge_count_;
                is_external_ = true;
            }
        }
        have_first_edge_      = true;
        samples_to_last_edge_ = 0;
        samples_since_edge_   = 0;
    } else {
        samples_to_last_edge_ += block_size;
        samples_since_edge_   += block_size;
        // Saturate: hours of silence must not overflow int. 5*max_period_ exceeds the
        // worst-case fallback timeout of 4*max_period_.
        int cap = (int)(5.0f * max_period_);
        if (samples_to_last_edge_ > cap) samples_to_last_edge_ = cap;
        if (samples_since_edge_   > cap) samples_since_edge_   = cap;
        if (is_external_) {
            float timeout = kFallbackPeriods * period_;
            if (timeout < silence_floor_) timeout = silence_floor_;
            if ((float)samples_since_edge_ > timeout) {
                is_external_     = false;
                period_          = fallback_period_;
                edge_count_      = 0;
                have_first_edge_ = false;
            }
        } else if ((float)samples_to_last_edge_ > max_period_) {
            have_first_edge_ = false;   // a lone edge older than the slowest accepted clock is forgotten
        }
    }
}

float Clock::period_samples() const { return period_; }
bool  Clock::tick()           const { return tick_;  }
bool  Clock::is_external()    const { return is_external_; }

}  // namespace legio
