// src/panner.h — mode + smoothed position -> per-sample gains and output mixing.
#pragma once
#include "params.h"

namespace legio {

class Panner {
  public:
    void Init(float sample_rate);

    void SetMode(Mode m)            { mode_ = m; }
    void SetInternalSource(bool on) { internal_ = on; }   // CV mode: +5 V constant instead of the jack

    void SetPositionTarget(float pos);      // per block; smoothed per sample (~5 ms)
    void SetPositionImmediate(float pos);   // snap (Init, tests)
    void SetCvSource(float source);         // per block, volts / 5; smoothed per sample (~2 ms)

    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, int n);

    float smoothed_position() const { return pos_s_; }

    // gL = cos(pos*pi/2), gR = sin(pos*pi/2). Center = -3 dB per side.
    static void EqualPowerGains(float pos, float& g_l, float& g_r);

  private:
    Mode  mode_;
    bool  internal_;
    float pos_target_;
    float pos_s_;
    float pos_alpha_;
    float cv_target_;
    float cv_s_;
    float cv_alpha_;
};

}  // namespace legio
