// src/cv_in.h — v/oct jack calibration and conversion for CV mode.
//
// The calibration constants are per-module: the Patch SM's ADC offset/gain
// vary slightly between units. Defaults below were measured on the author's
// module (0 V read 0.3019, +1 V read 0.4321, linear to ~1 LSB at -2 V). To use
// your own values without touching tracked files, copy
// calibration_local.h.example to calibration_local.h (gitignored) and edit it;
// see README "Calibrating the v/oct input". kCvScale = 0 disables the path
// entirely (every norm reads as 0 V).
#pragma once
#include <cmath>

namespace legio {

#if __has_include("calibration_local.h")
#include "calibration_local.h"
#else
constexpr float kCvZero     = 0.3019f;   // raw ADC at 0 V
constexpr float kCvScale    = 7.6805f;   // volts per raw unit = 1 / (0.4321 - 0.3019)
#endif
constexpr float kCvDeadband = 0.05f;     // volts; |volts| below this reads as exactly 0

// Raw ADC (0..1) -> volts. The deadband is a hard gate, not a subtraction:
// a value just above the threshold passes through unchanged.
inline float cv_volts_from_norm(float norm,
                                float zero     = kCvZero,
                                float scale    = kCvScale,
                                float deadband = kCvDeadband) {
    if (scale == 0.0f) return 0.0f;
    float volts = (norm - zero) * scale;
    return std::fabs(volts) < deadband ? 0.0f : volts;
}

// 1.0 float at the DC-coupled output = +5 V.
inline float cv_source_from_volts(float volts) { return volts * 0.2f; }

}  // namespace legio
