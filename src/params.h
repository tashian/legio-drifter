// src/params.h
#pragma once

#include "cv_in.h"

namespace legio {

// Left switch, panel-relative: up = PAN, center = XFADE, down = CV.
enum class Mode { PAN = 0, XFADE = 1, CV = 2 };

// Right switch, panel-relative: up = CLIP, center = FOLD, down = WRAP.
enum class Edge { CLIP = 0, FOLD = 1, WRAP = 2 };

struct Params {
    // Continuous controls, 0..1 from the ADC (knob + CV jack summed in hardware).
    float top_knob    = 0.5f;     // center: 0 = left / A, 1 = right / B
    float bottom_knob = 0.0f;     // depth:  0 = plain CV panner, 1 = full-field wander
    float cv_norm     = kCvZero;  // raw v/oct ADC 0..1 (default = calibrated 0 V)

    // Switches.
    Mode mode = Mode::PAN;
    Edge edge = Edge::FOLD;

    // Encoder. main.cpp turns raw presses into these clean edges.
    int  encoder_increment  = 0;      // -1 / 0 / +1 per block, 0 while pressed
    bool encoder_tap        = false;  // press-and-release < 400 ms (one block)
    bool encoder_long_press = false;  // held >= 800 ms, fires once (one block)

    // Gate jack rising edge this block.
    bool gate_edge = false;
};

}  // namespace legio
