// src/main.cpp — Drifter HAL. The ONLY file that includes daisy_legio.h.
//
// Per block: snapshot controls into Params (panel-relative switch labels, clean
// encoder edges), update the clock, run the chain, publish a volatile UI snapshot.
// The slow loop turns the snapshot into LEDs and 5 Hz serial telemetry.
#include "daisy_legio.h"
#include "clock.h"
#include "drifter_chain.h"
#include "dsp_common.h"
#include "params.h"

using namespace daisy;
using legio::Clock;
using legio::DrifterChain;
using legio::Edge;
using legio::Mode;
using legio::Params;

DaisyLegio   hw;
Clock        clock;
DrifterChain chain;

namespace {

constexpr uint32_t kTapMaxMs     = 400;   // release before this = tap
constexpr uint32_t kLongPressMs  = 800;   // hold past this = long press (fires once, while held)
constexpr uint32_t kFlashMs      = 50;    // yellow event flash length
constexpr float    kInternalYellowFloor = 0.15f;

// Encoder press state, carried across audio callbacks.
bool     s_press_armed = false;
bool     s_long_fired  = false;
uint32_t s_press_start = 0;

struct UiSnapshot {
    float pos;              // smoothed position 0..1
    float center;
    float depth;
    float period_s;
    bool  clocked;
    float curve;
    bool  curve_edit;
    bool  internal_active;  // CV mode && internal source on
    bool  new_target_pulse; // set by the callback, cleared by the slow loop
    bool  clock_pulse;      // same, for gate edges
    int   mode;             // (int)Mode
    int   edge;             // (int)Edge
    float cv_norm;
    float cv_volts;
};
volatile UiSnapshot ui_state{0.5f, 0.5f, 0.0f, 10.0f, false, 0.5f, false, false,
                             false, false, 0, 1, 0.3019f, 0.0f};

const char* ModeName(int m) { return m == 0 ? "PAN" : m == 1 ? "XFADE" : "CV"; }
const char* EdgeName(int e) { return e == 0 ? "CLIP" : e == 1 ? "FOLD" : "WRAP"; }

}  // namespace

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size) {
    hw.ProcessAllControls();
    const int      n   = (int)(size / 2);
    const uint32_t now = System::GetNow();

    Params p;

    // --- Encoder: tap (< 400 ms release) / long press (>= 800 ms, fires once while held).
    //     400..800 ms is dead. Rotation while pressed is ignored.
    if (hw.encoder.RisingEdge()) {
        s_press_armed = true;
        s_long_fired  = false;
        s_press_start = now;
    }
    if (hw.encoder.Pressed() && s_press_armed && !s_long_fired
        && (now - s_press_start) >= kLongPressMs) {
        p.encoder_long_press = true;
        s_long_fired         = true;
    }
    if (hw.encoder.FallingEdge()) {
        if (s_press_armed && !s_long_fired && (now - s_press_start) < kTapMaxMs) {
            p.encoder_tap = true;
        }
        s_press_armed = false;
    }
    int inc = (int)hw.encoder.Increment();
    p.encoder_increment = hw.encoder.Pressed() ? 0 : inc;

    // --- Knobs (knob + CV summed in hardware) and the v/oct jack.
    p.top_knob    = hw.GetKnobValue(DaisyLegio::CONTROL_KNOB_TOP);
    p.bottom_knob = hw.GetKnobValue(DaisyLegio::CONTROL_KNOB_BOTTOM);
    p.cv_norm     = hw.controls[DaisyLegio::CONTROL_PITCH].Value();

    // --- Switches. Switch3.Read() returns 0=CENTER, 1=POS_UP, 2=POS_DOWN, and on
    //     Legio's panel the lib's labels are inverted: 1 = panel DOWN, 2 = panel UP.
    int swl = hw.sw[DaisyLegio::SW_LEFT].Read();
    p.mode = (swl == 2) ? Mode::PAN          // panel up
           : (swl == 1) ? Mode::CV           // panel down
           :              Mode::XFADE;       // center
    int swr = hw.sw[DaisyLegio::SW_RIGHT].Read();
    p.edge = (swr == 2) ? Edge::CLIP         // panel up
           : (swr == 1) ? Edge::WRAP         // panel down
           :              Edge::FOLD;        // center

    // --- Clock and chain.
    bool gate_edge = hw.gate.Trig();
    p.gate_edge    = gate_edge;
    clock.update(gate_edge, n);

    chain.ApplyParams(p, clock, n);

    // De-interleave, process, re-interleave. n <= 48 always.
    float in_l[legio::kAudioBlockSize], in_r[legio::kAudioBlockSize];
    float out_l[legio::kAudioBlockSize], out_r[legio::kAudioBlockSize];
    for (int i = 0; i < n; ++i) {
        in_l[i] = in[2 * i];
        in_r[i] = in[2 * i + 1];
    }
    chain.ProcessBlock(in_l, in_r, out_l, out_r, n);
    for (int i = 0; i < n; ++i) {
        out[2 * i]     = out_l[i];
        out[2 * i + 1] = out_r[i];
    }

    // --- UI snapshot (never PrintLine here).
    ui_state.pos             = chain.position();
    ui_state.center          = chain.center();
    ui_state.depth           = chain.depth();
    ui_state.period_s        = chain.period_samples() / legio::kSampleRate;
    ui_state.clocked         = chain.clocked();
    ui_state.curve           = chain.curve();
    ui_state.curve_edit      = chain.curve_edit();
    ui_state.internal_active = chain.internal_source_active();
    ui_state.mode            = (int)p.mode;
    ui_state.edge            = (int)p.edge;
    ui_state.cv_norm         = p.cv_norm;
    ui_state.cv_volts        = chain.cv_volts();
    if (chain.new_target()) ui_state.new_target_pulse = true;
    if (gate_edge)          ui_state.clock_pulse      = true;
}

static void BypassCallback(AudioHandle::InterleavingInputBuffer  in,
                           AudioHandle::InterleavingOutputBuffer out,
                           size_t                                size) {
    for (size_t i = 0; i < size; i += 2) {
        out[i]     = in[i];
        out[i + 1] = in[i + 1];
    }
}

static uint32_t SeedFromAdcNoise() {
    // Low byte of each of the three ADC channels + the millisecond tick.
    uint32_t a = hw.controls[DaisyLegio::CONTROL_KNOB_TOP].GetRawValue()    & 0xFF;
    uint32_t b = hw.controls[DaisyLegio::CONTROL_KNOB_BOTTOM].GetRawValue() & 0xFF;
    uint32_t c = hw.controls[DaisyLegio::CONTROL_PITCH].GetRawValue()       & 0xFF;
    uint32_t t = System::GetNow() & 0xFF;
    return a | (b << 8) | (c << 16) | (t << 24);
}

int main() {
    hw.Init();

    // Bypass mode: encoder held at boot.
    hw.ProcessAllControls();
    bool bypass_mode = hw.encoder.Pressed();

    hw.seed.StartLog(false);
    clock.Init(legio::kSampleRate);
    hw.StartAdc();
    System::Delay(20);                          // let the ADC DMA fill before seeding
    chain.Init(legio::kSampleRate, SeedFromAdcNoise());

    if (bypass_mode) hw.StartAudio(BypassCallback);
    else             hw.StartAudio(AudioCallback);

    uint32_t last_log = 0, target_flash_end = 0, clock_flash_end = 0;
    while (1) {
        uint32_t now = System::GetNow();

        if (bypass_mode) {
            hw.SetLed(DaisyLegio::LED_LEFT,  0.3f, 0.3f, 0.3f);
            hw.SetLed(DaisyLegio::LED_RIGHT, 0.3f, 0.3f, 0.3f);
            hw.UpdateLeds();
            if (now - last_log >= 1000) {
                last_log = now;
                hw.seed.PrintLine("BYPASS MODE - release encoder + reset to exit");
            }
            continue;
        }

        // Consume event pulses into 50 ms flashes.
        if (ui_state.new_target_pulse) { ui_state.new_target_pulse = false; target_flash_end = now + kFlashMs; }
        if (ui_state.clock_pulse)      { ui_state.clock_pulse      = false; clock_flash_end  = now + kFlashMs; }
        bool target_flash = (int32_t)(target_flash_end - now) > 0;
        bool clock_flash  = (int32_t)(clock_flash_end  - now) > 0;

        float pos   = ui_state.pos;
        float curve = ui_state.curve;

        if (ui_state.curve_edit) {
            // Yellow only: left bright = CCW / cusped, right bright = CW / eased, equal = linear.
            float yl = 0.5f * (1.0f - curve);
            float yr = 0.5f * (1.0f + curve);
            hw.SetLed(DaisyLegio::LED_LEFT,  yl, yl, 0.0f);
            hw.SetLed(DaisyLegio::LED_RIGHT, yr, yr, 0.0f);
        } else {
            // Blue = computed position; yellow flash = event; dim yellow floor = internal CV source.
            float floor = ui_state.internal_active ? kInternalYellowFloor : 0.0f;
            float yl = target_flash ? 1.0f : floor;
            float yr = clock_flash  ? 1.0f : floor;
            hw.SetLed(DaisyLegio::LED_LEFT,  yl, yl, 1.0f - pos);
            hw.SetLed(DaisyLegio::LED_RIGHT, yr, yr, pos);
        }
        hw.UpdateLeds();

        if (now - last_log >= 200) {
            last_log = now;
            hw.seed.PrintLine("mode=%s pos=%.2f ctr=%.2f dep=%.2f T=%.1fs ext=%d curve=%+.2f "
                              "edge=%s cv_norm=%.4f cv=%.3fV src=%s%s",
                              ModeName(ui_state.mode),
                              ui_state.pos, ui_state.center, ui_state.depth,
                              ui_state.period_s,
                              ui_state.clocked ? 1 : 0,
                              ui_state.curve,
                              EdgeName(ui_state.edge),
                              ui_state.cv_norm, ui_state.cv_volts,
                              ui_state.internal_active ? "int" : "jack",
                              ui_state.curve_edit ? " [curve edit]" : "");
        }
    }
}
