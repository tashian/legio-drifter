// src/main.cpp — placeholder HAL. Replaced wholesale in Task 10.
#include "daisy_legio.h"
#include "dsp_common.h"

using namespace daisy;

DaisyLegio hw;

static void PassthroughCallback(AudioHandle::InterleavingInputBuffer  in,
                                AudioHandle::InterleavingOutputBuffer out,
                                size_t                                size) {
    for (size_t i = 0; i < size; i += 2) {
        out[i]     = in[i];
        out[i + 1] = in[i + 1];
    }
}

int main() {
    hw.Init();
    hw.seed.StartLog(false);
    hw.StartAdc();
    hw.StartAudio(PassthroughCallback);

    uint32_t last_log = 0;
    while (1) {
        uint32_t now = System::GetNow();
        hw.SetLed(DaisyLegio::LED_LEFT,  0.3f, 0.3f, 0.3f);
        hw.SetLed(DaisyLegio::LED_RIGHT, 0.3f, 0.3f, 0.3f);
        hw.UpdateLeds();
        if (now - last_log >= 1000) {
            last_log = now;
            hw.seed.PrintLine("drifter skeleton: passthrough, sr=%.0f", legio::kSampleRate);
        }
    }
}
