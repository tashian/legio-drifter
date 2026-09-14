// src/dsp_common.h
#pragma once

namespace legio {

constexpr float kSampleRate     = 48000.0f;
constexpr int   kAudioBlockSize = 48; // samples per channel per callback

}  // namespace legio

// On host builds the Daisy SDRAM section attribute doesn't exist; stub it.
// Drifter has no big buffers, but every app in the workspace carries this shim.
#ifndef DSY_SDRAM_BSS
#  define DSY_SDRAM_BSS
#endif
