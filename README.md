# legio-drifter

Random Bézier auto-panner / crossfader / CV panner firmware for the [Noise Engineering Legio](https://noiseengineering.us/) Eurorack platform.

## Controls

- **Left switch:** AUDIO PAN (up) / XFADE (center) / CV PAN (down).
  - AUDIO PAN makes the module act as an audio panner
  - XFADE turns it into a crossfader (mono output on L)
  - CV PAN makes the module act as a CV panner for CV sent to the V/OCT input jack
- **Top knob + CV:** where the image sits (left … right / A … B).
- **Bottom knob + CV:** how far the random wander pushes it.
- **Right switch:** edge behavior when the image is pushed past the rails: CLIP / FOLD / WRAP.
- **Encoder:** rate (free: 5 min … 20 Hz; clocked: ÷8 … ×8). Tap to edit the curve shape
  instead (LEDs turn yellow). Long-press in CV mode to use an internal +5 V source and turn
  the outputs into a complementary pair of slow random voltages.
- **Gate in:** clock. Each qualifying edge lands a new random target on the beat.

How the right switch maps an out-of-range position back into the field:

![Edge behavior: CLIP, FOLD and WRAP transfer curves with sample input/output values](docs/img/edge-behavior.svg)

Build: `make -C lib/libDaisy && make`. Tests: `make -C test`. Flash: `make program-dfu`.
See [`AGENTS.md`](AGENTS.md) for layout, behavior notes, and Legio hardware quirks.

## Calibrating the v/oct input for your module

Only **CV mode** reads the v/oct jack (as a plain DC input, not as pitch). The Patch SM's ADC
offset and gain vary slightly per unit; `src/cv_in.h` holds `kCvZero` / `kCvScale` measured on the
author's module. If CV mode reads a patched 0 V as slightly non-zero, or +5 V as not-quite-5 V:

1. Flash, then open serial telemetry: `screen /dev/cu.usbmodem* 115200`. Each line includes
   `cv_norm=<f>` (raw 0…1 ADC reading) and `cv=<f>V` (converted with the current constants).
2. Patch a known **0 V** into the v/oct jack; note `cv_norm` → `kCvZero`.
3. Patch a known **+1 V**; note `cv_norm` → `raw_1v`. `kCvScale = 1.0 / (raw_1v - kCvZero)`.
4. `cp src/calibration_local.h.example src/calibration_local.h`, put your two values in it, rebuild,
   reflash. The file is gitignored, so your values survive `git pull`. (Editing the defaults in
   `src/cv_in.h` also works.)

`kCvScale = 0` disables the path (every reading becomes 0 V). A ±0.05 V deadband around 0 V is
applied after conversion so a patched 0 V reads as exactly 0.

## License

MIT — see [`LICENSE`](LICENSE). libDaisy (submodule) is MIT-licensed by Electrosmith.
