# legio-drifter

Random Bézier auto-panner / crossfader / CV panner firmware for the Noise Engineering Legio Eurorack module.

Part of a family of alternative firmwares for the [Noise Engineering Legio](https://noiseengineering.us/) platform (Daisy Patch SM, STM32H750):
[legio-sawstack](https://github.com/tashian/legio-sawstack) (supersaw oscillator) ·
[legio-stutterer](https://github.com/tashian/legio-stutterer) (stutter → tape delay → DJ filter) ·
[legio-drifter](https://github.com/tashian/legio-drifter) (random Bézier panner / crossfader / CV).
Flash any of them onto a Legio via DFU; the stock firmware can be restored from Noise Engineering's customer portal.

- **Left switch:** PAN (up) / XFADE (center) / CV (down).
- **Right switch:** edge behavior when the image is pushed past the rails: CLIP / FOLD / WRAP.
- **Top knob + CV:** where the image sits (left … right / A … B).
- **Bottom knob + CV:** how far the random wander pushes it.
- **Encoder:** rate (free: 5 min … 20 Hz; clocked: ÷8 … ×8). Tap to edit the curve shape
  instead (LEDs turn yellow). Long-press in CV mode to use an internal +5 V source and turn
  the outputs into a complementary pair of slow random voltages.
- **Gate in:** clock. Each qualifying edge lands a new random target on the beat.

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
