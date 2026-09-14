# Drifter

Random Bézier auto-panner / crossfader / CV panner firmware for the Noise Engineering Legio.

- **Left switch:** PAN (up) / XFADE (center) / CV (down).
- **Right switch:** edge behavior when the image is pushed past the rails: CLIP / FOLD / WRAP.
- **Top knob + CV:** where the image sits (left … right / A … B).
- **Bottom knob + CV:** how far the random wander pushes it.
- **Encoder:** rate (free: 5 min … 20 Hz; clocked: ÷8 … ×8). Tap to edit the curve shape
  instead (LEDs turn yellow). Long-press in CV mode to use an internal +5 V source and turn
  the outputs into a complementary pair of slow random voltages.
- **Gate in:** clock. Each qualifying edge lands a new random target on the beat.

Build: `make -C lib/libDaisy && make`. Tests: `make -C test`. Flash: `make program-dfu`.
