# CLAUDE.md — drifter

Random Bézier auto-panner / crossfader / CV panner for the Noise Engineering Legio
(Daisy Patch SM, STM32H750). One switch picks what the module is (PAN / XFADE / CV),
the top knob is "where", the bottom knob is "how much wander", the encoder is "how fast".

Spec: `docs/superpowers/specs/2026-09-13-drifter-random-panner-design.md`
Plan: `docs/superpowers/plans/2026-09-13-drifter-random-panner.md` (historical — used to
bootstrap; deviations are recorded at the bottom of the plan and in this file).

## Layout

```
src/
  main.cpp                 HAL wiring + slow loop. ONLY file that includes daisy_legio.h.
  drifter_chain.{h,cpp}    Orchestrator: rate / ratio / curve-edit / internal-source state,
                           ApplyParams + ProcessBlock.
  bezier_random.{h,cpp}    Random Bézier wander generator, -1..+1.
  panner.{h,cpp}           Smoothed position -> gains + mixing for PAN / XFADE / CV.
  clock.{h,cpp}            Edge detect + smoothed period; fallback after max(4 periods, 2 s).
  edge.h                   clip / fold / wrap.
  cv_in.h                  v/oct calibration constants + norm -> volts -> source.
  params.h                 Carrier struct between main.cpp and the chain (panel-relative labels).
  dsp_common.h             kSampleRate, kAudioBlockSize, DSY_SDRAM_BSS host shim.
test/
  test_*.cpp               One per module + test_chain. Run with `make -C test`.
  test_assert.h            Tiny EXPECT_* macros — no external test framework.
lib/libDaisy               Submodule. No DaisySP (not needed).
```

## Build / test / flash

```sh
make -C lib/libDaisy   # one-time after submodule init
make                   # firmware → build/drifter.bin
make -C test           # host DSP tests (no hardware)
make program-dfu       # flash (module must be in DFU mode first)
```

DFU entry on Legio: BOOT + RESET on the Patch SM submodule (back of the module). The user
can't always do this without disturbing the patch — be patient at flash gates. dfu-util's
`Error 74` / "Error during download get_status" after flashing is harmless; the module may
need a reseat before `/dev/cu.usbmodem*` reappears.

Live serial: `screen /dev/cu.usbmodem* 115200`. If `screen` is attached elsewhere, `cat`
can't open the port — check `screen -ls`. Telemetry at 5 Hz:
`mode=PAN pos=0.42 ctr=0.50 dep=0.30 T=12.0s ext=0 curve=+0.35 edge=FOLD cv_norm=0.3021 cv=0.000V src=jack`

## Hardware quirks (workspace lessons — see ../CLAUDE.md for the full list)

1. **3 ADC channels.** Top/bottom knobs each read knob + CV jack summed. Treated as a feature.
2. **Switch3.Read() polarity is inverted.** lib `1` = panel DOWN, lib `2` = panel UP. Inverted
   in `main.cpp`. Don't "fix" it.
3. **`-u _printf_float`** is in the Makefile. Remove it and float telemetry goes silent.
4. **Audio inputs are AC coupled**, outputs are DC coupled (±5 V). CV mode writes DC to the
   outputs on purpose. In R is normalled to In L in hardware.
5. **Never `PrintLine` in the audio callback.** Telemetry goes through `ui_state`.

## Behavior notes

- Boot defaults: free-running period ≈ 10 s, clock ratio ×1, curve = +0.5 (eased), internal
  CV source off, curve-edit mode off. Nothing persists across power cycles.
- Encoder: tap (< 400 ms) toggles curve-edit mode; long press (≥ 800 ms, fires at 800 ms)
  toggles the internal +5 V source in CV mode only; 400–800 ms is dead; rotation while
  pressed is ignored.
- Bypass mode = encoder held during boot. Pure passthrough, both LEDs dim white.
- LEDs: blue = computed position (left = 1 − pos, right = pos); yellow flash on the left
  = new random target; yellow flash on the right = incoming clock edge; curve-edit mode
  shows curve in yellow only; internal CV source adds a dim yellow floor.

## CV-mode calibration

Constants in `src/cv_in.h` were reused from sawstack (same Patch SM, measured 2026-05-10):
`kCvZero = 0.3019`, `kCvScale = 7.6805`. Measurements taken on this firmware:

- Unpatched v/oct jack: **not yet measured** (Task 12 fills this in).
- 0 V patched: **not yet measured**.
- +1 V / +5 V patched: **not yet measured**.
- Automatic internal-source selection: **undecided** (needs the unpatched reading).

## Things to avoid

- Don't add a second board class or rewrite the HAL. `DaisyLegio` is complete.
- Don't reach for external test frameworks.
- Don't add documentation files unless asked.
- Don't claim work is done because host tests pass. The flash + in-rack test is the gate.

## Open / known issues

- (none yet)
