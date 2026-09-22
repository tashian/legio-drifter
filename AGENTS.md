# AGENTS.md — drifter

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

DFU entry, `Error 74`, and `screen`/`cat` port contention: see "Legio hardware notes" below. Telemetry at 5 Hz:
`mode=PAN pos=0.42 ctr=0.50 dep=0.30 T=12.0s ext=0 curve=+0.35 edge=FOLD cv_norm=0.3021 cv=0.000V src=jack`
(appends ` [curve edit]` while curve-edit mode is active)

## Legio hardware notes (shared across all Legio firmware in this family)

1. **Legio has 3 ADC channels, not 4.** `CONTROL_KNOB_TOP` and `CONTROL_KNOB_BOTTOM` each read
   **the analog sum of the knob position + the CV jack above it** — they cannot be separated.
   `CONTROL_PITCH` is the dedicated v/oct jack. Treat knob+CV summing as a feature.
2. **`Switch3.Read()` polarity is inverted vs. the panel.** libDaisy returns 0/1/2 =
   CENTER/POS_UP/POS_DOWN, but on Legio's panel `POS_UP=1` is **panel DOWN** and `POS_DOWN=2` is
   **panel UP**. `main.cpp` inverts once so the rest of the code is panel-relative. Verified by feel
   — don't "correct" it.
3. **newlib-nano strips float printf.** `PrintLine("%f", x)` prints nothing unless `-u _printf_float`
   is in `LDFLAGS` (already set in the Makefile, ~10 KB flash). Keep it or float telemetry goes silent.
4. **V/oct needs per-module calibration.** A floating jack reads a non-zero indeterminate ADC value;
   pushing that through `pow(2, volts)` pins pitch at an extreme. Never ship a v/oct path that
   defaults to a wild value — measure 0 V and +1 V on real hardware first, or leave it disabled.
5. **The audio callback is ~1 ms** (48-sample blocks @ 48 kHz). Never call `PrintLine` from inside
   it — it crashes audio. Telemetry goes through a `volatile` UI snapshot read from a slow loop.
6. **Big buffers (delays, loopers) live in SDRAM** via `DSY_SDRAM_BSS` at the point of definition;
   `dsp_common.h` stubs the macro empty for host builds so the same source compiles both ways.

**Flashing.** DFU entry is BOOT + RESET on the Patch SM submodule on the back of the module. After
`make program-dfu`, dfu-util's "Error during download get_status" / `Error 74` is harmless (the
device resets faster than dfu-util can ack), but the module sometimes won't re-enumerate without a
manual reseat. `/dev/cu.usbmodem*` missing right after flashing isn't necessarily a firmware crash.

**Serial.** `screen /dev/cu.usbmodem* 115200`. If `screen` is already attached in another terminal it
holds the device exclusively and `cat`/other readers fail — check `screen -ls`.

## Hardware quirks specific to this app

- **Audio inputs are AC coupled**, outputs are DC coupled (±5 V). CV mode writes DC to the
  outputs on purpose. In R is normalled to In L in hardware.
- Switch3 inversion happens in `main.cpp`. Don't "fix" it.

## Behavior notes

- Boot defaults: free-running period ≈ 10 s, clock ratio ×1, curve = +0.5 (eased), internal
  CV source off, curve-edit mode off. Nothing persists across power cycles.
- Encoder: tap (< 400 ms) toggles curve-edit mode; long press (≥ 800 ms, fires at 800 ms)
  toggles the internal +5 V source in CV mode only; 400–800 ms is dead; rotation while
  pressed is ignored.
- Bypass mode = encoder held during boot. Pure passthrough, both LEDs dim white. **The boot
  check must poll `ProcessAllControls()` ~12× with 1 ms delays before reading
  `encoder.Pressed()`**: libDaisy's `Switch::Debounce()` shifts one bit per ≥1 ms call and
  `Pressed()` needs eight consecutive 1s, so a single poll can never read pressed.
  (stutterer's `main.cpp` still has the single-poll idiom; its bypass is dead.)
- Clock accepts periods from 0.05 s to **20 s** (stutterer's copy caps at 5 s); while
  acquiring, a lone first edge is kept for up to 20 s so slow ambient clocks can lock; once
  locked, free-run resumes after max(4 periods, 2 s) of silence.
- LEDs: blue = computed position (left = 1 − pos, right = pos); yellow flash on the left
  = new random target; yellow flash on the right = incoming clock edge; curve-edit mode
  shows curve in yellow only; internal CV source adds a yellow floor that scales with the
  blue under it (max(0.3, 0.6·blue)), so the pair reads pale instead of pure blue.

## CV-mode calibration

Defaults in `src/cv_in.h` were reused from sawstack (measured on the author's Patch SM, 2026-05-10);
a gitignored `src/calibration_local.h` (copy the `.example`) overrides them via `__has_include`. See
README "Calibrating the v/oct input" for the procedure. Current defaults: `kCvZero = 0.3019`,
`kCvScale = 7.6805`. Measurements taken on this firmware:

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

- Flash is ~83% of 128 KB (≈106 KB text) after the full chain; there is not much room for
  new features.
- Link-time newlib-nano warnings (`_close`, `_fstat`, … not implemented; `LOAD segment with
  RWX permissions`) are pre-existing toolchain noise, not a firmware problem.
