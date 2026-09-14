# Drifter — Random Bézier Auto-Panner / Crossfader for Legio

**Date:** 2026-09-13
**Target hardware:** Noise Engineering Legio (Daisy Patch SM, STM32H750)
**Repo:** the repo root (independent git repo, libDaisy submodule)

## Goal

A stereo utility firmware built around a **voltage-controlled panner / crossfader**
with a **random Bézier wander** (in the style of hardware random-Bézier CV generators) layered on top. Designed for slow
generative ambient patches: the image drifts on its own, CV and knob set where it
drifts around, and an optional external clock makes it resettle in time with the
patch. A third mode turns the module into a **CV panner**: an envelope in, two
complementary DC envelopes out, for sharing "presence" between two voices.

Everything is a utility, not an instrument. It should be usable with no manual:
one switch picks what the module is, the top knob is "where", the bottom knob is
"how much wander", the encoder is "how fast".

## Hardware reality (verified)

Per `libDaisy/src/daisy_legio.h`, the Patch SM datasheet v1.0.5, and the
workspace-level `CLAUDE.md`:

- **Audio:** stereo in, stereo out, 48 kHz, 48-sample blocks (~1 ms callback).
  - **Inputs are AC coupled** (coupling capacitor on the Patch SM, before the
    codec). They cannot carry CV. No register setting changes this.
  - **Outputs are DC coupled**, −5 V … +5 V. They *can* carry CV.
  - **In R is normalled to In L in hardware** when nothing is patched into In R
    (confirmed by the user and by the Librae Legio manual). With one cable, both
    inputs carry the same signal.
- **3 ADC channels.** `CONTROL_KNOB_TOP` and `CONTROL_KNOB_BOTTOM` each read the
  analog sum of the knob and the CV jack above it — inseparable, treated as a
  feature. `CONTROL_PITCH` (the v/oct jack) is a separate, DC-coupled CV input.
  A floating v/oct jack reads an indeterminate non-zero value.
- **Digital:** 1 gate input, 1 push encoder, 2 three-position switches
  (`Switch3.Read()` polarity is inverted vs. the panel — invert in `main.cpp`
  as the other apps do), 2 RGB LEDs.
- No jack detection of any kind.
- User is colorblind: LEDs use **blue vs. yellow**, paired with brightness,
  never hue alone.

## Modes (left switch)

| Panel position | Mode | Behavior |
|---|---|---|
| Up | **PAN** | Out L = In L · gL, Out R = In R · gR. With only In L patched (hardware normalling) this is a mono→stereo panner. With a stereo pair patched it is a stereo balance control: hard left mutes Out R, hard right mutes Out L, and each output only ever carries its own input. |
| Center | **XFADE** | A = In L, B = In R. Out L = A · gA + B · gB (the crossfade). Out R = A · gB + B · gA (the complementary crossfade). Taking only Out L gives a plain A/B crossfader; Out R is the swapped mix for free. |
| Down | **CV** | Source = the v/oct jack, read as calibrated volts. Out L = source · (1 − pos), Out R = source · pos, as DC voltage on the DC-coupled outputs. Audio inputs are ignored. |

Gains in PAN and XFADE use the **equal-power law**: `gL = cos(pos·π/2)`,
`gR = sin(pos·π/2)`. Center is −3 dB per side, no perceived dip.

Gains in CV mode use the **linear law** so the two outputs always sum to the
source: an envelope split across two filters keeps its total "presence".

`pos` ∈ [0, 1]: 0 = full left / A, 1 = full right / B.

## The position signal

```
center  = top knob + top CV      (0..1, hardware-summed)
depth   = bottom knob + bottom CV (0..1, hardware-summed)
r       = random Bézier generator output, −1..+1
pos_raw = center + 0.5 · depth · r
pos     = edge(pos_raw)          (right switch: CLIP / FOLD / WRAP)
pos_s   = one-pole smoothed pos  (per sample, ~5 ms)
gains   = law(mode, pos_s)
```

- With `depth = 0` the module is a plain CV panner: `pos = center`.
- With `depth = 1` and `center = 0.5` the wander covers exactly the full field.
  With center off-noon the wander pushes past the edges constantly, which is
  what the edge behavior is for.
- Position is computed once per block; the smoother runs per sample so gains
  never zipper. Gains are evaluated per sample from the smoothed position
  (two `sinf`/`cosf` per sample is negligible on the H7).

### Edge behavior (right switch) — the classic LIMIT / FOLD / THRU trio

| Panel position | Name | Rule on `pos_raw` |
|---|---|---|
| Up | **CLIP** | Clamp to [0, 1]. The image parks at the rail when pushed. |
| Center | **FOLD** | Reflect about 0 and 1 (triangle map, period 2). The image bounces off the rail and keeps moving. Default for ambient use. |
| Down | **WRAP** | `pos_raw mod 1`. The image snaps across the field when it crosses an edge. The smoother turns the snap into a ~5 ms sweep. A deliberate, bold effect. |

Applied to the *summed* position, so it also handles CV excursions from the
top jack, not just the random wander.

## The random Bézier generator

Modeled on the behaviour of hardware random-Bézier CV generators. Where that
behaviour is well established we follow it; where it is open, we choose here.

**Points.** A new random target `v_next` is drawn uniformly in [−1, +1] at the
start of every cycle. The generator interpolates from the current value to
`v_next` over the cycle. Targets and timing never depend on the curve setting
(the "curves overlapped" property: all curves pass through the same values
at the same times).

**Cycle timing.** A phase `φ ∈ [0, 1)` advances by `block_size / T` per block,
where `T` (in samples) is the *current* period. When `φ` wraps, the next target
is drawn and a `new_target` flag is raised for one block. Because `T` is read
every block, changing the rate takes effect immediately even mid-cycle (with a
5-minute period, "takes effect next cycle" would be unusable).

**Curve.** A single parameter `curve ∈ [−1, +1]`, a single bipolar shape knob:

- `curve = 0` (noon): linear ramp from `v_cur` to `v_next`.
- `curve < 0` (CCW): the two Bézier control points move **vertically**,
  `P1 = (0, v_cur + k·Δ)`, `P2 = (1, v_next − k·Δ)` with `k = 0.5·|curve|`,
  `Δ = v_next − v_cur`. Fast-slow-fast: leaves each point steeply, flattens
  mid-cycle, arrives steeply. Cusps at the random values.
- `curve > 0` (CW): the control points move **horizontally**,
  `P1 = (k, v_cur)`, `P2 = (1 − k, v_next)` with `k = 0.5·curve`. Slow-fast-slow:
  an eased S-curve at moderate settings, rounded plateaus at each point with the
  fastest motion mid-cycle at full CW. This is the sweet spot for ambient drift.
- `P0 = (0, v_cur)`, `P3 = (1, v_next)` always. All four points lie inside the
  box spanned by the endpoints, so the curve **never overshoots** and never
  leaves the cycle window (convex-hull property, as the manual notes).

**Evaluation.** The curve is a parametric cubic Bézier in (time, value). For the
CW family `x(s)` is not the identity, so at each block we solve `x(s) = φ` for
`s` with 2–3 Newton iterations warm-started from the previous `s` (φ is
monotonic, so `s` is monotonic and the warm start converges immediately), then
output `y(s)`. This gives the true cusps and plateaus rather than an easing
approximation. For `curve = 0` the solve is skipped (`s = φ`).

**Rate.**

- **Free-running:** period from the encoder on a log scale, **5 minutes per
  point (≈ 0.0033 Hz) at the slow end to 20 Hz at the fast end**, roughly 1/3
  octave per detent. The slow end is the ambient use; the fast end is
  tremolo-ish wobble.
- **Clocked:** when the gate input is receiving a clock, `T = clock_period ×
  ratio`, with `ratio` from the encoder in {÷8, ÷4, ÷2, ×1, ×2, ×4, ×8}
  (encoder steps through the list; the number means "targets per clock" for
  ×, "clocks per target" for ÷). On every qualifying clock edge (each edge for
  ×N, every Nth edge for ÷N) the phase hard-resets to 0 and a new target is
  drawn. Between edges the phase free-runs at the measured period so the
  motion stays continuous.
- The encoder edits whichever rate is active (free period or clock ratio);
  both values are kept in RAM so switching back restores the previous setting.
- **Clock detection and fallback:** reuse the stutterer's `Clock` module
  (edge detection, smoothed period, external/fallback state). "External" while
  edges keep arriving; falls back to free-running when no edge has arrived for
  the greater of 4 measured periods or 2 s. Legio cannot sense whether the
  gate jack is patched, so this timeout is the only signal.

**Random source.** xorshift32, seeded at boot from the low bits of the three
ADC channels so each power-up drifts differently. Tests inject a fixed seed.

**What we deliberately leave out** (features found on hardware random-Bézier modules): the second channel and
its cross-normalization, the comparator gate output, the inverted/average
outputs (no jacks for any of it), and rate CV (the only free CV input is now
the CV-mode source).

## Controls

| Control | Function |
|---|---|
| Top knob + CV jack | `center`. Full CCW = left / A, full CW = right / B. |
| Bottom knob + CV jack | `depth`. Zero = plain CV panner. Full = wander covers the whole field. |
| Encoder rotate (normal) | Rate. Free-running: log period, 5 min → 20 Hz. Clocked: ratio ÷8 … ×8. |
| Encoder tap | Toggles **curve edit mode**. In edit mode the LEDs turn yellow and show `curve`, and rotating edits `curve` (−1 … +1 in 24 steps: CCW = cusped, noon = linear, CW = eased/plateau). A second tap returns to normal: rotate edits rate, LEDs show position. A tap is a press-and-release under 400 ms; rotation while pressed is ignored. |
| Encoder held at boot | Bypass: pure passthrough, both LEDs dim white. Workspace convention for isolating "is the audio path alive". |
| Left switch | Mode: PAN (up) / XFADE (center) / CV (down). |
| Right switch | Edge behavior: CLIP (up) / FOLD (center) / WRAP (down). |
| Gate in | Clock for the random generator. Unpatched = free-running. |
| V/oct jack | CV-mode source. Unused in PAN and XFADE. |

Reserved for later, not in this spec: encoder long press (candidate: freeze the
wander in place), rate CV, persistence across power cycles.

## LEDs

Colorblind-safe: blue is the "where" channel, yellow (R+G) is the "event"
channel. Brightness always carries meaning alongside hue.

| LED | Blue (steady) | Yellow (pulse) |
|---|---|---|
| Left | `1 − pos_s`: bright when the image is left. | 50 ms flash at each **new random target** (the generator's internal clock, made visible). |
| Right | `pos_s`: bright when the image is right. | 50 ms flash at each **incoming clock edge**, so a locked clock is visible as a pulse on the beat. |

The blue pair therefore always shows the **computed** position — the sum of
knob, CV, wander, edge behavior and smoothing — which is what you hear. It is
not the knob position; with depth at zero the two coincide.

In **curve edit mode** (entered and left with an encoder tap), both LEDs drop
the blue position display and show `curve` in yellow: left bright = CCW/cusped,
right bright = CW/eased, equal = linear. The new-target and clock flashes are
suppressed while editing so the yellow reading is unambiguous. The wander keeps
running underneath; only the display and the encoder's target change. Tapping
again returns to the position display.

## CV-mode calibration

The v/oct jack ADC needs the workspace's standard calibration before CV mode
can ship (workspace lesson 4). Constants `kCvZero` / `kCvScale` in
`src/cv_in.h`:

```
volts = (norm - kCvZero) * kCvScale
```

Procedure at the first flash gate: patch a known 0 V and a known +5 V (or +1 V)
into the v/oct jack, read `cv_norm` from the serial telemetry, solve the two
constants, rebuild. Output scaling: 1.0 float = +5 V at the DC-coupled output
(Patch SM datasheet range), so `source = volts / 5`. **Until calibrated,
`kCvScale = 0` and CV mode outputs silence** — the firmware is fully usable in
PAN and XFADE from the first flash.

The source is sampled once per block (1 kHz) and smoothed with a ~2 ms one-pole
before the gains are applied. Adequate for envelopes and LFOs; not an audio path.

## Architecture

Same DSP / HAL split as `stutterer` and `sawstack`: DSP modules take and return
`float`, include only `<cmath>` and each other, and never touch libDaisy.
`main.cpp` is the only file that includes `daisy_legio.h`. No DaisySP is needed
(the libDaisy Makefile makes `DAISYSP_DIR` optional), so the repo carries only
the libDaisy submodule.

```
src/
  main.cpp            HAL: reads controls into Params, drives the chain, LEDs, telemetry.
  params.h            Carrier struct (panel-relative switch labels, encoder deltas, knob values).
  dsp_common.h        kSampleRate, kAudioBlockSize, DSY_SDRAM_BSS host shim (unused here but kept for convention).
  bezier_random.{h,cpp}  The generator: Init(sr), SetPeriodSamples, SetCurve, Sync(), Process(block) -> value, new_target().
  clock.{h,cpp}       Copied from stutterer (edge detect, period, fallback).
  edge.h              clip / fold / wrap pure functions.
  panner.{h,cpp}      Mode + smoothed position -> per-sample gains and output mixing for PAN / XFADE / CV.
  cv_in.h             kCvZero / kCvScale and the norm->volts conversion.
  drifter_chain.{h,cpp}  Orchestrator: ApplyParams(Params, clock), ProcessBlock(inL, inR, cv, outL, outR, n).
test/
  test_assert.h       Copied micro-harness.
  test_bezier_random.cpp, test_edge.cpp, test_panner.cpp, test_clock.cpp, test_chain.cpp
lib/libDaisy          Submodule.
Makefile              Classic layout; keeps -u _printf_float.
```

Data flow per block (in the audio callback):

1. `main.cpp` snapshots controls into `Params` (knobs, switch positions with
   panel polarity fixed, encoder increment, encoder tap edge, gate edge, cv
   norm). The curve-edit toggle state lives in the chain, not the HAL, so the
   host tests cover it.
2. `clock.update(gate_edge, n)`.
3. `chain.ApplyParams(p, clock)`: derives period (free or clocked), forwards
   curve/rate edits, computes `center`/`depth`, steps the generator once,
   computes `pos_raw` → edge → sets the smoother target, sets mode/law.
4. `chain.ProcessBlock(...)`: per sample, advance the smoother, compute gains,
   mix per mode.
5. `main.cpp` writes a `volatile` UI snapshot (pos_s, new_target, clock_edge,
   curve, mode) that the slow loop turns into LEDs and 5 Hz serial telemetry.
   Never `PrintLine` from the callback.

Telemetry line (5 Hz): `mode=PAN pos=0.42 ctr=0.50 dep=0.30 T=12.0s ext=0 curve=+0.35 edge=FOLD cv=0.000`.

## Testing

Host tests (`make -C test`, plain g++) are the gate for every algorithm claim
above. Frequencies and periods in tests are exact binary fractions of 48 kHz
where sample-counting matters (workspace memory: never 1000 Hz-style values).

- **bezier_random:** output stays in [−1, 1] for 10⁶ blocks; targets change
  exactly at period boundaries (period = 4096 samples); `curve = 0` is a
  straight line between points to 1e-6; `curve = −1` has larger |slope| in the
  first 5 % of the cycle than at the middle (cusp); `curve = +1` has near-zero
  slope in the first 5 % and largest slope at the middle (plateau); Newton
  solve converges (|x(s) − φ| < 1e-5 every block); `Sync()` resets phase and
  draws a target; fixed seed is deterministic; a period change mid-cycle
  changes the next step size immediately.
- **edge:** clip/fold/wrap on a table of values including 1.5, −0.25, 2.0, −1.0.
- **panner:** equal-power center = 0.7071 both sides; hard left/right; PAN with
  identical inputs equals a mono panner (the normalling case); XFADE Out R is
  the complement of Out L; CV law sums to 1.0 for any pos; CV mode ignores
  audio inputs.
- **clock:** copied from stutterer.
- **chain:** an encoder tap toggles edit mode; encoder increments go to rate
  in normal mode and to curve in edit mode, never both; depth 0 → pos ==
  center; fold keeps pos in [0, 1] for any center
  and depth; the smoothed position never jumps more than a bound per sample
  (no zipper); switching mode mid-block produces no NaN and bounded output.

Hardware gates, in order, at the first flash session:

1. Bypass boot → audio passes. Then PAN with In L only → image follows the knob,
   both outputs live (confirms hardware normalling).
2. XFADE with two sources → Out L crossfades, Out R is the complement.
3. Wander: depth up, encoder to a ~10 s period, watch the blue LEDs drift and
   the yellow left flash on each target. Try all three curve regions by feel.
4. Clock: patch a slow clock, confirm the right LED pulses on the beat and
   targets land on edges; unpatch and confirm free-run resumes within the
   timeout.
5. CV-mode calibration, then an envelope into the v/oct jack and the outs into
   two filter cutoffs.

Host tests passing is not "done". Step 5 is the last gate.

## Non-goals

- Rate CV (no free jack once CV mode claims the v/oct input).
- A second random channel, cross-modulation, comparator/trigger outputs.
- Freeze, persistence, a desktop plugin. All could come later without changing
  the DSP/HAL split.
- Audio-rate crossfading of CV through the audio inputs (impossible: AC coupled).
