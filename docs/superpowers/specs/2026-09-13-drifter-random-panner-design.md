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
complementary DC envelopes out, for sharing "presence" between two voices. With
no envelope patched, the same mode becomes a **generative CV source**: the
wander itself comes out as a complementary pair of slow random voltages.

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
  Sawstack calibrated this ADC on this same Patch SM (2026-05-10): 0 V reads
  0.3019, +1 V reads 0.4321, linear to within ~1 LSB at −2 V. What an
  **unpatched** v/oct jack reads has never been measured in this workspace;
  the older "indeterminate non-zero" notes predate calibration and most likely
  describe the 0 V offset itself (raw ≈ 0.30, not 0.0). Measured at the first
  flash gate, see "CV-mode calibration".
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
| Down | **CV** | Source = the v/oct jack, read as calibrated volts, with a small deadband around 0 V so an unpatched jack gives clean silence; or an internal +5 V constant (see "Generative CV"). Out L = source · (1 − pos), Out R = source · pos, as DC voltage on the DC-coupled outputs. Audio inputs are ignored. |

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
| Encoder long press (≥ 800 ms) | CV mode only: toggles the internal +5 V source (generative CV). No effect in PAN / XFADE. |
| Encoder held at boot | Bypass: pure passthrough, both LEDs dim white. Workspace convention for isolating "is the audio path alive". |
| Left switch | Mode: PAN (up) / XFADE (center) / CV (down). |
| Right switch | Edge behavior: CLIP (up) / FOLD (center) / WRAP (down). |
| Gate in | Clock for the random generator. Unpatched = free-running. |
| V/oct jack | CV-mode source. Unused in PAN and XFADE. |

Reserved for later, not in this spec: rate CV, freeze, persistence across
power cycles.

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

The v/oct jack ADC needs the workspace's standard linear calibration
(workspace lesson 4). Constants live in `src/cv_in.h`:

```
kCvZero     = 0.3019f   // raw ADC at 0 V   (sawstack, this Patch SM, 2026-05-10)
kCvScale    = 7.6805f   // volts per raw unit = 1 / (0.4321 − 0.3019)
kCvDeadband = 0.05f     // volts; |volts| below this reads as exactly 0

volts  = (norm − kCvZero) · kCvScale
volts  = 0 if |volts| < kCvDeadband
source = volts / 5      // 1.0 float = +5 V at the DC-coupled output
```

The starting constants are **reused from sawstack**, which measured the same
ADC channel on the same module, so CV mode is live from the first flash rather
than gated behind a measurement. `kCvScale = 0` still disables the path
entirely (source = 0) if the numbers turn out not to transfer.

The deadband is a hard gate, not a subtraction: a value just above the
threshold passes unchanged. The step at the threshold is 50 mV of CV, which the
2 ms output smoother and any downstream filter cutoff render inaudible. The
threshold is chosen to sit well above floating-input noise and well below any
useful envelope; it is not a "noise floor" to tune by ear.

Procedure at the first flash gate, with serial telemetry open:

1. **Nothing patched.** Record `cv_norm` and how much it wanders over ~10 s.
   Write both numbers into `CLAUDE.md`. This is the first measurement of the
   floating input anyone in this workspace has taken; it decides two things:
   whether the deadband actually covers it, and whether an unpatched jack is
   distinguishable from a patched 0 V (see "Generative CV" below).
2. **0 V patched** (a dummy cable from a quiet output, or a DC source at 0).
   Confirm `cv_norm ≈ 0.3019`. If it differs by more than ~0.005, replace
   `kCvZero`.
3. **+1 V or +5 V patched.** Confirm `volts` in telemetry reads the source to
   within ~1 %. If not, recompute `kCvScale = ΔV / Δnorm` and rebuild.
4. Only if steps 2–3 changed a constant: rebuild, reflash, repeat 1–3.

The source is sampled once per block (1 kHz) and smoothed with a ~2 ms one-pole
before the gains are applied. Adequate for envelopes and LFOs; not an audio path.

## Generative CV (CV mode with the internal source)

In CV mode the source can be the v/oct jack **or an internal +5 V constant**.
With the internal source the outputs are the position itself, as voltage:

```
Out L = 5 V · (1 − pos_s)
Out R = 5 V · pos_s
```

Everything that shapes the position shapes these voltages: center is the
offset, depth is the amplitude, the edge switch folds or wraps them, the
encoder sets rate and curve, and a clock resettles them on the beat. This is a
single random-Bézier channel plus its inverted output, 0 … +5 V unipolar, and the
same pair of outputs that in PAN mode carries audio. Patch them into two filter
cutoffs, two VCA CVs, or a cutoff and a wavefolder, and the two voices breathe
against each other.

The v/oct jack is ignored entirely while the internal source is active (no
deadband decision is involved).

**Selection.** Legio has no jack detection, and the unpatched v/oct reading has
never been measured (see "CV-mode calibration", step 1). So the spec fixes an
explicit selection that works regardless, and names the upgrade path:

- **Encoder long press (≥ 800 ms) toggles the internal source, in CV mode
  only.** In PAN and XFADE a long press does nothing. The setting is
  remembered in RAM while the mode switch is elsewhere. Press timing: a
  release before 400 ms is a tap (curve edit), a hold past 800 ms fires the
  toggle once at the 800 ms mark without waiting for release, and the
  400–800 ms window is dead so a slow tap can't toggle by accident.
- **Automatic upgrade, decided at the flash gate.** If the unpatched jack
  reads a raw value a patched cable cannot produce (outside the ±5 V span, or
  with a distinctive high-variance signature that a driven 0 V does not
  show), then the internal source is selected automatically whenever that
  signature is present for > 1 s, and deselected within one block when a real
  voltage appears. The long press is then freed. If the unpatched jack simply
  reads ≈ 0 V, as the calibration notes suggest it will, detection is not
  possible and the long press stays. Either way the DSP takes a single
  `internal_source` bool; only where it comes from changes.

**LED.** While the internal source is active both LEDs carry a **dim steady
yellow floor** under the usual blue position display (hue paired with a
brightness change, per the colorblind rule). New-target and clock flashes
still appear on top. Leaving CV mode, or toggling the source off, removes the
floor.

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
  cv_in.h             kCvZero / kCvScale / kCvDeadband and the norm->volts->source conversion.
  drifter_chain.{h,cpp}  Orchestrator: ApplyParams(Params, clock), ProcessBlock(inL, inR, cv, outL, outR, n).
test/
  test_assert.h       Copied micro-harness.
  test_bezier_random.cpp, test_edge.cpp, test_panner.cpp, test_cv_in.cpp, test_clock.cpp, test_chain.cpp
lib/libDaisy          Submodule.
Makefile              Classic layout; keeps -u _printf_float.
```

Data flow per block (in the audio callback):

1. `main.cpp` snapshots controls into `Params` (knobs, switch positions with
   panel polarity fixed, encoder increment, encoder tap edge, encoder
   long-press edge, gate edge, cv norm). The curve-edit and internal-source
   toggle states live in the chain, not the HAL, so the host tests cover them.
   The tap / long-press timing itself is HAL code (it needs a millisecond
   clock), but it produces two clean edges the chain consumes.
2. `clock.update(gate_edge, n)`.
3. `chain.ApplyParams(p, clock)`: derives period (free or clocked), forwards
   curve/rate edits, computes `center`/`depth`, steps the generator once,
   computes `pos_raw` → edge → sets the smoother target, sets mode/law.
4. `chain.ProcessBlock(...)`: per sample, advance the smoother, compute gains,
   mix per mode.
5. `main.cpp` writes a `volatile` UI snapshot (pos_s, new_target, clock_edge,
   curve, mode) that the slow loop turns into LEDs and 5 Hz serial telemetry.
   Never `PrintLine` from the callback.

Telemetry line (5 Hz): `mode=PAN pos=0.42 ctr=0.50 dep=0.30 T=12.0s ext=0 curve=+0.35 edge=FOLD cv_norm=0.3021 cv=0.000V src=jack`.

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
  audio inputs; internal source gives Out L + Out R = 1.0 (= 5 V) for any pos
  and ignores the cv input.
- **cv_in:** `norm = kCvZero` → 0; `norm = kCvZero + 1/kCvScale` → exactly 1 V;
  |volts| just under `kCvDeadband` → 0; just over → passes through unchanged;
  `kCvScale = 0` → 0 for any norm; source = volts / 5.
- **clock:** copied from stutterer.
- **chain:** an encoder tap toggles edit mode; a long-press edge toggles the
  internal source in CV mode and is ignored in PAN / XFADE; the internal-source
  setting survives a trip through PAN and back; encoder increments go to rate
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
5. CV-mode calibration (including the unpatched-jack measurement), then an
   envelope into the v/oct jack and the outs into two filter cutoffs. With the
   jack unpatched, both outputs must read 0 V on a meter.
6. Generative CV: long press in CV mode, confirm the yellow floor appears and
   the two outputs drift complementarily on a meter (sum ≈ 5 V). Decide from
   the step-5 floating measurement whether automatic selection is possible;
   record the decision in `CLAUDE.md`.

Host tests passing is not "done". Step 5 is the last gate.

## Non-goals

- Rate CV (no free jack once CV mode claims the v/oct input).
- A second random channel, cross-modulation, comparator/trigger outputs.
- Freeze, persistence, a desktop plugin. All could come later without changing
  the DSP/HAL split.
- Audio-rate crossfading of CV through the audio inputs (impossible: AC coupled).
