# Drifter Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the Drifter firmware for the Noise Engineering Legio: a voltage-controlled panner / crossfader / CV panner with a random Bézier wander, per the spec below.

**Architecture:** Pure-float DSP modules (`edge`, `cv_in`, `clock`, `bezier_random`, `panner`, `drifter_chain`) compile under both `arm-none-eabi-g++` (firmware) and host `g++` (unit tests) and never include libDaisy. `main.cpp` is the only file that touches the HAL: it snapshots controls into a `Params` struct, feeds the chain each audio block, and drives LEDs + serial telemetry from the slow loop through a `volatile` UI snapshot. Every DSP module is built test-first.

**Tech Stack:** C++17. libDaisy (git submodule, no DaisySP). GNU make. dfu-util for flashing. Host tests use the workspace micro-harness (`test_assert.h`), no external framework.

**Spec:** `docs/superpowers/specs/2026-09-13-drifter-random-panner-design.md` (inside this repo). The plan argues from the spec; read both.

## Global Constraints

Copied from the spec and the workspace `CLAUDE.md`. Every task's requirements implicitly include these.

- Repo: the repo root, independent git repo, `main` branch, libDaisy submodule only (no DaisySP). All commands below run from that directory unless stated.
- Audio: 48 kHz, 48-sample blocks (~1 ms callback). `kSampleRate = 48000.0f`, `kAudioBlockSize = 48`.
- DSP files include only `<cmath>`, `<cstdint>`, and each other. Only `src/main.cpp` includes `daisy_legio.h`.
- `LDFLAGS += -u _printf_float` stays in the Makefile (float telemetry goes silent otherwise).
- Never call `PrintLine` from the audio callback. Telemetry goes through a `volatile` snapshot read by the slow loop.
- `Switch3.Read()` polarity: lib `1` (`POS_UP`) = **panel DOWN**, lib `2` (`POS_DOWN`) = **panel UP**, `0` = center. Invert in `main.cpp`. Do not "fix" this.
- Knob ADCs are the hardware sum of knob + CV jack. Treat as a feature.
- LEDs are colorblind-safe: blue = "where", yellow (R+G) = "event"; hue is always paired with brightness.
- Test frequencies and periods are exact binary fractions of the sample rate where sample counting matters (e.g. period 4096 with block 64 or block 1, never 1000-style values).
- Position law: `pos_raw = center + 0.5 · depth · r`, edge-mapped, then one-pole smoothed (~5 ms) per sample. PAN / XFADE use equal-power gains `gL = cos(pos·π/2)`, `gR = sin(pos·π/2)`. CV uses linear gains `(1 − pos)`, `pos`.
- CV calibration constants: `kCvZero = 0.3019f`, `kCvScale = 7.6805f`, `kCvDeadband = 0.05f` (volts). `kCvScale = 0` disables the CV path.
- Rate: free-running period on a log scale from 5 min (300 s) to 20 Hz (0.05 s); clocked ratio in {÷8, ÷4, ÷2, ×1, ×2, ×4, ×8}.
- Encoder timing: tap = press-and-release under 400 ms; long press fires once at 800 ms while still held; 400–800 ms is dead. Rotation while pressed is ignored.
- Clock fallback: free-running when no edge has arrived for the greater of 4 measured periods or 2 s.
- Random source: xorshift32, seeded at boot from ADC low bits; tests inject a fixed seed.
- Host tests passing is not "done". The hardware gates in Tasks 11–12 are the real gate, and the CV calibration in Task 12 is the last one.

---

## File structure

All new files live in the repo root. The reference for shape and conventions is the sibling `stutterer` firmware (Makefile, test harness, `clock`, `main.cpp` idioms).

```
drifter/
├── CLAUDE.md                       Hardware quirks + this app's specifics + deviations (Task 1, updated Task 12).
├── README.md                       Player-facing one-pager.
├── Makefile                        Firmware build. -u _printf_float. No DAISYSP_DIR.
├── .gitignore                      build/, build_host/, *.bin, *.elf, .DS_Store
├── .gitmodules                     libDaisy only
├── lib/libDaisy/                   submodule (electro-smith/libDaisy)
├── docs/superpowers/{specs,plans}/ (already present)
├── src/
│   ├── dsp_common.h                kSampleRate, kAudioBlockSize, DSY_SDRAM_BSS host shim.
│   ├── params.h                    Mode / Edge enums + Params carrier struct.
│   ├── edge.h                      Header-only clip / fold / wrap + apply_edge(Edge, x).
│   ├── cv_in.h                     Header-only kCvZero/kCvScale/kCvDeadband + norm→volts→source.
│   ├── clock.{h,cpp}               Copied from stutterer; fallback rule = max(4 periods, 2 s).
│   ├── bezier_random.{h,cpp}       The wander generator: RNG, cycle timing, Bézier curve, Newton solve, Sync.
│   ├── panner.{h,cpp}              Mode + smoothed position → per-sample gains and mixing for PAN / XFADE / CV.
│   ├── drifter_chain.{h,cpp}       Orchestrator: rate/ratio/curve state, encoder + long-press logic, ApplyParams, ProcessBlock.
│   └── main.cpp                    HAL: controls → Params, encoder timing, chain, LEDs, telemetry, bypass, seed.
└── test/
    ├── Makefile                    Same micro-harness shape as stutterer/test.
    ├── test_assert.h               EXPECT_EQ / EXPECT_NEAR / EXPECT_TRUE / RUN_TEST / TEST_MAIN.
    ├── test_smoke.cpp              Harness link check.
    ├── test_edge.cpp               Task 2
    ├── test_cv_in.cpp              Task 3
    ├── test_clock.cpp              Task 4
    ├── test_bezier_random.cpp      Tasks 5–6
    ├── test_panner.cpp             Tasks 7–8
    └── test_chain.cpp              Task 9
```

Shared type vocabulary used by every task (defined in Task 1, `src/params.h`):

```cpp
namespace legio {
enum class Mode { PAN = 0, XFADE = 1, CV = 2 };
enum class Edge { CLIP = 0, FOLD = 1, WRAP = 2 };
struct Params { /* see Task 1 */ };
}
```

---

## Task 1: Project skeleton

**Files:**
- Create: `.gitignore`
- Create: `.gitmodules` (via `git submodule add`)
- Create: `Makefile`
- Create: `src/dsp_common.h`
- Create: `src/params.h`
- Create: `src/main.cpp` (minimal passthrough — replaced in Task 10)
- Create: `test/Makefile`
- Create: `test/test_assert.h`
- Create: `test/test_smoke.cpp`
- Create: `CLAUDE.md`
- Create: `README.md`

**Interfaces:**
- Produces: `legio::kSampleRate`, `legio::kAudioBlockSize`, `DSY_SDRAM_BSS` shim, `legio::Mode`, `legio::Edge`, `legio::Params` (exact fields below), the test harness macros, `make`, `make -C test`, `make program-dfu`.

- [ ] **Step 1: Confirm the repo state and add the libDaisy submodule**

```bash
cd drifter
git status --short        # expect: clean (only docs/ committed so far)
mkdir -p src test lib
git submodule add https://github.com/electro-smith/libDaisy.git lib/libDaisy
git -C lib/libDaisy checkout 08f29653610c08cdf1ca7b5cfcac3bb4922e60c4   # same commit stutterer builds against
git submodule update --init --recursive
```

Expected: `lib/libDaisy` populated, `.gitmodules` created with one entry.

- [ ] **Step 2: Build libDaisy once**

```bash
make -C lib/libDaisy
```

Expected: ends with `build/libdaisy.a` (takes a couple of minutes the first time). If `arm-none-eabi-g++` is missing, it is the same toolchain the sibling projects use; stop and report rather than installing.

- [ ] **Step 3: Write `.gitignore`**

```gitignore
# Firmware build artifacts
/build/
*.o
*.d
*.elf
*.bin
*.hex
*.map
*.lst

# Host test build artifacts
/build_host/

# Editor / OS noise
.DS_Store
.vscode/
*.swp
```

- [ ] **Step 4: Write `Makefile`**

No `DAISYSP_DIR`: libDaisy's core Makefile guards every DaisySP reference with `ifdef DAISYSP_DIR`, so omitting it links only `-ldaisy`.

```make
# Project Name
TARGET = drifter

# Sources — each DSP task appends its .cpp here.
CPP_SOURCES = src/main.cpp

# Pull in the float-printf code from full newlib so PrintLine("%f") actually
# emits the float (newlib-nano strips this out by default to save ~10KB).
LDFLAGS += -u _printf_float

# Library Locations (no DaisySP: this app is pure float math + libDaisy HAL)
LIBDAISY_DIR = lib/libDaisy

# Project source includes
C_INCLUDES += -Isrc

# Use Daisy's stock build system
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
```

- [ ] **Step 5: Write `src/dsp_common.h`**

```cpp
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
```

- [ ] **Step 6: Write `src/params.h`**

All switch labels are **panel-relative** (main.cpp has already inverted the lib's polarity by the time this struct is filled).

```cpp
// src/params.h
#pragma once

namespace legio {

// Left switch, panel-relative: up = PAN, center = XFADE, down = CV.
enum class Mode { PAN = 0, XFADE = 1, CV = 2 };

// Right switch, panel-relative: up = CLIP, center = FOLD, down = WRAP.
enum class Edge { CLIP = 0, FOLD = 1, WRAP = 2 };

struct Params {
    // Continuous controls, 0..1 from the ADC (knob + CV jack summed in hardware).
    float top_knob    = 0.5f;     // center: 0 = left / A, 1 = right / B
    float bottom_knob = 0.0f;     // depth:  0 = plain CV panner, 1 = full-field wander
    float cv_norm     = 0.3019f;  // raw v/oct ADC 0..1 (default = calibrated 0 V)

    // Switches.
    Mode mode = Mode::PAN;
    Edge edge = Edge::FOLD;

    // Encoder. main.cpp turns raw presses into these clean edges.
    int  encoder_increment  = 0;      // -1 / 0 / +1 per block, 0 while pressed
    bool encoder_tap        = false;  // press-and-release < 400 ms (one block)
    bool encoder_long_press = false;  // held >= 800 ms, fires once (one block)

    // Gate jack rising edge this block.
    bool gate_edge = false;
};

}  // namespace legio
```

- [ ] **Step 7: Write a minimal `src/main.cpp` (passthrough, proves the toolchain)**

```cpp
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
```

- [ ] **Step 8: Build the firmware**

```bash
make
```

Expected: `build/drifter.bin` exists; the memory-usage summary prints with no errors.

- [ ] **Step 9: Write `test/test_assert.h`** (verbatim copy of the workspace harness)

```cpp
// test/test_assert.h
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

inline int g_test_failures = 0;

#define EXPECT_EQ(a, b) do { \
    auto _a = (a); auto _b = (b); \
    if (!((_a) == (_b))) { \
        std::fprintf(stderr, "FAIL %s:%d: %s == %s (got %g vs %g)\n", \
                     __FILE__, __LINE__, #a, #b, (double)_a, (double)_b); \
        ++g_test_failures; \
    } \
} while (0)

#define EXPECT_NEAR(a, b, tol) do { \
    double _a = (double)(a); double _b = (double)(b); double _t = (double)(tol); \
    if (std::fabs(_a - _b) > _t) { \
        std::fprintf(stderr, "FAIL %s:%d: |%s - %s| <= %g (got %g vs %g, |diff|=%g)\n", \
                     __FILE__, __LINE__, #a, #b, _t, _a, _b, std::fabs(_a - _b)); \
        ++g_test_failures; \
    } \
} while (0)

#define EXPECT_TRUE(x) do { \
    if (!(x)) { \
        std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); \
        ++g_test_failures; \
    } \
} while (0)

#define RUN_TEST(fn) do { \
    int before = g_test_failures; \
    std::printf("RUN  %s\n", #fn); \
    fn(); \
    if (g_test_failures == before) std::printf("PASS %s\n", #fn); \
    else                           std::printf("FAIL %s\n", #fn); \
} while (0)

#define TEST_MAIN() \
    int main() { \
        run_all(); \
        if (g_test_failures > 0) { \
            std::fprintf(stderr, "%d test failure(s)\n", g_test_failures); \
            return 1; \
        } \
        std::printf("All tests passed.\n"); \
        return 0; \
    }
```

- [ ] **Step 10: Write `test/test_smoke.cpp`**

```cpp
// test/test_smoke.cpp
#include "test_assert.h"
#include "dsp_common.h"
#include "params.h"

static void test_assertion_macros_work() {
    EXPECT_EQ(1 + 1, 2);
    EXPECT_NEAR(0.1 + 0.2, 0.3, 1e-9);
    EXPECT_TRUE(true);
}

static void test_params_defaults() {
    legio::Params p;
    EXPECT_EQ((int)p.mode, (int)legio::Mode::PAN);
    EXPECT_EQ((int)p.edge, (int)legio::Edge::FOLD);
    EXPECT_NEAR(p.top_knob, 0.5f, 1e-9);
    EXPECT_EQ(p.encoder_increment, 0);
    EXPECT_TRUE(!p.encoder_tap);
    EXPECT_TRUE(!p.encoder_long_press);
    EXPECT_EQ(legio::kAudioBlockSize, 48);
}

static void run_all() {
    RUN_TEST(test_assertion_macros_work);
    RUN_TEST(test_params_defaults);
}

TEST_MAIN()
```

- [ ] **Step 11: Write `test/Makefile`**

Each later task appends its test to `TESTS` and adds a recipe. No DaisySP include paths.

```make
# test/Makefile — host-side unit tests for the DSP modules.
# Build & run with:   make -C test
CXX      ?= g++
CXXFLAGS  = -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -I../src -I.

OUT       = ../build_host

# Default test list — extended as new modules are added.
TESTS = test_smoke

all: $(addprefix $(OUT)/, $(TESTS))
	@for t in $(TESTS); do \
	  echo "===== $$t ====="; \
	  $(OUT)/$$t || exit 1; \
	done

$(OUT):
	mkdir -p $(OUT)

$(OUT)/test_smoke: test_smoke.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@

clean:
	rm -rf $(OUT)

.PHONY: all clean
```

- [ ] **Step 12: Run the host tests**

```bash
make -C test
```

Expected: `===== test_smoke =====`, two `PASS` lines, `All tests passed.`

- [ ] **Step 13: Write `CLAUDE.md`** (four-backtick fence below because the file itself contains code fences)

````markdown
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
````

- [ ] **Step 14: Write `README.md`**

```markdown
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
```

- [ ] **Step 15: Commit**

```bash
git add .gitignore .gitmodules lib/libDaisy Makefile src test CLAUDE.md README.md
git commit -m "feat: project skeleton (libDaisy submodule, Makefile, params, test harness)"
```

---

## Task 2: Edge behavior (`edge.h`)

**Files:**
- Create: `src/edge.h`
- Create: `test/test_edge.cpp`
- Modify: `test/Makefile` (add `test_edge`)

**Interfaces:**
- Consumes: `legio::Edge` from `src/params.h`.
- Produces: `float legio::edge_clip(float)`, `float legio::edge_fold(float)`, `float legio::edge_wrap(float)`, `float legio::apply_edge(Edge, float)`. All map any float to `[0, 1]`.

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_edge.cpp
#include "test_assert.h"
#include "edge.h"

using namespace legio;

static void test_clip() {
    EXPECT_NEAR(edge_clip(0.3f),   0.3f, 1e-7);
    EXPECT_NEAR(edge_clip(1.5f),   1.0f, 1e-7);
    EXPECT_NEAR(edge_clip(-0.25f), 0.0f, 1e-7);
    EXPECT_NEAR(edge_clip(2.0f),   1.0f, 1e-7);
    EXPECT_NEAR(edge_clip(-1.0f),  0.0f, 1e-7);
}

static void test_fold_reflects_about_0_and_1() {
    EXPECT_NEAR(edge_fold(0.3f),   0.3f,  1e-6);
    EXPECT_NEAR(edge_fold(1.5f),   0.5f,  1e-6);   // bounce off 1
    EXPECT_NEAR(edge_fold(-0.25f), 0.25f, 1e-6);   // bounce off 0
    EXPECT_NEAR(edge_fold(2.0f),   0.0f,  1e-6);   // full period back to 0
    EXPECT_NEAR(edge_fold(-1.0f),  1.0f,  1e-6);
    EXPECT_NEAR(edge_fold(1.0f),   1.0f,  1e-6);
    EXPECT_NEAR(edge_fold(3.25f),  0.75f, 1e-6);   // period 2
}

static void test_wrap_is_mod_1() {
    EXPECT_NEAR(edge_wrap(0.3f),   0.3f,  1e-6);
    EXPECT_NEAR(edge_wrap(1.5f),   0.5f,  1e-6);
    EXPECT_NEAR(edge_wrap(-0.25f), 0.75f, 1e-6);
    EXPECT_NEAR(edge_wrap(2.0f),   0.0f,  1e-6);
    EXPECT_NEAR(edge_wrap(-1.0f),  0.0f,  1e-6);
}

static void test_all_stay_in_unit_interval() {
    for (int i = -400; i <= 400; ++i) {
        float x = i * 0.0125f;   // -5 .. +5
        for (int e = 0; e < 3; ++e) {
            float y = apply_edge((Edge)e, x);
            EXPECT_TRUE(y >= 0.0f && y <= 1.0f);
        }
    }
}

static void test_apply_edge_dispatch() {
    EXPECT_NEAR(apply_edge(Edge::CLIP, 1.5f), 1.0f, 1e-6);
    EXPECT_NEAR(apply_edge(Edge::FOLD, 1.5f), 0.5f, 1e-6);
    EXPECT_NEAR(apply_edge(Edge::WRAP, 1.5f), 0.5f, 1e-6);
    EXPECT_NEAR(apply_edge(Edge::FOLD, -0.25f), 0.25f, 1e-6);
    EXPECT_NEAR(apply_edge(Edge::WRAP, -0.25f), 0.75f, 1e-6);
}

static void run_all() {
    RUN_TEST(test_clip);
    RUN_TEST(test_fold_reflects_about_0_and_1);
    RUN_TEST(test_wrap_is_mod_1);
    RUN_TEST(test_all_stay_in_unit_interval);
    RUN_TEST(test_apply_edge_dispatch);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

Change `TESTS = test_smoke` to `TESTS = test_smoke test_edge` and add before `clean:`:

```make
$(OUT)/test_edge: test_edge.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `edge.h: No such file or directory`.

- [ ] **Step 4: Write `src/edge.h`**

```cpp
// src/edge.h — position edge behavior (right switch). Borrowed from the
// LIMIT / FOLD / THRU edge behaviours. All three map any float onto [0, 1].
#pragma once
#include <cmath>
#include "params.h"

namespace legio {

// Clamp to [0, 1]: the image parks at the rail.
inline float edge_clip(float x) {
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

// Triangle map with period 2: reflect about 0 and 1. The image bounces.
inline float edge_fold(float x) {
    float t = std::fmod(x, 2.0f);
    if (t < 0.0f) t += 2.0f;          // now 0 <= t < 2
    return t > 1.0f ? 2.0f - t : t;
}

// x mod 1: the image snaps across the field.
inline float edge_wrap(float x) {
    float t = x - std::floor(x);
    return t >= 1.0f ? 0.0f : t;       // guard float rounding at the seam
}

inline float apply_edge(Edge e, float x) {
    switch (e) {
        case Edge::CLIP: return edge_clip(x);
        case Edge::WRAP: return edge_wrap(x);
        case Edge::FOLD:
        default:         return edge_fold(x);
    }
}

}  // namespace legio
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
make -C test
```

Expected: `test_edge` shows 5 `PASS` lines, `All tests passed.`

- [ ] **Step 6: Commit**

```bash
git add src/edge.h test/test_edge.cpp test/Makefile
git commit -m "feat(edge): clip / fold / wrap position edge behavior"
```

---

## Task 3: CV input conversion (`cv_in.h`)

**Files:**
- Create: `src/cv_in.h`
- Create: `test/test_cv_in.cpp`
- Modify: `test/Makefile` (add `test_cv_in`)

**Interfaces:**
- Produces: `legio::kCvZero`, `legio::kCvScale`, `legio::kCvDeadband`; `float legio::cv_volts_from_norm(float norm, float zero = kCvZero, float scale = kCvScale, float deadband = kCvDeadband)`; `float legio::cv_source_from_volts(float volts)` (= volts / 5).

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_cv_in.cpp
#include "test_assert.h"
#include "cv_in.h"

using namespace legio;

static void test_zero_norm_reads_zero_volts() {
    EXPECT_NEAR(cv_volts_from_norm(kCvZero), 0.0f, 1e-7);
}

static void test_one_volt() {
    float norm = kCvZero + 1.0f / kCvScale;
    EXPECT_NEAR(cv_volts_from_norm(norm), 1.0f, 1e-5);
}

static void test_five_volts_and_negative() {
    EXPECT_NEAR(cv_volts_from_norm(kCvZero + 5.0f / kCvScale),  5.0f, 1e-4);
    EXPECT_NEAR(cv_volts_from_norm(kCvZero - 2.0f / kCvScale), -2.0f, 1e-4);
}

static void test_deadband_is_a_hard_gate() {
    float just_under = kCvZero + (kCvDeadband - 0.001f) / kCvScale;
    float just_over  = kCvZero + (kCvDeadband + 0.001f) / kCvScale;
    EXPECT_NEAR(cv_volts_from_norm(just_under), 0.0f, 1e-7);
    EXPECT_NEAR(cv_volts_from_norm(just_over),  kCvDeadband + 0.001f, 1e-5);  // unchanged, not subtracted
    float neg_under = kCvZero - (kCvDeadband - 0.001f) / kCvScale;
    float neg_over  = kCvZero - (kCvDeadband + 0.001f) / kCvScale;
    EXPECT_NEAR(cv_volts_from_norm(neg_under), 0.0f, 1e-7);
    EXPECT_NEAR(cv_volts_from_norm(neg_over),  -(kCvDeadband + 0.001f), 1e-5);
}

static void test_scale_zero_disables_path() {
    EXPECT_NEAR(cv_volts_from_norm(0.0f, kCvZero, 0.0f), 0.0f, 1e-9);
    EXPECT_NEAR(cv_volts_from_norm(1.0f, kCvZero, 0.0f), 0.0f, 1e-9);
    EXPECT_NEAR(cv_volts_from_norm(kCvZero + 0.3f, kCvZero, 0.0f), 0.0f, 1e-9);
}

static void test_source_is_volts_over_five() {
    EXPECT_NEAR(cv_source_from_volts(5.0f),  1.0f, 1e-7);
    EXPECT_NEAR(cv_source_from_volts(2.5f),  0.5f, 1e-7);
    EXPECT_NEAR(cv_source_from_volts(-5.0f), -1.0f, 1e-7);
    EXPECT_NEAR(cv_source_from_volts(0.0f),  0.0f, 1e-7);
}

static void run_all() {
    RUN_TEST(test_zero_norm_reads_zero_volts);
    RUN_TEST(test_one_volt);
    RUN_TEST(test_five_volts_and_negative);
    RUN_TEST(test_deadband_is_a_hard_gate);
    RUN_TEST(test_scale_zero_disables_path);
    RUN_TEST(test_source_is_volts_over_five);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

`TESTS = test_smoke test_edge test_cv_in`, and:

```make
$(OUT)/test_cv_in: test_cv_in.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `cv_in.h: No such file or directory`.

- [ ] **Step 4: Write `src/cv_in.h`**

```cpp
// src/cv_in.h — v/oct jack calibration and conversion for CV mode.
//
// Constants reused from sawstack (same Patch SM, measured 2026-05-10):
//   0 V read 0.3019, +1 V read 0.4321, linear to ~1 LSB at -2 V.
// Task 12 of the plan re-verifies them on this firmware. kCvScale = 0
// disables the path entirely (every norm reads as 0 V).
#pragma once
#include <cmath>

namespace legio {

constexpr float kCvZero     = 0.3019f;   // raw ADC at 0 V
constexpr float kCvScale    = 7.6805f;   // volts per raw unit = 1 / (0.4321 - 0.3019)
constexpr float kCvDeadband = 0.05f;     // volts; |volts| below this reads as exactly 0

// Raw ADC (0..1) -> volts. The deadband is a hard gate, not a subtraction:
// a value just above the threshold passes through unchanged.
inline float cv_volts_from_norm(float norm,
                                float zero     = kCvZero,
                                float scale    = kCvScale,
                                float deadband = kCvDeadband) {
    if (scale == 0.0f) return 0.0f;
    float volts = (norm - zero) * scale;
    return std::fabs(volts) < deadband ? 0.0f : volts;
}

// 1.0 float at the DC-coupled output = +5 V.
inline float cv_source_from_volts(float volts) { return volts * 0.2f; }

}  // namespace legio
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
make -C test
```

Expected: `test_cv_in` shows 6 `PASS` lines, `All tests passed.`

- [ ] **Step 6: Commit**

```bash
git add src/cv_in.h test/test_cv_in.cpp test/Makefile
git commit -m "feat(cv_in): calibrated v/oct -> volts -> source with hard deadband"
```

---

## Task 4: Clock (copied from stutterer, new fallback rule)

**Files:**
- Create: `src/clock.h`
- Create: `src/clock.cpp`
- Create: `test/test_clock.cpp`
- Modify: `test/Makefile` (add `test_clock`)
- Modify: `Makefile` (add `src/clock.cpp` to `CPP_SOURCES`)

**Interfaces:**
- Produces: `class legio::Clock` with `void Init(float sample_rate)`, `void update(bool gate_edge, int block_size)`, `float period_samples() const`, `bool tick() const` (true on the block with an edge), `bool is_external() const`.

Two deliberate deviations from the stutterer copy, both from the spec: the fallback timeout is `max(4 · measured period, 2 s)` instead of a flat 2 s, and the maximum accepted clock period is raised from 5 s to 20 s so slow ambient clocks are not rejected as "too slow" (a 5 s ceiling would have silently ignored a 0.1 Hz clock, the spec's main use case). Fallback period (when not external) stays 0.5 s; the chain never uses it because free-running mode has its own period.

- [ ] **Step 1: Write the failing test**

```cpp
// test/test_clock.cpp
#include "test_assert.h"
#include "clock.h"

using legio::Clock;

// Helper: one edge, then `silence_blocks` blocks of 48 samples.
static void edge_then_silence(Clock& c, int silence_blocks) {
    c.update(true, 48);
    for (int i = 0; i < silence_blocks; ++i) c.update(false, 48);
}

static void test_starts_internal() {
    Clock c;
    c.Init(48000.0f);
    EXPECT_TRUE(!c.is_external());
    EXPECT_TRUE(c.period_samples() > 0.0f);
}

static void test_two_intervals_lock_period() {
    Clock c;
    c.Init(48000.0f);
    // Edges 12000 samples apart = 250 blocks of 48.
    edge_then_silence(c, 250);
    edge_then_silence(c, 250);
    c.update(true, 48);
    EXPECT_NEAR(c.period_samples(), 12000.0f, 100.0f);
    EXPECT_TRUE(c.is_external());
}

static void test_falls_back_after_two_seconds_for_fast_clock() {
    Clock c;
    c.Init(48000.0f);
    edge_then_silence(c, 250);
    c.update(true, 48);
    EXPECT_TRUE(c.is_external());
    // 4 periods = 1 s < 2 s, so the 2 s floor applies. 2 s = 2000 blocks.
    for (int i = 0; i < 1990; ++i) c.update(false, 48);
    EXPECT_TRUE(c.is_external());           // 1.9 s: still locked
    for (int i = 0; i < 20; ++i) c.update(false, 48);
    EXPECT_TRUE(!c.is_external());          // 2.02 s: fallen back
}

static void test_slow_clock_waits_four_periods() {
    Clock c;
    c.Init(48000.0f);
    // Edges 48000 samples apart (1 s) = 1000 blocks. 4 periods = 4 s > 2 s.
    edge_then_silence(c, 1000);
    c.update(true, 48);
    EXPECT_TRUE(c.is_external());
    EXPECT_NEAR(c.period_samples(), 48000.0f, 100.0f);
    for (int i = 0; i < 3000; ++i) c.update(false, 48);   // 3 s silence
    EXPECT_TRUE(c.is_external());                          // still waiting
    for (int i = 0; i < 1100; ++i) c.update(false, 48);   // 4.1 s total
    EXPECT_TRUE(!c.is_external());
}

static void test_accepts_ten_second_clock() {
    Clock c;
    c.Init(48000.0f);
    // 10 s = 480000 samples = 10000 blocks.
    edge_then_silence(c, 10000);
    c.update(true, 48);
    EXPECT_TRUE(c.is_external());
    EXPECT_NEAR(c.period_samples(), 480000.0f, 100.0f);
}

static void test_tick_true_only_on_edge_block() {
    Clock c;
    c.Init(48000.0f);
    EXPECT_TRUE(!c.tick());
    c.update(true, 48);
    EXPECT_TRUE(c.tick());
    c.update(false, 48);
    EXPECT_TRUE(!c.tick());
}

static void run_all() {
    RUN_TEST(test_starts_internal);
    RUN_TEST(test_two_intervals_lock_period);
    RUN_TEST(test_falls_back_after_two_seconds_for_fast_clock);
    RUN_TEST(test_slow_clock_waits_four_periods);
    RUN_TEST(test_accepts_ten_second_clock);
    RUN_TEST(test_tick_true_only_on_edge_block);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

`TESTS = test_smoke test_edge test_cv_in test_clock`, and:

```make
$(OUT)/test_clock: test_clock.cpp ../src/clock.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `clock.h: No such file or directory`.

- [ ] **Step 4: Write `src/clock.h`**

```cpp
// src/clock.h — gate-input clock: edge detect, smoothed period, external / fallback state.
// Copied from stutterer; fallback rule changed to max(4 periods, 2 s) per the drifter spec.
#pragma once

namespace legio {

class Clock {
  public:
    void  Init(float sample_rate);
    void  update(bool gate_edge, int block_size);
    float period_samples() const;
    bool  tick() const;          // true on the block where an edge arrived
    bool  is_external() const;   // true while edges keep arriving

  private:
    float sample_rate_;
    float fallback_period_;
    float silence_floor_;        // 2 s in samples
    float min_period_;
    float max_period_;
    float period_;
    int   samples_since_edge_;
    int   samples_to_last_edge_;
    bool  have_first_edge_;
    bool  is_external_;
    bool  tick_;
    int   edge_count_;
};

}  // namespace legio
```

- [ ] **Step 5: Write `src/clock.cpp`**

```cpp
// src/clock.cpp
#include "clock.h"
#include "dsp_common.h"

namespace legio {

namespace {
constexpr float kFallbackBpm      = 120.0f;
constexpr float kFallbackSeconds  = 60.0f / kFallbackBpm;   // 0.5 s (unused by drifter's chain)
constexpr float kSilenceFloorSec  = 2.0f;                   // minimum time before falling back
constexpr float kFallbackPeriods  = 4.0f;                   // ... or this many measured periods
constexpr float kSmoothingAlpha   = 0.3f;
constexpr float kMinPeriodSec     = 0.05f;                  // 20 Hz ceiling
constexpr float kMaxPeriodSec     = 20.0f;                  // 0.05 Hz floor (slow ambient clocks)
}

void Clock::Init(float sample_rate) {
    sample_rate_          = sample_rate;
    fallback_period_      = kFallbackSeconds * sample_rate_;
    silence_floor_        = kSilenceFloorSec * sample_rate_;
    min_period_           = kMinPeriodSec * sample_rate_;
    max_period_           = kMaxPeriodSec * sample_rate_;
    period_               = fallback_period_;
    samples_since_edge_   = 0;
    samples_to_last_edge_ = 0;
    have_first_edge_      = false;
    is_external_          = false;
    tick_                 = false;
    edge_count_           = 0;
}

void Clock::update(bool gate_edge, int block_size) {
    tick_ = gate_edge;
    if (gate_edge) {
        if (have_first_edge_) {
            float interval = (float)samples_to_last_edge_;
            if (interval >= min_period_ && interval <= max_period_) {
                if (edge_count_ == 0) {
                    period_ = interval;       // first measured interval = direct
                } else {
                    period_ += kSmoothingAlpha * (interval - period_);
                }
                ++edge_count_;
                is_external_ = true;
            }
        }
        have_first_edge_      = true;
        samples_to_last_edge_ = 0;
        samples_since_edge_   = 0;
    } else {
        samples_to_last_edge_ += block_size;
        samples_since_edge_   += block_size;
        float timeout = kFallbackPeriods * period_;
        if (timeout < silence_floor_) timeout = silence_floor_;
        if ((float)samples_since_edge_ > timeout) {
            is_external_     = false;
            period_          = fallback_period_;
            edge_count_      = 0;
            have_first_edge_ = false;
        }
    }
}

float Clock::period_samples() const { return period_; }
bool  Clock::tick()           const { return tick_;  }
bool  Clock::is_external()    const { return is_external_; }

}  // namespace legio
```

- [ ] **Step 6: Add `src/clock.cpp` to the firmware Makefile**

In `Makefile`: `CPP_SOURCES = src/main.cpp src/clock.cpp`

- [ ] **Step 7: Run the tests and the firmware build**

```bash
make -C test && make
```

Expected: `test_clock` shows 6 `PASS` lines, `All tests passed.`; firmware links.

- [ ] **Step 8: Commit**

```bash
git add src/clock.h src/clock.cpp test/test_clock.cpp test/Makefile Makefile
git commit -m "feat(clock): gate clock with max(4 periods, 2 s) fallback and 20 s ceiling"
```

---

## Task 5: Bézier random generator — RNG, cycle timing, linear curve, Sync

**Files:**
- Create: `src/bezier_random.h`
- Create: `src/bezier_random.cpp`
- Create: `test/test_bezier_random.cpp`
- Modify: `test/Makefile` (add `test_bezier_random`)
- Modify: `Makefile` (add `src/bezier_random.cpp`)

**Interfaces:**
- Produces: `class legio::BezierRandom`:
  - `void Init(float sample_rate, uint32_t seed)`
  - `void SetPeriodSamples(float period_samples)` — read every block, takes effect immediately
  - `void SetCurve(float curve)` — clamped to [−1, +1]
  - `void SetEndpoints(float v_cur, float v_next)` — test hook, also resets phase to 0
  - `void Sync()` — phase → 0, current value becomes the start point, new target drawn
  - `float Process(int block_size)` — advance one block, return value in [−1, 1]
  - observers: `bool new_target() const`, `float value() const`, `float phase() const`, `float current() const`, `float target() const`, `float curve() const`, `float solve_error() const`

This task implements everything except the curved families: `Evaluate()` handles only `curve == 0` (linear) and Task 6 adds the Bézier + Newton path. The header is written complete here so Task 6 only touches the `.cpp` and the test.

- [ ] **Step 1: Write the failing tests**

```cpp
// test/test_bezier_random.cpp
#include "test_assert.h"
#include "bezier_random.h"
#include <cmath>

using legio::BezierRandom;

static void test_output_stays_in_range_for_a_million_blocks() {
    BezierRandom g;
    g.Init(48000.0f, 12345u);
    g.SetPeriodSamples(4096.0f);
    g.SetCurve(0.0f);
    float lo = 1.0f, hi = -1.0f;
    for (int i = 0; i < 1000000; ++i) {
        float v = g.Process(48);
        if (v < lo) lo = v;
        if (v > hi) hi = v;
        EXPECT_TRUE(v >= -1.0f && v <= 1.0f);
        if (v < -1.0f || v > 1.0f) return;   // don't spam
    }
    EXPECT_TRUE(lo < -0.5f);   // it actually wanders
    EXPECT_TRUE(hi >  0.5f);
}

static void test_targets_change_exactly_at_period_boundaries() {
    BezierRandom g;
    g.Init(48000.0f, 7u);
    g.SetPeriodSamples(4096.0f);   // 64 blocks of 64 samples: phase step = 1/64 exactly
    int new_target_blocks = 0;
    for (int b = 1; b <= 256; ++b) {
        g.Process(64);
        if (g.new_target()) {
            ++new_target_blocks;
            EXPECT_EQ(b % 64, 0);   // only on blocks 64, 128, 192, 256
        }
    }
    EXPECT_EQ(new_target_blocks, 4);
}

static void test_linear_curve_is_a_straight_line() {
    BezierRandom g;
    g.Init(48000.0f, 1u);
    g.SetPeriodSamples(4096.0f);
    g.SetCurve(0.0f);
    g.SetEndpoints(-0.5f, 0.75f);
    for (int b = 1; b < 64; ++b) {
        float v   = g.Process(64);
        float phi = b / 64.0f;
        EXPECT_NEAR(v, -0.5f + 1.25f * phi, 1e-6);
    }
}

static void test_fixed_seed_is_deterministic() {
    BezierRandom a, b;
    a.Init(48000.0f, 99u);
    b.Init(48000.0f, 99u);
    a.SetPeriodSamples(4096.0f);
    b.SetPeriodSamples(4096.0f);
    for (int i = 0; i < 2000; ++i) {
        EXPECT_NEAR(a.Process(48), b.Process(48), 0.0);
    }
    BezierRandom c;
    c.Init(48000.0f, 100u);
    c.SetPeriodSamples(4096.0f);
    bool differs = false;
    for (int i = 0; i < 2000; ++i) {
        if (std::fabs(a.Process(48) - c.Process(48)) > 1e-6f) differs = true;
    }
    EXPECT_TRUE(differs);
}

static void test_sync_resets_phase_and_draws_a_target() {
    BezierRandom g;
    g.Init(48000.0f, 3u);
    g.SetPeriodSamples(4096.0f);
    for (int i = 0; i < 20; ++i) g.Process(64);   // phase = 20/64
    float before_value  = g.value();
    float before_target = g.target();
    g.Sync();
    EXPECT_NEAR(g.phase(), 0.0f, 1e-9);
    EXPECT_NEAR(g.current(), before_value, 1e-7);   // continues from where it was, no jump
    EXPECT_TRUE(std::fabs(g.target() - before_target) > 1e-6f);
    g.Process(64);
    EXPECT_TRUE(g.new_target());          // reported on the block after Sync
    EXPECT_NEAR(g.phase(), 1.0f / 64.0f, 1e-7);
    g.Process(64);
    EXPECT_TRUE(!g.new_target());         // one block only
}

static void test_period_change_takes_effect_immediately() {
    BezierRandom g;
    g.Init(48000.0f, 5u);
    g.SetPeriodSamples(4096.0f);
    for (int i = 0; i < 10; ++i) g.Process(64);
    EXPECT_NEAR(g.phase(), 10.0f / 64.0f, 1e-7);
    g.SetPeriodSamples(2048.0f);          // mid-cycle
    g.Process(64);                         // step is now 1/32
    EXPECT_NEAR(g.phase(), 10.0f / 64.0f + 1.0f / 32.0f, 1e-7);
}

static void run_all() {
    RUN_TEST(test_output_stays_in_range_for_a_million_blocks);
    RUN_TEST(test_targets_change_exactly_at_period_boundaries);
    RUN_TEST(test_linear_curve_is_a_straight_line);
    RUN_TEST(test_fixed_seed_is_deterministic);
    RUN_TEST(test_sync_resets_phase_and_draws_a_target);
    RUN_TEST(test_period_change_takes_effect_immediately);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

`TESTS = test_smoke test_edge test_cv_in test_clock test_bezier_random`, and:

```make
$(OUT)/test_bezier_random: test_bezier_random.cpp ../src/bezier_random.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `bezier_random.h: No such file or directory`.

- [ ] **Step 4: Write `src/bezier_random.h`** (complete; Task 6 does not change it)

```cpp
// src/bezier_random.h — random Bézier wander generator, one channel.
// Output is in [-1, +1]. Pure float math, no HAL.
#pragma once
#include <cstdint>

namespace legio {

class BezierRandom {
  public:
    void  Init(float sample_rate, uint32_t seed);
    void  SetPeriodSamples(float period_samples);   // read every block; clamped >= 1
    void  SetCurve(float curve);                    // -1 cusped .. 0 linear .. +1 plateau
    void  SetEndpoints(float v_cur, float v_next);  // test hook: fixes the cycle, phase -> 0
    void  Sync();                                   // phase -> 0, restart from current value toward a new target
    float Process(int block_size);                  // advance one block, return value

    bool  new_target()  const { return new_target_; }   // true for the one block a target was drawn
    float value()       const { return value_; }
    float phase()       const { return phase_; }
    float current()     const { return v_cur_; }
    float target()      const { return v_next_; }
    float curve()       const { return curve_; }
    float solve_error() const { return solve_error_; }  // |x(s) - phase| after the last solve

  private:
    float NextRandom();                 // uniform in [-1, +1)
    void  DrawTarget();                 // v_cur <- v_next, v_next <- random
    float Evaluate(float phi);          // value at time fraction phi of the current cycle
    void  ControlPoints(float& x1, float& y1, float& x2, float& y2) const;

    uint32_t rng_;
    float    sample_rate_;
    float    period_;
    float    curve_;
    float    phase_;
    float    s_;             // Bézier parameter, warm start for the solver
    float    v_cur_;
    float    v_next_;
    float    value_;
    float    solve_error_;
    bool     new_target_;
    bool     synced_;        // Sync() happened since the last Process()
};

}  // namespace legio
```

- [ ] **Step 5: Write `src/bezier_random.cpp` (linear path only)**

```cpp
// src/bezier_random.cpp
#include "bezier_random.h"
#include <cmath>

namespace legio {

void BezierRandom::Init(float sample_rate, uint32_t seed) {
    sample_rate_ = sample_rate;
    rng_         = seed ? seed : 0x9E3779B9u;   // xorshift must never hold 0
    period_      = 10.0f * sample_rate;
    curve_       = 0.0f;
    phase_       = 0.0f;
    s_           = 0.0f;
    v_cur_       = 0.0f;
    v_next_      = NextRandom();
    value_       = 0.0f;
    solve_error_ = 0.0f;
    new_target_  = false;
    synced_      = false;
}

float BezierRandom::NextRandom() {
    uint32_t x = rng_;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_ = x;
    // Top 24 bits -> [0, 1) -> [-1, +1).
    return (float)(x >> 8) * (2.0f / 16777216.0f) - 1.0f;
}

void BezierRandom::SetPeriodSamples(float p) { period_ = p < 1.0f ? 1.0f : p; }

void BezierRandom::SetCurve(float c) {
    curve_ = c < -1.0f ? -1.0f : (c > 1.0f ? 1.0f : c);
}

void BezierRandom::SetEndpoints(float v_cur, float v_next) {
    v_cur_  = v_cur;
    v_next_ = v_next;
    value_  = v_cur;
    phase_  = 0.0f;
    s_      = 0.0f;
}

void BezierRandom::DrawTarget() {
    v_cur_  = v_next_;
    v_next_ = NextRandom();
    s_      = 0.0f;
}

void BezierRandom::Sync() {
    v_cur_  = value_;          // continue from where we are: no jump
    v_next_ = NextRandom();
    phase_  = 0.0f;
    s_      = 0.0f;
    synced_ = true;
}

float BezierRandom::Process(int block_size) {
    bool nt = synced_;
    synced_ = false;
    phase_ += (float)block_size / period_;
    if (phase_ >= 1.0f) {
        phase_ -= std::floor(phase_);
        DrawTarget();
        nt = true;
    }
    new_target_ = nt;
    value_      = Evaluate(phase_);
    return value_;
}

void BezierRandom::ControlPoints(float& x1, float& y1, float& x2, float& y2) const {
    // Filled in by Task 6. Linear placeholder keeps the build green.
    x1 = 0.0f; y1 = v_cur_;
    x2 = 1.0f; y2 = v_next_;
}

float BezierRandom::Evaluate(float phi) {
    // Task 5: linear only. Task 6 adds the Bézier families.
    s_           = phi;
    solve_error_ = 0.0f;
    return v_cur_ + (v_next_ - v_cur_) * phi;
}

}  // namespace legio
```

- [ ] **Step 6: Add `src/bezier_random.cpp` to the firmware Makefile**

`CPP_SOURCES = src/main.cpp src/clock.cpp src/bezier_random.cpp`

- [ ] **Step 7: Run the tests and the firmware build**

```bash
make -C test && make
```

Expected: `test_bezier_random` shows 6 `PASS` lines; `All tests passed.`; firmware links.

- [ ] **Step 8: Commit**

```bash
git add src/bezier_random.h src/bezier_random.cpp test/test_bezier_random.cpp test/Makefile Makefile
git commit -m "feat(bezier_random): xorshift targets, cycle timing, linear ramp, Sync"
```

---

## Task 6: Bézier random generator — curve families and the Newton solve

**Files:**
- Modify: `src/bezier_random.cpp` (`ControlPoints`, `Evaluate`)
- Modify: `test/test_bezier_random.cpp` (add curve tests)

**Interfaces:**
- Consumes: everything from Task 5; no signature changes.
- Produces: `SetCurve(c)` now shapes the motion. `curve < 0` → control points move vertically (cusps, fast-slow-fast); `curve > 0` → horizontally (plateaus, slow-fast-slow); `solve_error()` reports the Newton residual.

The math, from the spec, with `Δ = v_next − v_cur`, `P0 = (0, v_cur)`, `P3 = (1, v_next)`:

- `curve < 0`: `k = 0.5·|curve|`, `P1 = (0, v_cur + kΔ)`, `P2 = (1, v_next − kΔ)`.
- `curve ≥ 0`: `k = 0.5·curve`, `P1 = (k, v_cur)`, `P2 = (1 − k, v_next)`.
- `x(s) = 3(1−s)²s·x1 + 3(1−s)s²·x2 + s³`, monotone non-decreasing because `0 ≤ x1 ≤ x2 ≤ 1`.
- `y(s) = (1−s)³·v_cur + 3(1−s)²s·y1 + 3(1−s)s²·y2 + s³·v_next`.
- Solve `x(s) = φ` with safeguarded Newton (bracket `[lo, hi]`, bisection when the Newton step leaves the bracket or `dx/ds ≈ 0`, which happens at `s = 0` and `s = 1` in the CCW family). Warm start from the previous `s`.

Note for the implementer: at `curve = 0` both families reduce to `P1 = (0, v_cur)`, `P2 = (1, v_next)`, which traces the same straight segment as the linear ramp, so the `curve == 0` shortcut is exact and the families are continuous through zero.

- [ ] **Step 1: Add the failing curve tests** to `test/test_bezier_random.cpp` (before `run_all`)

```cpp
// Samples one full cycle at phase step 1/4096 (period 4096, block 1) and returns
// the average slope dy/dphi over [phi_a, phi_b].
static float average_slope(BezierRandom& g, float phi_a, float phi_b) {
    // g must be at phase 0 with endpoints set. Step until phi_a, note value, step to phi_b.
    int a = (int)std::lround(phi_a * 4096.0f);
    int b = (int)std::lround(phi_b * 4096.0f);
    float va = g.current();
    for (int i = 1; i <= b; ++i) {
        float v = g.Process(1);
        if (i == a) va = v;
        if (a == 0) va = g.current();
    }
    float vb = g.value();
    return (vb - va) / (phi_b - phi_a);
}

static void test_ccw_curve_has_cusps() {
    // curve = -1: steep at the start (first 6.25%), flattest at the middle.
    BezierRandom g;
    g.Init(48000.0f, 1u);
    g.SetPeriodSamples(4096.0f);
    g.SetCurve(-1.0f);
    g.SetEndpoints(-0.8f, 0.6f);                 // delta = 1.4
    float start = average_slope(g, 0.0f, 0.0625f);
    g.SetEndpoints(-0.8f, 0.6f);
    float mid   = average_slope(g, 0.46875f, 0.53125f);
    EXPECT_TRUE(start > 2.0f * mid);
    EXPECT_TRUE(mid > 0.0f);
}

static void test_cw_curve_has_plateaus() {
    // curve = +1: near-zero slope at the start, largest slope at the middle.
    BezierRandom g;
    g.Init(48000.0f, 1u);
    g.SetPeriodSamples(4096.0f);
    g.SetCurve(1.0f);
    g.SetEndpoints(-0.8f, 0.6f);
    float start = average_slope(g, 0.0f, 0.0625f);
    g.SetEndpoints(-0.8f, 0.6f);
    float mid   = average_slope(g, 0.46875f, 0.53125f);
    EXPECT_TRUE(start < 0.2f * mid);
    EXPECT_TRUE(start >= 0.0f);
    EXPECT_NEAR(mid, 2.0f * 1.4f, 0.15f);         // full-CW midpoint slope is 2*delta (window average ~2.77)
}

static void test_curves_pass_through_the_same_endpoints() {
    // The "curves overlapped" property: every curve starts at v_cur and ends at v_next.
    const float curves[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
    for (float c : curves) {
        BezierRandom g;
        g.Init(48000.0f, 1u);
        g.SetPeriodSamples(4096.0f);
        g.SetCurve(c);
        g.SetEndpoints(0.25f, -0.9f);
        float first = g.Process(1);
        EXPECT_NEAR(first, 0.25f, 0.02);           // phi = 1/4096; cusped curves move ~1.5% here by design
        for (int i = 2; i < 4096; ++i) g.Process(1);
        EXPECT_NEAR(g.value(), -0.9f, 0.02);       // phi = 4095/4096
    }
}

static void test_curves_never_overshoot() {
    const float curves[] = {-1.0f, -0.7f, 0.3f, 1.0f};
    for (float c : curves) {
        BezierRandom g;
        g.Init(48000.0f, 42u);
        g.SetPeriodSamples(4096.0f);
        g.SetCurve(c);
        for (int i = 0; i < 200000; ++i) {
            g.Process(48);
            float lo = g.current() < g.target() ? g.current() : g.target();
            float hi = g.current() < g.target() ? g.target() : g.current();
            EXPECT_TRUE(g.value() >= lo - 1e-5f && g.value() <= hi + 1e-5f);
            if (g.value() < lo - 1e-5f || g.value() > hi + 1e-5f) return;
        }
    }
}

static void test_newton_solve_converges_every_block() {
    const float curves[] = {-1.0f, -0.3f, 0.3f, 0.66f, 1.0f};
    for (float c : curves) {
        BezierRandom g;
        g.Init(48000.0f, 9u);
        g.SetPeriodSamples(4096.0f);
        g.SetCurve(c);
        float worst = 0.0f;
        for (int i = 0; i < 100000; ++i) {
            g.Process(48);
            if (g.solve_error() > worst) worst = g.solve_error();
        }
        EXPECT_TRUE(worst < 1e-5f);
    }
    // Also with a very slow period (5 min) and a very fast one (20 Hz).
    BezierRandom slow, fast;
    slow.Init(48000.0f, 9u); slow.SetPeriodSamples(300.0f * 48000.0f); slow.SetCurve(1.0f);
    fast.Init(48000.0f, 9u); fast.SetPeriodSamples(2400.0f);           fast.SetCurve(-1.0f);
    float worst = 0.0f;
    for (int i = 0; i < 100000; ++i) {
        slow.Process(48); fast.Process(48);
        if (slow.solve_error() > worst) worst = slow.solve_error();
        if (fast.solve_error() > worst) worst = fast.solve_error();
    }
    EXPECT_TRUE(worst < 1e-5f);
}

static void test_curve_is_clamped() {
    BezierRandom g;
    g.Init(48000.0f, 1u);
    g.SetCurve(3.0f);
    EXPECT_NEAR(g.curve(), 1.0f, 1e-9);
    g.SetCurve(-3.0f);
    EXPECT_NEAR(g.curve(), -1.0f, 1e-9);
}
```

Add to `run_all`:

```cpp
    RUN_TEST(test_ccw_curve_has_cusps);
    RUN_TEST(test_cw_curve_has_plateaus);
    RUN_TEST(test_curves_pass_through_the_same_endpoints);
    RUN_TEST(test_curves_never_overshoot);
    RUN_TEST(test_newton_solve_converges_every_block);
    RUN_TEST(test_curve_is_clamped);
```

- [ ] **Step 2: Run the tests to verify the new ones fail**

```bash
make -C test
```

Expected: `test_ccw_curve_has_cusps` and `test_cw_curve_has_plateaus` FAIL (the linear placeholder has equal slopes everywhere); the others may pass trivially. Do not proceed until you have seen the failures.

- [ ] **Step 3: Replace `ControlPoints` and `Evaluate` in `src/bezier_random.cpp`**

Add these helpers to the top of the file, inside `namespace legio` before `Init`:

```cpp
namespace {
constexpr int   kMaxSolveIters = 16;
constexpr float kSolveTol      = 1e-6f;

inline float Bez(float p0, float p1, float p2, float p3, float s) {
    float u = 1.0f - s;
    return u * u * u * p0 + 3.0f * u * u * s * p1 + 3.0f * u * s * s * p2 + s * s * s * p3;
}
inline float BezDeriv(float p0, float p1, float p2, float p3, float s) {
    float u = 1.0f - s;
    return 3.0f * u * u * (p1 - p0) + 6.0f * u * s * (p2 - p1) + 3.0f * s * s * (p3 - p2);
}
}  // namespace
```

Then replace the two placeholder functions:

```cpp
void BezierRandom::ControlPoints(float& x1, float& y1, float& x2, float& y2) const {
    float d = v_next_ - v_cur_;
    if (curve_ < 0.0f) {
        // CCW: control points move vertically. Fast-slow-fast, cusps at the targets.
        float k = 0.5f * -curve_;
        x1 = 0.0f; y1 = v_cur_ + k * d;
        x2 = 1.0f; y2 = v_next_ - k * d;
    } else {
        // CW: control points move horizontally. Slow-fast-slow, plateaus at the targets.
        float k = 0.5f * curve_;
        x1 = k;        y1 = v_cur_;
        x2 = 1.0f - k; y2 = v_next_;
    }
}

float BezierRandom::Evaluate(float phi) {
    if (curve_ == 0.0f) {
        s_           = phi;
        solve_error_ = 0.0f;
        return v_cur_ + (v_next_ - v_cur_) * phi;
    }
    float x1, y1, x2, y2;
    ControlPoints(x1, y1, x2, y2);

    // Solve x(s) = phi. x(s) is monotone non-decreasing on [0, 1] because
    // 0 <= x1 <= x2 <= 1, so a bracket [lo, hi] always contains the root.
    // Newton from the previous s (phi only grows within a cycle, so the warm
    // start is already close); bisect whenever Newton would leave the bracket
    // or the slope vanishes (it is exactly 0 at s = 0 and s = 1 for curve < 0).
    float lo = 0.0f, hi = 1.0f;
    float s  = s_ < 0.0f ? 0.0f : (s_ > 1.0f ? 1.0f : s_);
    float f  = Bez(0.0f, x1, x2, 1.0f, s) - phi;
    for (int i = 0; i < kMaxSolveIters && std::fabs(f) > kSolveTol; ++i) {
        if (f < 0.0f) lo = s; else hi = s;
        float d    = BezDeriv(0.0f, x1, x2, 1.0f, s);
        float cand = (d > 1e-6f) ? s - f / d : -1.0f;
        s = (cand > lo && cand < hi) ? cand : 0.5f * (lo + hi);
        f = Bez(0.0f, x1, x2, 1.0f, s) - phi;
    }
    s_           = s;
    solve_error_ = std::fabs(f);
    return Bez(v_cur_, y1, y2, v_next_, s);
}
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
make -C test
```

Expected: `test_bezier_random` shows 12 `PASS` lines, `All tests passed.` If `test_newton_solve_converges_every_block` fails, print the worst `solve_error()` and the curve; the fix is a larger `kMaxSolveIters`, never a looser test tolerance.

- [ ] **Step 5: Commit**

```bash
git add src/bezier_random.cpp test/test_bezier_random.cpp
git commit -m "feat(bezier_random): CCW cusp / CW plateau Bézier families with safeguarded Newton solve"
```

---

## Task 7: Panner — PAN / XFADE with equal-power gains and the position smoother

**Files:**
- Create: `src/panner.h`
- Create: `src/panner.cpp`
- Create: `test/test_panner.cpp`
- Modify: `test/Makefile` (add `test_panner`)
- Modify: `Makefile` (add `src/panner.cpp`)

**Interfaces:**
- Consumes: `legio::Mode` from `params.h`.
- Produces: `class legio::Panner`:
  - `void Init(float sample_rate)` — mode PAN, position 0.5 snapped, CV source 0
  - `void SetMode(Mode m)`
  - `void SetInternalSource(bool on)` (used in Task 8)
  - `void SetPositionTarget(float pos)` — per block, 0..1; smoothed per sample (~5 ms)
  - `void SetPositionImmediate(float pos)` — snaps target and smoothed value (Init + tests)
  - `void SetCvSource(float source)` — per block, volts/5; smoothed per sample (~2 ms) (used in Task 8)
  - `void ProcessBlock(const float* in_l, const float* in_r, float* out_l, float* out_r, int n)`
  - `float smoothed_position() const`
  - `static void EqualPowerGains(float pos, float& g_l, float& g_r)`

- [ ] **Step 1: Write the failing tests**

```cpp
// test/test_panner.cpp
#include "test_assert.h"
#include "panner.h"
#include <cmath>

using namespace legio;

static const int kN = 48;

static void fill(float* buf, float v) { for (int i = 0; i < kN; ++i) buf[i] = v; }

static void test_equal_power_center_is_minus_3db() {
    float gl, gr;
    Panner::EqualPowerGains(0.5f, gl, gr);
    EXPECT_NEAR(gl, 0.70710678f, 1e-5);
    EXPECT_NEAR(gr, 0.70710678f, 1e-5);
    EXPECT_NEAR(gl * gl + gr * gr, 1.0f, 1e-5);   // constant power everywhere
    Panner::EqualPowerGains(0.2f, gl, gr);
    EXPECT_NEAR(gl * gl + gr * gr, 1.0f, 1e-5);
}

static void test_pan_hard_left_and_right() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::PAN);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f); fill(in_r, 1.0f);

    p.SetPositionImmediate(0.0f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1], 1.0f, 1e-6);
    EXPECT_NEAR(out_r[kN - 1], 0.0f, 1e-6);

    p.SetPositionImmediate(1.0f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1], 0.0f, 1e-6);
    EXPECT_NEAR(out_r[kN - 1], 1.0f, 1e-6);
}

static void test_pan_each_output_carries_only_its_own_input() {
    // Stereo balance: hard right must mute Out L, and Out R must never contain In L.
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::PAN);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f); fill(in_r, 0.0f);   // signal only on L
    p.SetPositionImmediate(1.0f);         // hard right
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1], 0.0f, 1e-6);
    EXPECT_NEAR(out_r[kN - 1], 0.0f, 1e-6);   // In L never leaks to Out R
    p.SetPositionImmediate(0.5f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1], 0.70710678f, 1e-5);
    EXPECT_NEAR(out_r[kN - 1], 0.0f, 1e-6);
}

static void test_pan_with_identical_inputs_is_a_mono_panner() {
    // The hardware-normalling case: In R == In L.
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::PAN);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 0.5f); fill(in_r, 0.5f);
    p.SetPositionImmediate(0.25f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    float gl, gr;
    Panner::EqualPowerGains(0.25f, gl, gr);
    EXPECT_NEAR(out_l[kN - 1], 0.5f * gl, 1e-6);
    EXPECT_NEAR(out_r[kN - 1], 0.5f * gr, 1e-6);
}

static void test_xfade_out_r_is_the_complement_of_out_l() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::XFADE);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f);   // A
    fill(in_r, -1.0f);  // B
    const float positions[] = {0.0f, 0.3f, 0.5f, 0.8f, 1.0f};
    for (float pos : positions) {
        p.SetPositionImmediate(pos);
        p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
        float gl, gr;
        Panner::EqualPowerGains(pos, gl, gr);
        EXPECT_NEAR(out_l[kN - 1], 1.0f * gl + -1.0f * gr, 1e-6);   // A*gA + B*gB
        EXPECT_NEAR(out_r[kN - 1], 1.0f * gr + -1.0f * gl, 1e-6);   // A*gB + B*gA
    }
    // pos = 0: Out L is pure A, Out R is pure B.
    p.SetPositionImmediate(0.0f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1],  1.0f, 1e-6);
    EXPECT_NEAR(out_r[kN - 1], -1.0f, 1e-6);
}

static void test_position_smoother_never_zippers() {
    // Jump the target 0 -> 1 and check the per-sample gain step stays small.
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::PAN);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f); fill(in_r, 1.0f);
    p.SetPositionImmediate(0.0f);
    p.SetPositionTarget(1.0f);
    float prev = 1.0f;
    float max_step = 0.0f;
    for (int b = 0; b < 100; ++b) {
        p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
        for (int i = 0; i < kN; ++i) {
            float step = std::fabs(out_l[i] - prev);
            if (step > max_step) max_step = step;
            prev = out_l[i];
        }
    }
    EXPECT_TRUE(max_step < 0.01f);                  // ~5 ms one-pole: alpha*pi/2 ~ 0.0065
    EXPECT_NEAR(p.smoothed_position(), 1.0f, 1e-3); // and it does get there (100 ms)
}

static void test_smoother_time_constant_is_about_5ms() {
    Panner p;
    p.Init(48000.0f);
    p.SetPositionImmediate(0.0f);
    p.SetPositionTarget(1.0f);
    float in[kN] = {0}, out_l[kN], out_r[kN];
    for (int b = 0; b < 5; ++b) p.ProcessBlock(in, in, out_l, out_r, kN);   // 240 samples = 5 ms
    EXPECT_NEAR(p.smoothed_position(), 1.0f - std::exp(-1.0f), 0.02f);       // one time constant
}

static void run_all() {
    RUN_TEST(test_equal_power_center_is_minus_3db);
    RUN_TEST(test_pan_hard_left_and_right);
    RUN_TEST(test_pan_each_output_carries_only_its_own_input);
    RUN_TEST(test_pan_with_identical_inputs_is_a_mono_panner);
    RUN_TEST(test_xfade_out_r_is_the_complement_of_out_l);
    RUN_TEST(test_position_smoother_never_zippers);
    RUN_TEST(test_smoother_time_constant_is_about_5ms);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

`TESTS = test_smoke test_edge test_cv_in test_clock test_bezier_random test_panner`, and:

```make
$(OUT)/test_panner: test_panner.cpp ../src/panner.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `panner.h: No such file or directory`.

- [ ] **Step 4: Write `src/panner.h`** (complete, including the CV members Task 8 fills in)

```cpp
// src/panner.h — mode + smoothed position -> per-sample gains and output mixing.
#pragma once
#include "params.h"

namespace legio {

class Panner {
  public:
    void Init(float sample_rate);

    void SetMode(Mode m)            { mode_ = m; }
    void SetInternalSource(bool on) { internal_ = on; }   // CV mode: +5 V constant instead of the jack

    void SetPositionTarget(float pos);      // per block; smoothed per sample (~5 ms)
    void SetPositionImmediate(float pos);   // snap (Init, tests)
    void SetCvSource(float source);         // per block, volts / 5; smoothed per sample (~2 ms)

    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, int n);

    float smoothed_position() const { return pos_s_; }

    // gL = cos(pos*pi/2), gR = sin(pos*pi/2). Center = -3 dB per side.
    static void EqualPowerGains(float pos, float& g_l, float& g_r);

  private:
    Mode  mode_;
    bool  internal_;
    float pos_target_;
    float pos_s_;
    float pos_alpha_;
    float cv_target_;
    float cv_s_;
    float cv_alpha_;
};

}  // namespace legio
```

- [ ] **Step 5: Write `src/panner.cpp` (PAN / XFADE; CV branch outputs silence until Task 8)**

```cpp
// src/panner.cpp
#include "panner.h"
#include <cmath>

namespace legio {

namespace {
constexpr float kPosSmoothSec = 0.005f;   // ~5 ms one-pole on position
constexpr float kCvSmoothSec  = 0.002f;   // ~2 ms one-pole on the CV source
constexpr float kHalfPi       = 1.57079632679f;

inline float OnePoleAlpha(float seconds, float sample_rate) {
    return 1.0f - std::exp(-1.0f / (seconds * sample_rate));
}
inline float Clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }
}  // namespace

void Panner::Init(float sample_rate) {
    mode_      = Mode::PAN;
    internal_  = false;
    pos_alpha_ = OnePoleAlpha(kPosSmoothSec, sample_rate);
    cv_alpha_  = OnePoleAlpha(kCvSmoothSec, sample_rate);
    SetPositionImmediate(0.5f);
    cv_target_ = 0.0f;
    cv_s_      = 0.0f;
}

void Panner::SetPositionTarget(float pos)    { pos_target_ = Clamp01(pos); }
void Panner::SetPositionImmediate(float pos) { pos_target_ = pos_s_ = Clamp01(pos); }
void Panner::SetCvSource(float source)       { cv_target_ = source; }

void Panner::EqualPowerGains(float pos, float& g_l, float& g_r) {
    float a = pos * kHalfPi;
    g_l = std::cos(a);
    g_r = std::sin(a);
}

void Panner::ProcessBlock(const float* in_l, const float* in_r,
                          float* out_l, float* out_r, int n) {
    for (int i = 0; i < n; ++i) {
        pos_s_ += pos_alpha_ * (pos_target_ - pos_s_);
        cv_s_  += cv_alpha_  * (cv_target_  - cv_s_);
        switch (mode_) {
            case Mode::PAN: {
                float gl, gr;
                EqualPowerGains(pos_s_, gl, gr);
                out_l[i] = in_l[i] * gl;
                out_r[i] = in_r[i] * gr;
                break;
            }
            case Mode::XFADE: {
                float ga, gb;
                EqualPowerGains(pos_s_, ga, gb);
                out_l[i] = in_l[i] * ga + in_r[i] * gb;   // the crossfade
                out_r[i] = in_l[i] * gb + in_r[i] * ga;   // the complementary crossfade
                break;
            }
            case Mode::CV:
            default:
                out_l[i] = 0.0f;   // Task 8
                out_r[i] = 0.0f;
                break;
        }
    }
}

}  // namespace legio
```

- [ ] **Step 6: Add `src/panner.cpp` to the firmware Makefile**

`CPP_SOURCES = src/main.cpp src/clock.cpp src/bezier_random.cpp src/panner.cpp`

- [ ] **Step 7: Run the tests and the firmware build**

```bash
make -C test && make
```

Expected: `test_panner` shows 7 `PASS` lines, `All tests passed.`; firmware links.

- [ ] **Step 8: Commit**

```bash
git add src/panner.h src/panner.cpp test/test_panner.cpp test/Makefile Makefile
git commit -m "feat(panner): equal-power PAN / XFADE with per-sample position smoothing"
```

---

## Task 8: Panner — CV mode (jack source and internal +5 V source)

**Files:**
- Modify: `src/panner.cpp` (`Mode::CV` branch)
- Modify: `test/test_panner.cpp` (add CV tests)

**Interfaces:**
- Consumes: Task 7's `Panner`.
- Produces: in `Mode::CV`, `out_l = source · (1 − pos_s)`, `out_r = source · pos_s`, where `source` is the smoothed `SetCvSource` value, or exactly `1.0` when `SetInternalSource(true)`. Audio inputs are ignored.

- [ ] **Step 1: Add the failing CV tests** to `test/test_panner.cpp` (before `run_all`)

```cpp
static void test_cv_linear_law_sums_to_source() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::CV);
    p.SetCvSource(0.6f);   // 3 V
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f); fill(in_r, 1.0f);
    for (int b = 0; b < 50; ++b) p.ProcessBlock(in_l, in_r, out_l, out_r, kN);   // let the 2 ms smoother settle
    const float positions[] = {0.0f, 0.25f, 0.5f, 0.9f, 1.0f};
    for (float pos : positions) {
        p.SetPositionImmediate(pos);
        p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
        EXPECT_NEAR(out_l[kN - 1], 0.6f * (1.0f - pos), 1e-4);
        EXPECT_NEAR(out_r[kN - 1], 0.6f * pos, 1e-4);
        EXPECT_NEAR(out_l[kN - 1] + out_r[kN - 1], 0.6f, 1e-4);
    }
}

static void test_cv_mode_ignores_audio_inputs() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::CV);
    p.SetCvSource(0.0f);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 0.9f); fill(in_r, -0.9f);
    p.SetPositionImmediate(0.5f);
    for (int b = 0; b < 10; ++b) p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    for (int i = 0; i < kN; ++i) {
        EXPECT_NEAR(out_l[i], 0.0f, 1e-7);
        EXPECT_NEAR(out_r[i], 0.0f, 1e-7);
    }
}

static void test_internal_source_sums_to_five_volts_and_ignores_jack() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::CV);
    p.SetInternalSource(true);
    p.SetCvSource(0.3f);   // must be ignored
    float in_l[kN] = {0}, out_l[kN], out_r[kN];
    for (int b = 0; b < 50; ++b) p.ProcessBlock(in_l, in_l, out_l, out_r, kN);
    const float positions[] = {0.0f, 0.4f, 1.0f};
    for (float pos : positions) {
        p.SetPositionImmediate(pos);
        p.ProcessBlock(in_l, in_l, out_l, out_r, kN);
        EXPECT_NEAR(out_l[kN - 1], 1.0f - pos, 1e-6);
        EXPECT_NEAR(out_r[kN - 1], pos, 1e-6);
        EXPECT_NEAR(out_l[kN - 1] + out_r[kN - 1], 1.0f, 1e-6);   // = 5 V
    }
    p.SetInternalSource(false);
    for (int b = 0; b < 50; ++b) p.ProcessBlock(in_l, in_l, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1] + out_r[kN - 1], 0.3f, 1e-4);        // back to the jack
}

static void test_internal_source_only_matters_in_cv_mode() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::PAN);
    p.SetInternalSource(true);
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    fill(in_l, 1.0f); fill(in_r, 1.0f);
    p.SetPositionImmediate(0.0f);
    p.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    EXPECT_NEAR(out_l[kN - 1], 1.0f, 1e-6);   // still audio panning
    EXPECT_NEAR(out_r[kN - 1], 0.0f, 1e-6);
}

static void test_cv_source_is_smoothed_not_stepped() {
    Panner p;
    p.Init(48000.0f);
    p.SetMode(Mode::CV);
    p.SetPositionImmediate(0.0f);   // Out L = source
    p.SetCvSource(1.0f);
    float in[kN] = {0}, out_l[kN], out_r[kN];
    p.ProcessBlock(in, in, out_l, out_r, kN);
    EXPECT_TRUE(out_l[0] < 0.05f);              // first sample is not the full step
    EXPECT_TRUE(out_l[kN - 1] > out_l[0]);
    for (int b = 0; b < 2; ++b) p.ProcessBlock(in, in, out_l, out_r, kN);   // 144 samples = 3 ms = 1.5 tau
    EXPECT_NEAR(out_l[kN - 1], 1.0f - std::exp(-1.5f), 0.03f);
}
```

Add to `run_all`:

```cpp
    RUN_TEST(test_cv_linear_law_sums_to_source);
    RUN_TEST(test_cv_mode_ignores_audio_inputs);
    RUN_TEST(test_internal_source_sums_to_five_volts_and_ignores_jack);
    RUN_TEST(test_internal_source_only_matters_in_cv_mode);
    RUN_TEST(test_cv_source_is_smoothed_not_stepped);
```

- [ ] **Step 2: Run the tests to verify the new ones fail**

```bash
make -C test
```

Expected: `test_cv_linear_law_sums_to_source`, `test_internal_source_sums_to_five_volts_and_ignores_jack`, `test_cv_source_is_smoothed_not_stepped` FAIL (CV branch outputs zeros).

- [ ] **Step 3: Implement the CV branch in `src/panner.cpp`**

Replace the `case Mode::CV:` block:

```cpp
            case Mode::CV:
            default: {
                // Linear law so the two outputs always sum to the source: an envelope
                // split across two destinations keeps its total "presence".
                float src = internal_ ? 1.0f : cv_s_;   // 1.0 = +5 V at the DC-coupled output
                out_l[i] = src * (1.0f - pos_s_);
                out_r[i] = src * pos_s_;
                break;
            }
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
make -C test
```

Expected: `test_panner` shows 12 `PASS` lines, `All tests passed.`

- [ ] **Step 5: Commit**

```bash
git add src/panner.cpp test/test_panner.cpp
git commit -m "feat(panner): CV mode with linear law, jack source, and internal +5 V source"
```

---

## Task 9: Drifter chain — rate, ratio, curve edit, internal source, position pipeline

**Files:**
- Create: `src/drifter_chain.h`
- Create: `src/drifter_chain.cpp`
- Create: `test/test_chain.cpp`
- Modify: `test/Makefile` (add `test_chain`)
- Modify: `Makefile` (add `src/drifter_chain.cpp`)

**Interfaces:**
- Consumes: `Params`, `Mode`, `Edge` (`params.h`); `Clock` (`clock.h`: `period_samples()`, `tick()`, `is_external()`); `BezierRandom` (`SetPeriodSamples`, `SetCurve`, `Sync`, `Process`, `new_target`, `value`, `curve`); `Panner` (`SetMode`, `SetInternalSource`, `SetPositionTarget`, `SetCvSource`, `ProcessBlock`, `smoothed_position`); `apply_edge` (`edge.h`); `cv_volts_from_norm`, `cv_source_from_volts` (`cv_in.h`).
- Produces: `class legio::DrifterChain`:
  - `void Init(float sample_rate, uint32_t seed)`
  - `void ApplyParams(const Params& p, const Clock& clk, int block_size)` — once per block, before `ProcessBlock`; steps the generator
  - `void ProcessBlock(const float* in_l, const float* in_r, float* out_l, float* out_r, int n)`
  - observers for the HAL / telemetry: `float position() const`, `bool new_target() const`, `bool curve_edit() const`, `float curve() const`, `bool internal_source() const`, `bool internal_source_active() const` (= mode is CV and internal source on), `float period_samples() const`, `bool clocked() const`, `int rate_index() const`, `int ratio_index() const`, `float center() const`, `float depth() const`, `float wander() const`, `float cv_volts() const`
  - constants exposed for tests / telemetry: `static constexpr int kRateSteps = 38`, `kRatioCount = 7`, `kCurveSteps = 12`; `static float FreePeriodSeconds(int rate_index)`; `static float RatioMultiplier(int ratio_index)`

Decisions fixed here (the spec leaves them open): boot defaults are `rate_index = 15` (≈ 9.7 s), `ratio_index = 3` (×1), `curve_steps = +6` (curve = +0.5, the eased "ambient" region), curve-edit off, internal source off. The free-running period is `300 s · (0.05 / 300)^(rate_index / 38)`, i.e. 5 min at index 0, 20 Hz at index 38, ≈ 1/3 octave per detent (12.55 octaves / 38).

- [ ] **Step 1: Write the failing tests**

```cpp
// test/test_chain.cpp
#include "test_assert.h"
#include "drifter_chain.h"
#include "clock.h"
#include <cmath>

using namespace legio;

static const int kN = 48;

struct Rig {
    DrifterChain chain;
    Clock        clock;
    float in_l[kN], in_r[kN], out_l[kN], out_r[kN];
    Rig() {
        chain.Init(48000.0f, 1234u);
        clock.Init(48000.0f);
        for (int i = 0; i < kN; ++i) { in_l[i] = 1.0f; in_r[i] = 1.0f; }
    }
    // One block with the given params; gate_edge is fed to the clock too.
    void step(Params p) {
        clock.update(p.gate_edge, kN);
        chain.ApplyParams(p, clock, kN);
        chain.ProcessBlock(in_l, in_r, out_l, out_r, kN);
    }
    void run(Params p, int blocks) { for (int i = 0; i < blocks; ++i) step(p); }
};

static void test_defaults() {
    Rig r;
    EXPECT_EQ(r.chain.rate_index(), 15);
    EXPECT_EQ(r.chain.ratio_index(), 3);
    EXPECT_NEAR(r.chain.curve(), 0.5f, 1e-6);
    EXPECT_TRUE(!r.chain.curve_edit());
    EXPECT_TRUE(!r.chain.internal_source());
    EXPECT_NEAR(DrifterChain::FreePeriodSeconds(0),  300.0f, 1e-3);
    EXPECT_NEAR(DrifterChain::FreePeriodSeconds(38), 0.05f,  1e-5);
    EXPECT_NEAR(DrifterChain::RatioMultiplier(0), 8.0f,   1e-6);   // ÷8: 8 clocks per target
    EXPECT_NEAR(DrifterChain::RatioMultiplier(3), 1.0f,   1e-6);
    EXPECT_NEAR(DrifterChain::RatioMultiplier(6), 0.125f, 1e-6);   // ×8: 8 targets per clock
}

static void test_depth_zero_means_pos_equals_center() {
    Rig r;
    Params p;
    p.top_knob = 0.3f;
    p.bottom_knob = 0.0f;
    p.mode = Mode::PAN;
    r.run(p, 400);   // plenty for the 5 ms smoother
    EXPECT_NEAR(r.chain.position(), 0.3f, 1e-4);
    p.top_knob = 0.8f;
    r.run(p, 400);
    EXPECT_NEAR(r.chain.position(), 0.8f, 1e-4);
}

static void test_tap_toggles_curve_edit_mode() {
    Rig r;
    Params p;
    p.encoder_tap = true;
    r.step(p);
    EXPECT_TRUE(r.chain.curve_edit());
    p.encoder_tap = false;
    r.run(p, 5);
    EXPECT_TRUE(r.chain.curve_edit());   // sticky
    p.encoder_tap = true;
    r.step(p);
    EXPECT_TRUE(!r.chain.curve_edit());
}

static void test_encoder_edits_rate_in_normal_mode_and_curve_in_edit_mode() {
    Rig r;
    Params p;
    float period0 = r.chain.period_samples();
    float curve0  = r.chain.curve();

    p.encoder_increment = +1;   // normal: faster
    r.step(p);
    EXPECT_TRUE(r.chain.period_samples() < period0);
    EXPECT_NEAR(r.chain.curve(), curve0, 1e-9);
    EXPECT_EQ(r.chain.rate_index(), 16);

    p.encoder_increment = 0;
    p.encoder_tap = true;
    r.step(p);                  // enter edit mode
    float period1 = r.chain.period_samples();
    p.encoder_tap = false;
    p.encoder_increment = +1;
    r.step(p);
    EXPECT_NEAR(r.chain.curve(), curve0 + 1.0f / 12.0f, 1e-6);
    EXPECT_NEAR(r.chain.period_samples(), period1, 1e-3);   // rate untouched
    EXPECT_EQ(r.chain.rate_index(), 16);
}

static void test_rate_and_curve_clamp_at_their_ends() {
    Rig r;
    Params p;
    p.encoder_increment = +1;
    r.run(p, 60);
    EXPECT_EQ(r.chain.rate_index(), 38);
    EXPECT_NEAR(r.chain.period_samples(), 2400.0f, 1.0f);       // 20 Hz
    p.encoder_increment = -1;
    r.run(p, 60);
    EXPECT_EQ(r.chain.rate_index(), 0);
    EXPECT_NEAR(r.chain.period_samples(), 300.0f * 48000.0f, 10.0f);

    p.encoder_increment = 0; p.encoder_tap = true; r.step(p); p.encoder_tap = false;
    p.encoder_increment = +1; r.run(p, 40);
    EXPECT_NEAR(r.chain.curve(), 1.0f, 1e-6);
    p.encoder_increment = -1; r.run(p, 40);
    EXPECT_NEAR(r.chain.curve(), -1.0f, 1e-6);
}

static void test_long_press_toggles_internal_source_only_in_cv_mode() {
    Rig r;
    Params p;
    p.mode = Mode::PAN;
    p.encoder_long_press = true;
    r.step(p);
    EXPECT_TRUE(!r.chain.internal_source());   // ignored in PAN
    p.mode = Mode::XFADE;
    r.step(p);
    EXPECT_TRUE(!r.chain.internal_source());   // ignored in XFADE
    p.mode = Mode::CV;
    r.step(p);
    EXPECT_TRUE(r.chain.internal_source());
    EXPECT_TRUE(r.chain.internal_source_active());
    p.encoder_long_press = false;
    // Survives a trip through PAN and back.
    p.mode = Mode::PAN;
    r.run(p, 3);
    EXPECT_TRUE(r.chain.internal_source());
    EXPECT_TRUE(!r.chain.internal_source_active());
    p.mode = Mode::CV;
    r.step(p);
    EXPECT_TRUE(r.chain.internal_source_active());
    p.encoder_long_press = true;
    r.step(p);
    EXPECT_TRUE(!r.chain.internal_source());   // toggled off
}

static void test_internal_source_outputs_sum_to_one_in_cv_mode() {
    Rig r;
    Params p;
    p.mode = Mode::CV;
    p.top_knob = 0.7f;
    p.bottom_knob = 0.0f;
    p.cv_norm = 0.3019f;          // 0 V at the jack
    p.encoder_long_press = true;
    r.step(p);
    p.encoder_long_press = false;
    r.run(p, 400);
    EXPECT_NEAR(r.out_l[kN - 1] + r.out_r[kN - 1], 1.0f, 1e-4);
    EXPECT_NEAR(r.out_r[kN - 1], 0.7f, 1e-3);
}

static void test_cv_mode_with_jack_source() {
    Rig r;
    Params p;
    p.mode = Mode::CV;
    p.top_knob = 0.5f;
    p.bottom_knob = 0.0f;
    p.cv_norm = 0.3019f + 2.5f / 7.6805f;   // +2.5 V
    r.run(p, 400);
    EXPECT_NEAR(r.chain.cv_volts(), 2.5f, 1e-3);
    EXPECT_NEAR(r.out_l[kN - 1] + r.out_r[kN - 1], 0.5f, 1e-3);   // 2.5 V total
    p.cv_norm = 0.3019f;          // 0 V: clean silence
    r.run(p, 400);
    EXPECT_NEAR(r.out_l[kN - 1], 0.0f, 1e-6);
    EXPECT_NEAR(r.out_r[kN - 1], 0.0f, 1e-6);
}

static void test_fold_keeps_position_in_unit_interval_for_any_center_and_depth() {
    Rig r;
    Params p;
    p.edge = Edge::FOLD;
    p.encoder_increment = +1;
    r.run(p, 60);                  // fastest rate so the wander actually moves
    p.encoder_increment = 0;
    for (int c = 0; c <= 4; ++c) {
        for (int d = 0; d <= 4; ++d) {
            p.top_knob    = c * 0.25f;
            p.bottom_knob = d * 0.25f;
            for (int b = 0; b < 2000; ++b) {
                r.step(p);
                float pos = r.chain.position();
                EXPECT_TRUE(pos >= 0.0f && pos <= 1.0f);
                if (pos < 0.0f || pos > 1.0f) return;
            }
        }
    }
}

static void test_wrap_never_zippers_the_output() {
    // WRAP at center 0.95, depth 1 crosses the seam constantly; the smoother must
    // keep every per-sample output step small.
    Rig r;
    Params p;
    p.mode = Mode::PAN;
    p.edge = Edge::WRAP;
    p.top_knob = 0.95f;
    p.bottom_knob = 1.0f;
    p.encoder_increment = +1;
    r.run(p, 60);
    p.encoder_increment = 0;
    float prev = r.out_l[kN - 1];
    float max_step = 0.0f;
    for (int b = 0; b < 5000; ++b) {
        r.step(p);
        for (int i = 0; i < kN; ++i) {
            float step = std::fabs(r.out_l[i] - prev);
            if (step > max_step) max_step = step;
            prev = r.out_l[i];
        }
    }
    EXPECT_TRUE(max_step < 0.01f);
}

static void test_clock_edges_sync_the_generator() {
    Rig r;
    Params p;
    p.bottom_knob = 1.0f;
    // Establish a 12000-sample external clock (250 blocks).
    for (int e = 0; e < 3; ++e) {
        p.gate_edge = true;  r.step(p);
        p.gate_edge = false; r.run(p, 249);
    }
    EXPECT_TRUE(r.chain.clocked());
    EXPECT_NEAR(r.chain.period_samples(), 12000.0f, 100.0f);   // ratio x1
    // The next edge lands a new target on that block.
    p.gate_edge = true;  r.step(p);
    EXPECT_TRUE(r.chain.new_target());
    p.gate_edge = false; r.step(p);
    EXPECT_TRUE(!r.chain.new_target());
}

static void test_clock_ratio_divides_and_multiplies() {
    Rig r;
    Params p;
    for (int e = 0; e < 3; ++e) {
        p.gate_edge = true;  r.step(p);
        p.gate_edge = false; r.run(p, 249);
    }
    EXPECT_TRUE(r.chain.clocked());
    // Encoder now edits the ratio: -1 -> ÷2.
    p.encoder_increment = -1; r.step(p); p.encoder_increment = 0;
    EXPECT_EQ(r.chain.ratio_index(), 2);
    EXPECT_NEAR(r.chain.period_samples(), 24000.0f, 200.0f);
    // ÷2: only every second edge syncs.
    int syncs = 0;
    for (int e = 0; e < 4; ++e) {
        p.gate_edge = true;  r.step(p);
        if (r.chain.new_target()) ++syncs;
        p.gate_edge = false; r.run(p, 249);
    }
    EXPECT_EQ(syncs, 2);
    // +3 -> ×4: period is a quarter of the clock.
    p.encoder_increment = +1; r.run(p, 3); p.encoder_increment = 0;
    EXPECT_EQ(r.chain.ratio_index(), 5);
    EXPECT_NEAR(r.chain.period_samples(), 3000.0f, 50.0f);
    // The free-running rate index was not touched by any of this.
    EXPECT_EQ(r.chain.rate_index(), 15);
}

static void test_free_rate_restored_after_clock_falls_back() {
    Rig r;
    Params p;
    p.encoder_increment = +1; r.run(p, 5); p.encoder_increment = 0;   // rate_index 20
    float free_period = r.chain.period_samples();
    for (int e = 0; e < 3; ++e) {
        p.gate_edge = true;  r.step(p);
        p.gate_edge = false; r.run(p, 249);
    }
    EXPECT_TRUE(r.chain.clocked());
    r.run(p, 2100);   // > 2 s silence -> fallback
    EXPECT_TRUE(!r.chain.clocked());
    EXPECT_NEAR(r.chain.period_samples(), free_period, 1e-3);
    EXPECT_EQ(r.chain.rate_index(), 20);
}

static void test_mode_switching_every_block_is_bounded_and_finite() {
    Rig r;
    Params p;
    p.bottom_knob = 1.0f;
    p.cv_norm = 0.3019f + 5.0f / 7.6805f;
    for (int b = 0; b < 300; ++b) {
        p.mode = (Mode)(b % 3);
        p.edge = (Edge)(b % 3);
        r.step(p);
        for (int i = 0; i < kN; ++i) {
            EXPECT_TRUE(std::isfinite(r.out_l[i]) && std::isfinite(r.out_r[i]));
            EXPECT_TRUE(std::fabs(r.out_l[i]) <= 2.0f && std::fabs(r.out_r[i]) <= 2.0f);
            if (!std::isfinite(r.out_l[i])) return;
        }
    }
}

static void run_all() {
    RUN_TEST(test_defaults);
    RUN_TEST(test_depth_zero_means_pos_equals_center);
    RUN_TEST(test_tap_toggles_curve_edit_mode);
    RUN_TEST(test_encoder_edits_rate_in_normal_mode_and_curve_in_edit_mode);
    RUN_TEST(test_rate_and_curve_clamp_at_their_ends);
    RUN_TEST(test_long_press_toggles_internal_source_only_in_cv_mode);
    RUN_TEST(test_internal_source_outputs_sum_to_one_in_cv_mode);
    RUN_TEST(test_cv_mode_with_jack_source);
    RUN_TEST(test_fold_keeps_position_in_unit_interval_for_any_center_and_depth);
    RUN_TEST(test_wrap_never_zippers_the_output);
    RUN_TEST(test_clock_edges_sync_the_generator);
    RUN_TEST(test_clock_ratio_divides_and_multiplies);
    RUN_TEST(test_free_rate_restored_after_clock_falls_back);
    RUN_TEST(test_mode_switching_every_block_is_bounded_and_finite);
}

TEST_MAIN()
```

- [ ] **Step 2: Add the test to `test/Makefile`**

`TESTS = test_smoke test_edge test_cv_in test_clock test_bezier_random test_panner test_chain`, and:

```make
$(OUT)/test_chain: test_chain.cpp ../src/drifter_chain.cpp ../src/bezier_random.cpp \
        ../src/panner.cpp ../src/clock.cpp | $(OUT)
	$(CXX) $(CXXFLAGS) $^ -o $@
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
make -C test
```

Expected: compile error `drifter_chain.h: No such file or directory`.

- [ ] **Step 4: Write `src/drifter_chain.h`**

```cpp
// src/drifter_chain.h — orchestrator. Owns the generator and the panner, holds the
// rate / ratio / curve / edit-mode / internal-source state, and turns Params into
// a smoothed position and mixed outputs once per block. Pure float, no HAL.
#pragma once
#include <cstdint>
#include "bezier_random.h"
#include "clock.h"
#include "panner.h"
#include "params.h"

namespace legio {

class DrifterChain {
  public:
    static constexpr int kRateSteps  = 38;   // free-running: 5 min (0) .. 20 Hz (38), ~1/3 oct per detent
    static constexpr int kRatioCount = 7;    // clocked: ÷8 ÷4 ÷2 ×1 ×2 ×4 ×8
    static constexpr int kCurveSteps = 12;   // curve = steps / 12, steps in -12..+12

    static float FreePeriodSeconds(int rate_index);
    static float RatioMultiplier(int ratio_index);   // period = clock period * this

    void Init(float sample_rate, uint32_t seed);
    void ApplyParams(const Params& p, const Clock& clk, int block_size);
    void ProcessBlock(const float* in_l, const float* in_r,
                      float* out_l, float* out_r, int n);

    // Observers for LEDs / telemetry.
    float position()               const { return panner_.smoothed_position(); }
    bool  new_target()             const { return gen_.new_target(); }
    bool  curve_edit()             const { return curve_edit_; }
    float curve()                  const { return gen_.curve(); }
    bool  internal_source()        const { return internal_source_; }
    bool  internal_source_active() const { return mode_ == Mode::CV && internal_source_; }
    float period_samples()         const { return period_; }
    bool  clocked()                const { return clocked_; }
    int   rate_index()             const { return rate_index_; }
    int   ratio_index()            const { return ratio_index_; }
    float center()                 const { return center_; }
    float depth()                  const { return depth_; }
    float wander()                 const { return gen_.value(); }
    float cv_volts()               const { return cv_volts_; }

    BezierRandom& generator() { return gen_; }
    Panner&       panner()    { return panner_; }

  private:
    BezierRandom gen_;
    Panner       panner_;
    float        sample_rate_;
    Mode         mode_;
    int          rate_index_;
    int          ratio_index_;
    int          curve_steps_;
    bool         curve_edit_;
    bool         internal_source_;
    int          div_counter_;      // edges seen since the last ÷N sync
    float        period_;
    bool         clocked_;
    float        center_;
    float        depth_;
    float        cv_volts_;
};

}  // namespace legio
```

- [ ] **Step 5: Write `src/drifter_chain.cpp`**

```cpp
// src/drifter_chain.cpp
#include "drifter_chain.h"
#include <cmath>
#include "cv_in.h"
#include "edge.h"

namespace legio {

namespace {
constexpr float kFreePeriodMaxSec   = 300.0f;   // 5 minutes per target
constexpr float kFreePeriodMinSec   = 0.05f;    // 20 Hz
constexpr int   kDefaultRateIndex   = 15;       // ~9.7 s
constexpr int   kDefaultRatioIndex  = 3;        // ×1
constexpr int   kDefaultCurveSteps  = 6;        // curve = +0.5, eased
// Index:            ÷8    ÷4    ÷2    ×1    ×2    ×4     ×8
const float kRatioMult[DrifterChain::kRatioCount] = {8.0f, 4.0f, 2.0f, 1.0f, 0.5f, 0.25f, 0.125f};
const int   kRatioDiv [DrifterChain::kRatioCount] = {8,    4,    2,    1,    1,    1,     1};

inline int ClampInt(int x, int lo, int hi) { return x < lo ? lo : (x > hi ? hi : x); }
}  // namespace

float DrifterChain::FreePeriodSeconds(int rate_index) {
    float t = (float)ClampInt(rate_index, 0, kRateSteps) / (float)kRateSteps;
    return kFreePeriodMaxSec * std::pow(kFreePeriodMinSec / kFreePeriodMaxSec, t);
}

float DrifterChain::RatioMultiplier(int ratio_index) {
    return kRatioMult[ClampInt(ratio_index, 0, kRatioCount - 1)];
}

void DrifterChain::Init(float sample_rate, uint32_t seed) {
    sample_rate_     = sample_rate;
    gen_.Init(sample_rate, seed);
    panner_.Init(sample_rate);
    mode_            = Mode::PAN;
    rate_index_      = kDefaultRateIndex;
    ratio_index_     = kDefaultRatioIndex;
    curve_steps_     = kDefaultCurveSteps;
    curve_edit_      = false;
    internal_source_ = false;
    div_counter_     = 0;
    clocked_         = false;
    period_          = FreePeriodSeconds(rate_index_) * sample_rate_;
    center_          = 0.5f;
    depth_           = 0.0f;
    cv_volts_        = 0.0f;
    gen_.SetPeriodSamples(period_);
    gen_.SetCurve((float)curve_steps_ / (float)kCurveSteps);
}

void DrifterChain::ApplyParams(const Params& p, const Clock& clk, int block_size) {
    mode_    = p.mode;
    clocked_ = clk.is_external();

    // --- Encoder events. Tap toggles edit mode; long press toggles the internal
    //     source in CV mode only; rotation edits exactly one of rate / ratio / curve.
    if (p.encoder_tap) curve_edit_ = !curve_edit_;
    if (p.encoder_long_press && mode_ == Mode::CV) internal_source_ = !internal_source_;
    if (p.encoder_increment != 0) {
        if (curve_edit_) {
            curve_steps_ = ClampInt(curve_steps_ + p.encoder_increment, -kCurveSteps, kCurveSteps);
        } else if (clocked_) {
            ratio_index_ = ClampInt(ratio_index_ + p.encoder_increment, 0, kRatioCount - 1);
        } else {
            rate_index_ = ClampInt(rate_index_ + p.encoder_increment, 0, kRateSteps);
        }
    }

    // --- Period: clocked = measured clock period × ratio; free = encoder log scale.
    period_ = clocked_ ? clk.period_samples() * kRatioMult[ratio_index_]
                       : FreePeriodSeconds(rate_index_) * sample_rate_;
    gen_.SetPeriodSamples(period_);
    gen_.SetCurve((float)curve_steps_ / (float)kCurveSteps);

    // --- Clock sync: every edge for ×N, every Nth edge for ÷N.
    if (clocked_ && clk.tick()) {
        if (++div_counter_ >= kRatioDiv[ratio_index_]) {
            div_counter_ = 0;
            gen_.Sync();
        }
    }
    if (!clocked_) div_counter_ = 0;

    // --- Position for this block.
    center_ = p.top_knob;
    depth_  = p.bottom_knob;
    float r       = gen_.Process(block_size);
    float pos_raw = center_ + 0.5f * depth_ * r;
    panner_.SetPositionTarget(apply_edge(p.edge, pos_raw));

    // --- CV source and mode.
    cv_volts_ = cv_volts_from_norm(p.cv_norm);
    panner_.SetCvSource(cv_source_from_volts(cv_volts_));
    panner_.SetInternalSource(internal_source_);
    panner_.SetMode(mode_);
}

void DrifterChain::ProcessBlock(const float* in_l, const float* in_r,
                                float* out_l, float* out_r, int n) {
    panner_.ProcessBlock(in_l, in_r, out_l, out_r, n);
}

}  // namespace legio
```

- [ ] **Step 6: Add `src/drifter_chain.cpp` to the firmware Makefile**

`CPP_SOURCES = src/main.cpp src/clock.cpp src/bezier_random.cpp src/panner.cpp src/drifter_chain.cpp`

- [ ] **Step 7: Run the tests and the firmware build**

```bash
make -C test && make
```

Expected: `test_chain` shows 14 `PASS` lines, `All tests passed.`; firmware links. If `test_clock_ratio_divides_and_multiplies` disagrees on the sync count, check that `div_counter_` counts edges *seen while clocked*, and remember the `Clock` marks itself external only after the second valid interval.

- [ ] **Step 8: Commit**

```bash
git add src/drifter_chain.h src/drifter_chain.cpp test/test_chain.cpp test/Makefile Makefile
git commit -m "feat(chain): rate / ratio / curve-edit / internal-source state and the position pipeline"
```

---

## Task 10: HAL — `main.cpp` with encoder timing, LEDs, telemetry, seed, bypass

**Files:**
- Modify: `src/main.cpp` (replace the Task 1 placeholder entirely)

**Interfaces:**
- Consumes: `DrifterChain` (`Init`, `ApplyParams`, `ProcessBlock`, all observers), `Clock` (`Init`, `update`, `period_samples`, `is_external`), `Params`, `Mode`, `Edge`, `kSampleRate`, `cv_volts_from_norm` for telemetry.
- Produces: the firmware. Switch inversion, encoder tap / long-press edges, the `volatile UiSnapshot`, LED rules from the spec, the 5 Hz telemetry line, the encoder-held-at-boot bypass, and the ADC-seeded RNG.

Facts the implementer needs:

- `hw.encoder.TimeHeldMs()` returns 0 once the button is released, so tap detection must record `System::GetNow()` at the rising edge in `main.cpp` and compute the hold time itself on the falling edge.
- `hw.encoder.Increment()` is −1 / 0 / +1 per `ProcessAllControls()` call. Rotation while pressed is discarded (spec).
- `hw.controls[i].GetRawValue()` returns the 16-bit ADC word; the low byte is the noise source for the seed.
- `hw.sw[DaisyLegio::SW_LEFT].Read()`: `2` = panel UP, `0` = center, `1` = panel DOWN.
- `hw.SetLed(idx, r, g, b)` + `hw.UpdateLeds()` in the slow loop only. Yellow = `(y, y, 0)`, blue = `(0, 0, b)`.

- [ ] **Step 1: Replace `src/main.cpp`**

```cpp
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
```

- [ ] **Step 2: Build the firmware**

```bash
make
```

Expected: `build/drifter.bin`, no warnings about unused variables (fix any), memory usage printed. Note the flash size; it should be well under 128 KB.

- [ ] **Step 3: Run the full host suite one more time**

```bash
make -C test
```

Expected: every test binary prints `All tests passed.`

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "feat(hal): main.cpp with encoder tap/long-press timing, LEDs, telemetry, ADC seed, bypass"
```

---

## Task 11: Hardware gates 1–4 (audio, wander, clock)

**Files:**
- Modify: `CLAUDE.md` (record observations under "Behavior notes" / "Open / known issues")

This task needs the module in the rack, a cable or two, and serial open. Each gate is pass/fail; record failures rather than "fixing" by feel.

- [ ] **Step 1: Flash**

Put the Patch SM in DFU (BOOT + RESET on the back), then:

```bash
make program-dfu
```

Expected: dfu-util reports the download; `Error 74` / "Error during download get_status" is harmless. If `/dev/cu.usbmodem*` does not reappear, reseat the module before assuming a crash.

- [ ] **Step 2: Open telemetry**

```bash
screen /dev/cu.usbmodem* 115200
```

Expected: a line every 200 ms of the form `mode=... pos=... ctr=... dep=... T=... ext=0 curve=+0.50 edge=... cv_norm=... cv=... src=jack`.

- [ ] **Step 3: Gate 1 — bypass, then PAN with In L only**

1. Reset with the encoder held. Both LEDs dim white, serial prints `BYPASS MODE`, audio passes both channels. Release, reset.
2. Left switch panel-up (PAN). Confirm `mode=PAN` in telemetry; if it says `CV`, the switch inversion is wrong — fix in `main.cpp` and re-flash, do not touch the chain.
3. Patch one source into In L only, depth (bottom knob) fully CCW. Sweep the top knob: the image moves left ↔ right and **both** outputs are live (hardware normalling). Telemetry `pos` tracks `ctr`.
4. Right switch: check `edge=` reads CLIP up / FOLD center / WRAP down.

- [ ] **Step 4: Gate 2 — XFADE with two sources**

Left switch center. Patch two different sources into In L and In R. Out L crossfades A → B as the top knob goes CCW → CW; Out R does the opposite. At noon both outputs carry an equal mix.

- [ ] **Step 5: Gate 3 — wander and curve**

1. Depth up. Turn the encoder until telemetry shows `T=10.0s` ± 1 (the boot default is ≈ 9.7 s, so at most a detent or two).
2. Watch the blue LEDs drift and the **left** LED flash yellow at each new target (once per `T`).
3. Tap the encoder: LEDs go yellow, telemetry appends `[curve edit]`. Rotate CCW to `curve=-1.00`, tap out, listen: motion cusps at each target. Tap in, rotate to `+1.00`, tap out: motion plateaus. Return to `+0.50`.
4. Confirm rotation while the encoder is held does nothing, and a slow ~600 ms press neither taps nor toggles anything.
5. Try CLIP / FOLD / WRAP with center off-noon and depth full: CLIP parks, FOLD bounces, WRAP snaps with a short sweep.

- [ ] **Step 6: Gate 4 — clock**

1. Patch a slow clock (~1 Hz) into the gate jack. Within two edges `ext=1` and `T=` reads the clock period. The **right** LED flashes yellow on each edge, and the left one flashes with it (each edge lands a target at ×1).
2. Rotate the encoder: `T=` steps through ÷8 … ×8 multiples of the clock. At ÷2 the left flash appears every second edge. At ×4 the left flash appears four times per beat.
3. Unpatch the clock. Free-run resumes (`ext=0`) after the greater of 4 clock periods or 2 s, and `T=` returns to the previous free-running value.

- [ ] **Step 7: Record**

In `CLAUDE.md` under "Behavior notes", add one line per gate with the result and any surprise (e.g. actual default `T`, whether the 50 ms flashes are visible enough, encoder feel). Anything that failed goes under "Open / known issues" with what was observed. Then:

```bash
git add CLAUDE.md
git commit -m "docs: hardware gates 1-4 results"
```

---

## Task 12: Hardware gates 5–6 — CV-mode calibration and generative CV

**Files:**
- Modify: `src/cv_in.h` (only if a measurement disagrees with the reused constants)
- Modify: `CLAUDE.md` ("CV-mode calibration" section, the auto-select decision)
- Modify: `src/main.cpp` and `src/drifter_chain.cpp` only if the auto-select upgrade is taken (see Step 5)

**Interfaces:**
- Consumes: telemetry fields `cv_norm` and `cv`; `kCvZero`, `kCvScale`, `kCvDeadband`.
- Produces: verified constants, the four measurements the spec asks for, and a recorded decision on automatic internal-source selection.

- [ ] **Step 1: Unpatched jack measurement**

Left switch panel-down (CV). Nothing in the v/oct jack. Watch `cv_norm` for ~10 s and note its mean and its min–max spread. Also note whether `cv=` reads `0.000V` (deadband covering it) or not. With a meter on Out L and Out R, both must read 0 V.

Write into `CLAUDE.md` → "CV-mode calibration":
`Unpatched v/oct jack: cv_norm = <mean> (spread <min>–<max> over 10 s); deadband covers it: yes/no.`

- [ ] **Step 2: 0 V patched**

Patch a cable from a quiet 0 V output (an unmodulated offset, or a DC source at 0). Note `cv_norm`. Expect ≈ 0.3019. If it differs by more than 0.005, edit `kCvZero` in `src/cv_in.h` to the new value.

- [ ] **Step 3: +1 V (or +5 V) patched**

Patch a known voltage. `cv=` should read it within 1 %. If not: `kCvScale = ΔV / (cv_norm_at_V − kCvZero)`; edit `src/cv_in.h`.

- [ ] **Step 4: If any constant changed: rebuild, re-run tests, re-flash, repeat Steps 1–3**

```bash
make -C test && make && make program-dfu
```

Expected: `test_cv_in` still passes (its tests are relative to the constants). Record the final values in `CLAUDE.md`.

- [ ] **Step 5: Gate 5 — CV panning**

Envelope into the v/oct jack, Out L and Out R into two filter cutoffs (or two VCA CVs). Top knob shares the envelope between them; depth adds wander; the two voices trade presence. With the jack unpatched again both outputs return to 0 V on the meter.

- [ ] **Step 6: Gate 6 — generative CV**

Long-press the encoder in CV mode. Telemetry shows `src=int`, both LEDs gain a dim yellow floor under the blue. On a meter Out L + Out R ≈ 5 V at every moment while the two drift complementarily; depth, center, edge switch, curve and clock all shape them. Long-press again: `src=jack`, floor gone. Switch to PAN and back to CV: the setting is remembered.

- [ ] **Step 7: Decide on automatic internal-source selection**

From the Step 1 numbers:

- If the unpatched `cv_norm` sits inside the range a patched cable can produce (≈ 0.30 ± the deadband, low spread), automatic detection is **not possible**. Record `Automatic internal-source selection: not possible (unpatched reads like 0 V); long press stays.` in `CLAUDE.md`. Done.
- If the unpatched reading has a signature a driven 0 V never shows (a value outside the ±5 V span, or a spread more than ~10× the patched spread), record the signature and implement the upgrade as a follow-up **outside this plan**: a detector in `main.cpp` (signature present for > 1 s → `internal` true; a real voltage → false within one block) that feeds `Params`, and `DrifterChain::ApplyParams` consuming that bool instead of the long-press toggle. The DSP already takes a single `internal_source_` flag, so only where it comes from changes. Record the decision and the measured signature in `CLAUDE.md` under "Open / known issues".

- [ ] **Step 8: Commit**

```bash
git add CLAUDE.md src/cv_in.h
git commit -m "docs: CV-mode calibration measurements and internal-source selection decision"
```

Then confirm the working tree is clean and the host suite is green one last time:

```bash
git status --short
make -C test
```

Expected: nothing to commit; `All tests passed.` from every binary. Only now is the app "done".

---

## Self-review

**Spec coverage:**

- Goal / "usable with no manual" → the control mapping in Task 10 and the README in Task 1.
- Hardware reality (AC-coupled inputs, DC outputs, normalling, 3 ADCs, switch polarity, no jack detection, colorblind LEDs) → Task 1 CLAUDE.md, Task 10 switch inversion and LED rules, Task 7 normalling test.
- Modes PAN / XFADE / CV with their exact output equations → Tasks 7–8 (`Panner::ProcessBlock`) with tests for each equation; mode from the left switch → Task 10.
- Equal-power law, linear CV law → Tasks 7–8.
- Position signal (`center`, `depth`, `pos_raw`, edge, smoother, per-sample gains) → Task 9 `ApplyParams` + Task 7 smoother; tests `depth 0 → pos == center`, no zipper, fold in [0,1].
- Edge behavior CLIP / FOLD / WRAP applied to the summed position → Task 2 + Task 9 (applied after summing knob/CV/wander).
- Bézier generator: uniform targets, phase timing with immediate period change, both curve families, no overshoot, Newton solve, `Sync`, xorshift with fixed seed → Tasks 5–6 with one test per claim.
- Rate: free-running log scale 5 min → 20 Hz; clocked ratios ÷8 … ×8 with ×N syncing every edge and ÷N every Nth; both values kept in RAM → Task 9 (`FreePeriodSeconds`, `kRatioMult`, `kRatioDiv`, `test_free_rate_restored_after_clock_falls_back`).
- Clock detection and fallback max(4 periods, 2 s) → Task 4.
- Random seed from ADC low bits at boot → Task 10 `SeedFromAdcNoise`.
- Controls table (knobs, encoder rotate / tap / long press / boot-hold, switches, gate, v/oct) → Tasks 9 and 10.
- LEDs (blue position, yellow flashes, curve-edit yellow display, internal-source floor, bypass white) → Task 10 slow loop.
- CV-mode calibration constants, deadband as hard gate, `kCvScale = 0` disables → Task 3; procedure → Task 12.
- Generative CV (internal +5 V, long press in CV only, remembered across modes, LED floor, auto-select decision) → Tasks 8, 9, 10, 12.
- Architecture (file list, DSP/HAL split, no DaisySP, data flow, telemetry line) → Task 1 skeleton, Task 9, Task 10 (telemetry format matches the spec's line, with `[curve edit]` appended as an extra).
- Testing section: every listed host test has a concrete test function in Tasks 2–9; hardware gates 1–6 → Tasks 11–12.
- Non-goals: nothing in the plan implements rate CV, a second channel, freeze, persistence, or a plugin.

Two small deviations, both stated in their tasks: the clock's maximum accepted period is 20 s rather than the stutterer's 5 s (Task 4), and boot defaults for rate / ratio / curve are fixed in Task 9 because the spec leaves them open. Also, the spec says "for `curve = 0` the solve is skipped (`s = φ`)" and implies only the CW family needs the solve; the math in Task 6 shows the CCW family's `x(s)` is not the identity either, so the solve runs for every non-zero curve and the `curve = 0` shortcut evaluates the line directly. The observable behavior is exactly what the spec describes.

**Placeholder scan:** no TBD / TODO / "handle edge cases" / "similar to Task N". The one intentional stub (`Mode::CV` outputs zeros in Task 7, `ControlPoints` linear in Task 5) is replaced by the immediately following task and is labeled as such. Task 12 Step 7 explicitly defers the auto-select upgrade outside this plan rather than leaving a hole.

**Type consistency:**
- `Mode { PAN, XFADE, CV }`, `Edge { CLIP, FOLD, WRAP }` — identical in `params.h`, `edge.h`, `panner`, `chain`, tests, `main.cpp` (cast to `int` for the snapshot).
- `Params` fields `top_knob`, `bottom_knob`, `cv_norm`, `mode`, `edge`, `encoder_increment`, `encoder_tap`, `encoder_long_press`, `gate_edge` — used with these exact names in Tasks 9 and 10.
- `Clock::Init / update / period_samples / tick / is_external` — Task 4 definition matches Task 9 and 10 use.
- `BezierRandom::Init(sr, seed) / SetPeriodSamples / SetCurve / SetEndpoints / Sync / Process(block) / new_target / value / phase / current / target / curve / solve_error` — Task 5 header matches Task 6 and Task 9 use.
- `Panner::Init / SetMode / SetInternalSource / SetPositionTarget / SetPositionImmediate / SetCvSource / ProcessBlock / smoothed_position / EqualPowerGains` — Task 7 header matches Task 8 and Task 9 use.
- `DrifterChain::Init(sr, seed) / ApplyParams(p, clk, block_size) / ProcessBlock / position / new_target / curve_edit / curve / internal_source / internal_source_active / period_samples / clocked / rate_index / ratio_index / center / depth / wander / cv_volts / FreePeriodSeconds / RatioMultiplier / kRateSteps / kRatioCount / kCurveSteps` — Task 9 header matches its tests and Task 10.
- `cv_volts_from_norm(norm, zero, scale, deadband)`, `cv_source_from_volts(v)`, `apply_edge(Edge, x)` — Tasks 2–3 match Task 9.

No mismatches found.
