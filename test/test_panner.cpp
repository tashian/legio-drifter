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

static void run_all() {
    RUN_TEST(test_equal_power_center_is_minus_3db);
    RUN_TEST(test_pan_hard_left_and_right);
    RUN_TEST(test_pan_each_output_carries_only_its_own_input);
    RUN_TEST(test_pan_with_identical_inputs_is_a_mono_panner);
    RUN_TEST(test_xfade_out_r_is_the_complement_of_out_l);
    RUN_TEST(test_position_smoother_never_zippers);
    RUN_TEST(test_smoother_time_constant_is_about_5ms);
    RUN_TEST(test_cv_linear_law_sums_to_source);
    RUN_TEST(test_cv_mode_ignores_audio_inputs);
    RUN_TEST(test_internal_source_sums_to_five_volts_and_ignores_jack);
    RUN_TEST(test_internal_source_only_matters_in_cv_mode);
    RUN_TEST(test_cv_source_is_smoothed_not_stepped);
}

TEST_MAIN()
