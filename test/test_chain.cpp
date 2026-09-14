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
