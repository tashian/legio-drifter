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
