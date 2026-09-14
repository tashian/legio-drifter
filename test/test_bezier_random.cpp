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

static void run_all() {
    RUN_TEST(test_output_stays_in_range_for_a_million_blocks);
    RUN_TEST(test_targets_change_exactly_at_period_boundaries);
    RUN_TEST(test_linear_curve_is_a_straight_line);
    RUN_TEST(test_fixed_seed_is_deterministic);
    RUN_TEST(test_sync_resets_phase_and_draws_a_target);
    RUN_TEST(test_period_change_takes_effect_immediately);
    RUN_TEST(test_ccw_curve_has_cusps);
    RUN_TEST(test_cw_curve_has_plateaus);
    RUN_TEST(test_curves_pass_through_the_same_endpoints);
    RUN_TEST(test_curves_never_overshoot);
    RUN_TEST(test_newton_solve_converges_every_block);
    RUN_TEST(test_curve_is_clamped);
}

TEST_MAIN()
