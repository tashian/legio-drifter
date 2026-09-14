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

static void test_lone_edge_is_forgotten_after_max_period() {
    Clock c;
    c.Init(48000.0f);
    // One edge, then 21 s of silence (21000 blocks): older than the 20 s max period.
    edge_then_silence(c, 21000);
    c.update(true, 48);                     // this edge must NOT pair with the stale one
    EXPECT_TRUE(!c.is_external());
    // It does become the new first edge: 1 s later a second edge locks a 1 s period.
    for (int i = 0; i < 1000; ++i) c.update(false, 48);
    c.update(true, 48);
    EXPECT_TRUE(c.is_external());
    EXPECT_NEAR(c.period_samples(), 48000.0f, 100.0f);
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
    RUN_TEST(test_lone_edge_is_forgotten_after_max_period);
    RUN_TEST(test_tick_true_only_on_edge_block);
}

TEST_MAIN()
