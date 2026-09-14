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
