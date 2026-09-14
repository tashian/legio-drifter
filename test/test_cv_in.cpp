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
