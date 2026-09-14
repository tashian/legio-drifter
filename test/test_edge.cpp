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
