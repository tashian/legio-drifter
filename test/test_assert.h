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
