#pragma once
// Minimal host stand-in for PlatformIO's Unity test macros -- just enough
// to run test_bundle_shared_jacobian.cpp's actual RUN_TEST/TEST_ASSERT_TRUE_MESSAGE
// calls unmodified and get a pass/fail exit code.
#include <cstdio>
#include <cstdlib>
static int g_tests_run = 0, g_tests_failed = 0, g_asserts_failed_this_test = 0;
#define UNITY_BEGIN() do { g_tests_run = 0; g_tests_failed = 0; } while (0)
#define UNITY_END() do { \
    printf("\n==== %d test function(s) run, %d with failures ====\n", g_tests_run, g_tests_failed); \
    if (g_tests_failed) { exit(1); } \
} while (0)
#define RUN_TEST(f) do { \
    g_tests_run++; g_asserts_failed_this_test = 0; \
    printf("--- RUN  %s\n", #f); \
    f(); \
    if (g_asserts_failed_this_test) { g_tests_failed++; printf("--- FAIL %s\n", #f); } \
    else { printf("--- PASS %s\n", #f); } \
} while (0)
#define TEST_ASSERT_TRUE_MESSAGE(cond, msg) do { \
    if (!(cond)) { g_asserts_failed_this_test++; printf("    ASSERT FAILED (%s:%d): %s\n", __FILE__, __LINE__, (msg)); } \
} while (0)
#define TEST_ASSERT_TRUE(cond) TEST_ASSERT_TRUE_MESSAGE(cond, #cond)
#define TEST_MESSAGE(msg) printf("    %s\n", (msg))
static inline void delay(int) {}
