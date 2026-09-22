/*
 * Minimal test helpers: no external dependencies, so the suite builds
 * with any C compiler and runs the same locally and in CI.
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>

extern int g_checks;
extern int g_failures;

#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                   \
  } while (0)

#define CHECK_EQ(actual, expected)                                      \
  do {                                                                  \
    long long a_ = (long long)(actual);                                 \
    long long e_ = (long long)(expected);                               \
    g_checks++;                                                         \
    if (a_ != e_) {                                                     \
      g_failures++;                                                     \
      printf("    FAIL %s:%d: %s == %lld, expected %lld\n",             \
             __FILE__, __LINE__, #actual, a_, e_);                      \
    }                                                                   \
  } while (0)

#define RUN_TEST(fn)                                                    \
  do {                                                                  \
    printf("  %s\n", #fn);                                              \
    fn();                                                               \
  } while (0)

void run_can_codec_tests(void);
void run_vehicle_control_tests(void);

#endif /* TEST_FRAMEWORK_H */
