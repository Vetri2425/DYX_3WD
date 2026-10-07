// Minimal dependency-free test macros: the native build has no ROS and no gtest.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace dyx3_test {
inline int& failures() {
  static int n = 0;
  return n;
}
inline int& checks() {
  static int n = 0;
  return n;
}
}  // namespace dyx3_test

#define CHECK(cond)                                                                 \
  do {                                                                              \
    ++dyx3_test::checks();                                                          \
    if (!(cond)) {                                                                  \
      ++dyx3_test::failures();                                                      \
      std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
    }                                                                               \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                                     \
  do {                                                                                            \
    ++dyx3_test::checks();                                                                        \
    const double a_ = (a);                                                                        \
    const double b_ = (b);                                                                        \
    if (!(std::fabs(a_ - b_) <= (tol))) {                                                         \
      ++dyx3_test::failures();                                                                    \
      std::fprintf(stderr, "%s:%d: CHECK_NEAR failed: %s=%.17g vs %s=%.17g (tol %g)\n", __FILE__, \
                   __LINE__, #a, a_, #b, b_, static_cast<double>(tol));                           \
    }                                                                                             \
  } while (0)

#define TEST_MAIN_RESULT()                                                            \
  (std::printf("%d checks, %d failed\n", dyx3_test::checks(), dyx3_test::failures()), \
   dyx3_test::failures() == 0 ? 0 : 1)
