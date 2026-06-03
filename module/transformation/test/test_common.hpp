#ifndef TEST_COMMON_HPP
#define TEST_COMMON_HPP

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <vector>
#include <string>
#include <functional>
#include <iostream>

// ==================== Test Framework Macros ====================

#define COLOR_GREEN  "\033[1;32m"
#define COLOR_RED    "\033[1;31m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_CYAN   "\033[1;36m"
#define COLOR_RESET  "\033[0m"

// Assertion macros
#define ASSERT_TRUE(expr)  do { _assert((expr), #expr, __FILE__, __LINE__); } while(0)
#define ASSERT_FALSE(expr) do { _assert(!(expr), "!" #expr, __FILE__, __LINE__); } while(0)

#define ASSERT_EQ(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a == _b)) { \
        printf(COLOR_RED "  X ASSERT_EQ failed: %s == %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  " << _a << std::endl; \
        std::cout << "    right: " << _b << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_NE(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a != _b)) { \
        printf(COLOR_RED "  X ASSERT_NE failed: %s != %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_GT(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a > _b)) { \
        printf(COLOR_RED "  X ASSERT_GT failed: %s > %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  " << _a << std::endl; \
        std::cout << "    right: " << _b << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_GE(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a >= _b)) { \
        printf(COLOR_RED "  X ASSERT_GE failed: %s >= %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  " << _a << std::endl; \
        std::cout << "    right: " << _b << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_LE(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a <= _b)) { \
        printf(COLOR_RED "  X ASSERT_LE failed: %s <= %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  " << _a << std::endl; \
        std::cout << "    right: " << _b << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_LT(a, b)    do { \
    auto _a = (a); auto _b = (b); \
    if (!(_a < _b)) { \
        printf(COLOR_RED "  X ASSERT_LT failed: %s < %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  " << _a << std::endl; \
        std::cout << "    right: " << _b << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_DOUBLE_EQ(a, b, eps) do { \
    double _a = (a); double _b = (b); \
    double diff = (_a > _b) ? (_a - _b) : (_b - _a); \
    if (diff > (eps)) { \
        printf(COLOR_RED "  X ASSERT_DOUBLE_EQ failed: |%s - %s| <= %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, #eps, __FILE__, __LINE__); \
        printf("    %s = %.10f, %s = %.10f, diff = %.10f\n", #a, _a, #b, _b, diff); \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

#define ASSERT_STREQ(a, b) do { \
    std::string _a = (a); std::string _b = (b); \
    if (_a != _b) { \
        printf(COLOR_RED "  X ASSERT_STREQ failed: %s == %s  (%s:%d)\n" COLOR_RESET, \
               #a, #b, __FILE__, __LINE__); \
        std::cout << "    left:  \"" << _a << "\"" << std::endl; \
        std::cout << "    right: \"" << _b << "\"" << std::endl; \
        g_assertions_failed++; \
    } else { g_assertions_passed++; } \
} while(0)

// Global test counters (defined in test_main.cpp)
extern int g_tests_passed;
extern int g_tests_failed;
extern int g_assertions_passed;
extern int g_assertions_failed;

// Test registration
extern std::vector<std::pair<const char*, std::function<bool()>>> g_all_tests;

inline void register_test(const char* name, std::function<bool()> func) {
    g_all_tests.push_back({name, func});
}

// Helper: inline assertion function
inline void _assert(bool condition, const char* expr, const char* file, int line) {
    if (!condition) {
        printf(COLOR_RED "  X ASSERT_TRUE failed: %s  (%s:%d)\n" COLOR_RESET, expr, file, line);
        g_assertions_failed++;
    } else {
        g_assertions_passed++;
    }
}

// Suite header macro
#define TEST_SUITE(name) \
    printf(COLOR_CYAN "\n========================================\n" COLOR_RESET); \
    printf(COLOR_CYAN "  Test Suite: %s\n" COLOR_RESET, name); \
    printf(COLOR_CYAN "========================================\n" COLOR_RESET);

#endif // TEST_COMMON_HPP
