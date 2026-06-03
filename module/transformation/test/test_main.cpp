/**
 * @file test_main.cpp
 * @brief Simple test framework - main runner for UUV formation simulation tests
 *
 * Test coverage:
 *   - JSON serialization round-trip consistency
 *   - Binary compression ratio verification
 *   - Large file (1000+ frames) read/write
 *   - Simulation export and read-back verification
 *   - Data comparison validation
 */

#include "test_common.hpp"

// ==================== Global test state (definitions) ====================

int g_tests_passed = 0;
int g_tests_failed = 0;
int g_assertions_passed = 0;
int g_assertions_failed = 0;

// Global test registry
std::vector<std::pair<const char*, std::function<bool()>>> g_all_tests;

// ==================== Forward declarations ====================
void register_serializer_tests();
void register_integration_tests();

// ==================== Test runner ====================

static void run_test(const char* name, std::function<bool()> test_func) {
    printf(COLOR_YELLOW "\n[TEST] %s\n" COLOR_RESET, name);

    int before_assertions = g_assertions_passed + g_assertions_failed;

    bool result = false;
    try {
        result = test_func();
    } catch (const std::exception& e) {
        printf(COLOR_RED "  X EXCEPTION: %s\n" COLOR_RESET, e.what());
        result = false;
    } catch (...) {
        printf(COLOR_RED "  X UNKNOWN EXCEPTION\n" COLOR_RESET);
        result = false;
    }

    int test_assertions = (g_assertions_passed + g_assertions_failed) - before_assertions;

    if (result) {
        g_tests_passed++;
        printf(COLOR_GREEN "  PASSED" COLOR_RESET);
        if (test_assertions > 0) {
            printf(" (%d assertions)", test_assertions);
        }
        printf("\n");
    } else {
        g_tests_failed++;
        printf(COLOR_RED "  FAILED" COLOR_RESET);
        if (test_assertions > 0) {
            printf(" (%d assertions)", test_assertions);
        }
        printf("\n");
    }
}

// ==================== Main ====================

int main(int argc, char* argv[]) {
    printf(COLOR_CYAN "============================================\n" COLOR_RESET);
    printf(COLOR_CYAN "  UUV Formation Simulation - Test Suite v1.0\n" COLOR_RESET);
    printf(COLOR_CYAN "  Transformation Module Tests\n" COLOR_RESET);
    printf(COLOR_CYAN "============================================\n" COLOR_RESET);

    // Register all tests
    register_serializer_tests();
    register_integration_tests();

    // Run all tests
    auto start_time = std::chrono::high_resolution_clock::now();

    for (const auto& test : g_all_tests) {
        run_test(test.first, test.second);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    // Print summary
    printf(COLOR_CYAN "\n============================================\n" COLOR_RESET);
    printf(COLOR_CYAN "  Test Results Summary\n" COLOR_RESET);
    printf(COLOR_CYAN "--------------------------------------------\n" COLOR_RESET);
    printf("  Total test cases:  %-3d\n", g_tests_passed + g_tests_failed);
    printf(COLOR_GREEN "  Passed: %-3d\n" COLOR_RESET, g_tests_passed);
    printf(COLOR_RED "  Failed: %-3d\n" COLOR_RESET, g_tests_failed);
    printf("  Total assertions:  %-3d\n", g_assertions_passed + g_assertions_failed);
    printf("  Time: %lld ms\n", duration_ms);
    printf(COLOR_CYAN "============================================\n" COLOR_RESET);

    if (g_tests_failed > 0) {
        printf(COLOR_RED "\n*** Some tests FAILED! ***\n" COLOR_RESET);
        return 1;
    }

    printf(COLOR_GREEN "\n*** All tests PASSED! ***\n" COLOR_RESET);
    return 0;
}
