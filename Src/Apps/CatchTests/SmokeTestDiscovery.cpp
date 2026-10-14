/**
 * @file SmokeTestDiscovery.cpp
 *
 * Minimal test to ensure that tests are auto-discovered across different source files
 *
 * @author Rudi Villing
 */

 #include <catch2/catch_test_macros.hpp>

TEST_CASE("Automatic discovery test", "[smoke_tests][discovery]") {
    SECTION("Verify that section in test in multiple files are discovered correctly") {
        bool catch2_is_working = true;
        REQUIRE(catch2_is_working);
    }
}