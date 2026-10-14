/**
 * @file TextLoggingBenchmarks.cpp
 *
 * Benchmarking of different TextLogging approaches
 * 
 * To run these tests:
 *
 * - Benchmark A: Background CPU Cost (Ignoring I/O) by executing:
 *   Build/.../CatchTests "[performance]" --benchmark-samples 100 -o /dev/stderr > /dev/null
 * 
 * - Benchmark B: Live Console Output:
 *   Build/.../CatchTests "[performance]" --benchmark-samples 100
 *
 * - Benchmark C: Pipe and File Redirect (based on the Launch Script):
 *   Build/.../CatchTests "[performance]" --benchmark-samples 50 | tee /home/nao/log/bench_results.log
 *
 * @author Rudi Villing
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <fmt/compile.h>
#include <cstdio>

TEST_CASE("TextLogging buffer benchmark", "[.][performance][!benchmark]") {
    const int timeMs = 123456;
    const char* lvl = "INFO";
    const char* tName = "Motion";
    const int team = 1;
    const int player = 3;
    const char* lName = "WalkingEngine";
    const std::string msg = "Cycle time > 10ms";

    static constexpr auto compiled = FMT_COMPILE("{}.{:03d} {}/ [{}] t{}-r{} {} : ");

    // Modeling the thread_local buffer reuse strategy
    fmt::memory_buffer sharedBuf;
    sharedBuf.reserve(1024); // Warm it up

    // --- SCENARIO A: IONBF (UNBUFFERED - Current Robot Setup) ---
    SECTION("Unbuffered I/O (IONBF)") {
        std::setvbuf(stdout, nullptr, _IONBF, 0);

        BENCHMARK("Old Setup (IONBF, Runtime fmt + Stack Re-allocation mem buf)")
        {
          fmt::memory_buffer buf;
          fmt::format_to(std::back_inserter(buf), "{}.{:03d} {}/ [{}] t{}-r{} {} : ", (timeMs / 1000), (timeMs % 1000),
                         lvl, tName, team, player, lName);
          buf.append(msg.data(), msg.data() + msg.size());
          buf.push_back('\n');
          return std::fwrite(buf.data(), 1, buf.size(), stdout);
        };

        BENCHMARK("New Setup (IONBF + FMT_COMPILE + sharedBuf reuse)")
        {
          sharedBuf.clear();
          fmt::format_to(std::back_inserter(sharedBuf), compiled, (timeMs / 1000), (timeMs % 1000), lvl, tName, team, player, lName);
          sharedBuf.append(msg.data(), msg.data() + msg.size());
          sharedBuf.push_back('\n');

          // Single system call
          return std::fwrite(sharedBuf.data(), 1, sharedBuf.size(), stdout );
        };
    }

    // --- SCENARIO B: IOLBF (LINE BUFFERED - Proposed for Speed) ---
    SECTION("Line Buffered I/O (IOLBF)") {
        std::setvbuf(stdout, nullptr, _IOLBF, 1024);

        BENCHMARK("Mod setup (IOLBF only)") {
          fmt::memory_buffer buf;
          fmt::format_to(std::back_inserter(buf), "{}.{:03d} {}/ [{}] t{}-r{} {} : ", (timeMs / 1000), (timeMs % 1000),
                         lvl, tName, team, player, lName);
          fmt::format_to(std::back_inserter(buf), " {} \n", msg);
          return std::fwrite(buf.data(), 1, buf.size(), stdout);
        };

        BENCHMARK("New Setup (IOLBF + FMT_COMPILE)")
        {
          fmt::memory_buffer buf;
          fmt::format_to(std::back_inserter(buf), compiled, (timeMs / 1000), (timeMs % 1000), lvl, tName, team, player, lName);
          fmt::format_to(std::back_inserter(buf), " {} \n", msg);
          return std::fwrite(buf.data(), 1, buf.size(), stdout);
        };
    }
}