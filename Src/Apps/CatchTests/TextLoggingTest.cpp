/**
 * @file TextLoggingTest.cpp
 *
 * Basic Unit testing of the TextLogging framework
 * 
 * Since TextLogging normally writes to stdout, for these tests we redirect
 * output to a file and then read back the file after some logging to check
 * that the output is as we expect.
 *
 * @author Rudi Villing
 */

#include "TextLogging/TextLogging.h"
#include "TextLogging/TextLoggingStream.h" // Essential for TLOG_STREAM and operator<< support

#include "catch2/catch_test_macros.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

// RAII helper to manage redirecting the log file. This ensures that the
// textlogging is always set back to stdout in case of an unplanned termination
// of a test (e.g. due to an exception)

struct LogTestContext
{
  std::string path = "test_text_log.txt";
  FILE *file;

  LogTestContext()
  {
    if (std::filesystem::exists(path))
      std::filesystem::remove(path);
    file = std::fopen(path.c_str(), "w+");
    TextLogging::setOutputFile(file);
  }

  ~LogTestContext()
  {
    TextLogging::setOutputFile(stdout); // Reset to default
    if (file)
      std::fclose(file);
    // if (std::filesystem::exists(path))
    //   std::filesystem::remove(path);
  }
};


// Helper to read the log file into a string for verification
std::string readLogFile(const std::string &path)
{
  std::fflush(nullptr); // Ensure C-style buffers are flushed to disk
  std::ifstream in(path);
  if (!in.is_open())
    return "";
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}




// 1. Custom type for testing Point 3 (Custom type logging)
struct CustomRobotPoint
{
  int x, y;
  friend std::ostream &operator<<(std::ostream &os, const CustomRobotPoint &p)
  {
    return os << "Point(" << p.x << "," << p.y << ")";
  }
};





TEST_CASE("TextLogging Functional Verification", "[TextLogging]")
{
  LogTestContext ctx;

  DECL_TLOGGER(tlogger, "UnitTest", TextLogging::INFO);

  SECTION("Fmt-style syntax (Sync)")
  {
    TextLogging::requestAsyncOutput(false);
    TLOGI(tlogger, "Hello {}, score is {}", "Booster", 7);

    std::string output = readLogFile(ctx.path);

    INFO("Full log text was: \n" << output);
    // Verify format: Severity/ [ThreadName] LoggerName : Message
    CHECK(output.find("I/ [CatchTests] UnitTest : Hello Booster, score is 7") != std::string::npos);
  }

  SECTION("Stream-style syntax and Custom Types (Sync)")
  {
    TextLogging::requestAsyncOutput(false);
    CustomRobotPoint pos{10, -5};

    TLOGI_STREAM(tlogger, "Robot position: " << pos << " at time " << 100);

    std::string output = readLogFile(ctx.path);
    CHECK(output.find("I/ [CatchTests] UnitTest : Robot position: Point(10,-5) at time 100") != std::string::npos);
  }

  SECTION("Severity Thresholding")
  {
    TextLogging::requestAsyncOutput(false);

    TLOGD(tlogger, "Debug should be hidden"); // Logger level is INFO
    TLOGW(tlogger, "Warning should be visible");

    std::string output = readLogFile(ctx.path);
    CHECK(output.find("Debug should be hidden") == std::string::npos);
    CHECK(output.find("Warning should be visible") != std::string::npos);
  }

  SECTION("Async Mode Dispatch")
  {
    TextLogging::requestAsyncOutput(false);
    TLOGI(tlogger, "Initially starting in sync mode");
    TLOGI(tlogger, "will now switch to async mode");

    // Switch to async
    TextLogging::requestAsyncOutput(true);

    TLOGI(tlogger, "Message Alpha");
    TLOGI_STREAM(tlogger, "Message Beta");

    // requestAsyncOutput(false) triggers a thread stop/join
    // which ensures all queued messages are processed before returning.
    TextLogging::requestAsyncOutput(false);

    TLOGI(tlogger, "confirming back in sync mode again");

    std::string output = readLogFile(ctx.path);

    INFO("Text log output was:\n" << output);

    CHECK(output.find("Message Alpha") != std::string::npos);
    CHECK(output.find("Message Beta") != std::string::npos);
    CHECK(output.find("Completed switch back to synchronous text logging") != std::string::npos);
  }

  SECTION("Stress testing Async Mode Dispatch")
  {
    TextLogging::requestAsyncOutput(false);
    TLOGI(tlogger, "Initially starting in sync mode");
    TLOGI(tlogger, "will now switch to async mode");

    // Switch to async
    TextLogging::requestAsyncOutput(true);

    for (int i=0; i<1500; i++)
      TLOGI(tlogger, "This is just a filler message number {}", i);

    // requestAsyncOutput(false) triggers a thread stop/join
    // which ensures all queued messages are processed before returning.
    TextLogging::requestAsyncOutput(false);

    TLOGI(tlogger, "confirming back in sync mode again");

    std::string output = readLogFile(ctx.path);

    CHECK(output.find("This is just a filler message number 1000") != std::string::npos);
    CHECK(output.find("Completed switch back to synchronous text logging") != std::string::npos);
  }
}