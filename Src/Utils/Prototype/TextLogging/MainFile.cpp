#include "Level2File.h"

static TextLogger& logger = TextLog::get("Main", TextLog::DEBUG);

int main(int argc, char* argv[])
{
  Level2::logTest();

  LOG_BASE();

  fmt::print("Main: {}:{} INC_LEVEL_BASE={}\n", __FILE__, __LINE__, INC_LEVEL_BASE);
  fmt::print("Main: {}:{} INC_LEVEL_V1={}, INC_LEVEL_V2={}, __INCLUDE_LEVEL__={}\n", __BASE_FILE__, __LINE__, INC_LEVEL_V1, INC_LEVEL_V2, __INCLUDE_LEVEL__);


  baseFoo();
  level2Foo();

  LOG(logger, TextLog::INFO, "this is main INFO");
  LOG(logger, TextLog::DEBUG, "this is main DEBUG");
  LOG(logger, TextLog::VERBOSE, "this is main VERBOSE");
}