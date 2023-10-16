#include "BaseFile.h"

// DECLARE_LOGGER("Level2", Logger::LVL_INFO)

static void level2Foo() 
{ 
  TextLogger& logger = TextLog::get("Level2", TextLog::INFO);
  LOG(logger, TextLog::INFO, "this is level2 INFO");
  LOG(logger, TextLog::DEBUG, "this is level2 DEBUG");
}

struct Level2
{
  static void logTest()
  {
    fmt::print("Level2 ----------------------------\n");

    LOG_BASE();

    fmt::print("Level2: {}:{} INC_LEVEL_V1={}, INC_LEVEL_V2={}, __INCLUDE_LEVEL__={}\n", __FILE__, __LINE__, INC_LEVEL_V1, INC_LEVEL_V2, __INCLUDE_LEVEL__);
  }
};