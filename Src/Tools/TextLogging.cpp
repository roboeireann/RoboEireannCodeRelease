/**
 * @file TextLogging.cpp
 *
 * Implementation file for text logging (see .h file)
 *
 * @author Rudi Villing
 */

#include "TextLogging.h"

#include "Platform/Thread.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

// #include "fmt/compile.h"


TextLogger& TextLogging::get(const std::string& name)
{
    LoggerMap& loggerMap(getLoggerMap());

    // does logger already exist?
    LoggerMap::const_iterator found = loggerMap.find(name);
    if (found == loggerMap.end())
      return registerLogger(name);
    else // existing logger with specified name was found
        return *(found->second);
}


TextLogger& TextLogging::get(const std::string& name, Level level)
{
    TextLogger& logger = get(name);
    logger.enabledLevel = level;
    return logger;
}


TextLogger& TextLogging::registerLogger(const std::string& name)
{
    LoggerMap& loggerMap(getLoggerMap());

    // TODO: there could be a race condition here if multiple
    // threads try to update the loggerMap simultaneously.
    // Currently this doesn't seem to be a problem, but we may need to
    // add synchronisation here

    // loggers are never deleted, so there shouldn't be any memory leaks
    // and hence no need for smart pointers.

    TextLogger* logger = new TextLogger(name);
    loggerMap[name] = logger;

    return *logger;
}


void TextLogging::macrosOnlyOutput_(TextLogger& logger, Level level, const MacrosOnlyTextLogMsg_& msg)
{
  FILE *outFile = stdout;

  fmt::memory_buffer mb;

  unsigned timeMs = Time::getCurrentSystemTime();

#ifndef TARGET_TOOL

  if (Global::settingsExist())
    fmt::format_to(std::back_inserter(mb), 
                    // FMT_COMPILE("{}.{:03d} [{}] : "), 
                    "{}.{:03d} {}/ [{}] t{}-r{} {} : ", 
                    (timeMs / 1000), (timeMs % 1000),
                    levelStrings[level],
                    Thread::getCurrentThreadCachedName(/*fullName*/ true),
                    Global::getSettings().teamNumber,
                    Global::getSettings().playerNumber,
                    logger.getName());
  else
    fmt::format_to(std::back_inserter(mb), 
                    // FMT_COMPILE("{}.{:03d} [{}] : "), 
                    "{}.{:03d} {}/ [{}] {} : ", 
                    (timeMs / 1000), (timeMs % 1000),
                    levelStrings[level],
                    Thread::getCurrentThreadCachedName(/*fullName*/ true),
                    logger.getName());

#else

  fmt::format_to(std::back_inserter(mb), 
                  "{}.{:03d} {}/ {} : ", 
                  (timeMs / 1000), (timeMs % 1000),
                  levelStrings[level],
                  logger.getName());

#endif

  fwrite(mb.data(), sizeof(char), mb.size(), outFile);

  if (msg.tmpFmt.size())
    fwrite(msg.tmpFmt.data(), sizeof(char), msg.tmpFmt.size(), outFile);
  else
  {
    std::string s(msg.tmpStream.str());
    fwrite(s.data(), sizeof(char), s.size(), outFile);
  }

  fputc('\n', outFile);
}
