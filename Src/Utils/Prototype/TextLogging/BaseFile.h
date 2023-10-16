#include "Tools/MacroUtils.h"
#include "fmt/format.h"
#include <unordered_map>

#define INC_LEVEL_V1 __INCLUDE_LEVEL__
#define INC_LEVEL_V2 M_EVAL(__INCLUDE_LEVEL__)

#define INC_LEVEL_BASE M_EVAL(INC_LEVEL_V2)

#define LOG_BASE() fmt::print("Base: {}:{} INC_LEVEL_V1={}, INC_LEVEL_V2={}\n", __FILE__, __LINE__, INC_LEVEL_V1, INC_LEVEL_V2);

// struct Logger;

class TextLogger;

class TextLog
{
public:
  enum Level
  {
    OFF = 0,
    FATAL,
    ERROR,
    WARNING,
    INFO,
    DEBUG,
    VERBOSE
  };

  using Ptr = TextLogger*;

  static TextLogger& get(const std::string& name)
  {
    LoggerMap& loggerMap(getLoggerMap());

    // does logger already exist?
    LoggerMap::const_iterator found = loggerMap.find(name);
    if (found != loggerMap.end())
      return *found->second;
    else
      return registerLogger(name);
  }

  static TextLogger& get(const std::string& name, Level level);

private:
  using LoggerMap = std::unordered_map<std::string, Ptr>;

  // get around initialisation issue
  static LoggerMap& getLoggerMap()
  {
    static LoggerMap map;
    return map;
  }

  static TextLogger& registerLogger(const std::string& name);
};

class TextLogger
{
public:
  using Level = TextLog::Level;

  std::string name;
  Level enabledLevel;

  TextLogger() : name("Default"), enabledLevel(Level::INFO) {}
  TextLogger(const std::string& nameIn, Level levelIn = Level::INFO) : name(nameIn), enabledLevel(levelIn) {}

  bool isEnabled(Level level) { return enabledLevel >= level; }
};



inline TextLogger& TextLog::get(const std::string& name, Level level)
{
  TextLogger& logger = get(name);
  logger.enabledLevel = level;
  return logger;
}

inline TextLogger& TextLog::registerLogger(const std::string& name)
{
  LoggerMap& loggerMap(getLoggerMap());

  // TODO SYNC
  fmt::print("Registering new logger: {}\n", name);

  // no existing logger, create a new one - we only leak this memory on exit so no smart pointer needed
  loggerMap[name] = new TextLogger(name);

  return *loggerMap[name];
}




#define LOG(loggerIn_, level_, fmt_, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    TextLogger& logger_ = (loggerIn_); /* prevent multiple evaluation */                                                      \
    /* fmt::print("LOG: name={}, level={}\n", logger_->name, logger_->enabledLevel); */                                       \
    if (logger_.isEnabled(level_))                                                                                     \
      fmt::print("{}: " fmt_ "\n", logger_.name, ##__VA_ARGS__);                                                       \
  } while (0)


static void baseFoo() 
{ 
  LOG(TextLog::get("Base"), TextLog::WARNING, "this is the base"); 
}