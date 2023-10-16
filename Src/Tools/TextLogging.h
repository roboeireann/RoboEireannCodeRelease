/**
 * @file TextLogging.h
 *
 * This file declares functions to aid with logging text (including added standard
 * information) to text log files. Think of it as a better version of printf/cout
 * 
 * Note: this code is not synchronised/thread-safe. Most of the time it is
 * good enough, but when a lot of messages are being printed it is possible
 * that messages from different threads get mixed up. (The solution to this is 
 * coming later.)
 *
 * @author Rudi Villing
 */

#pragma once

#include "Platform/Time.h"

#include "fmt/format.h"

#include <sstream>
#include <unordered_map>



/**
 * utility class to help with errno to string and wrap reentrant version
 * of strerror (which is a little inconvenient to use otherwise)
 */

class ErrorStr
{
    enum { SIZE = 256 };
    char buf[SIZE];
    const char * msg;
public:
    ErrorStr(int errnum = errno)
    {
//        // XSI/POSIX
//        int rc = strerror_r(errnum, buf, SIZE);
//        if (rc == 0)
//            msg = buf;
//        else if (rc == EINVAL)
//            msg = "Strerror:unknown error number";
//        else
//            msg = "Strerror:msg too big";

        // GCC version
        msg = strerror_r(errnum, buf, SIZE);
    }

    /**
     * return a c string. Note that this is a pointer to internal
     * data that goes away if the ErrorStr object goes out of scope.
     *
     * WARNING: Do not call on a temporary object used as a function
     * parameter, e.g. foo(ErrorStr(errno).cstr()).
     * Instead do the following:
     *    ErrorStr errorStr(errno);
     *    foo(errorStr.cstr());
     */
    const char * cstr() const { return msg; }

    /**
     * return a std::string which is a copy of the internal data.
     * This is inefficient but easy to use and can be used as follows:
     *
     *   foo(ErrorStr(errno).str());
     */
    std::string str() const { return std::string(msg); }
};



// don't use this class directly - it's really part of the internal implementation
// but needs to be public for access by the macros
struct MacrosOnlyTextLogMsg_
{
  fmt::memory_buffer tmpFmt;
  std::stringstream tmpStream;

  template <typename S, typename... Args, typename Char = fmt::char_t<S> >
  void format(const S& formatStr, Args&&... args)
  {
    const auto& vargs = fmt::make_args_checked<Args...>(formatStr, args...);
    fmt::vformat_to(tmpFmt, fmt::to_string_view(formatStr), vargs);
    // fmt::format_to(tmpFmt, formatStr, std::forward<Args>(args)...);
  }
  
};


class TextLogger;

/**
 * General access to TextLoggers.
 */
class TextLogging
{
public:
  /**
   * possible log levels
   */
  enum Level
  {
    OFF = 0,
    FATAL, // TODO: the same as ERROR for now, but should mean continuation not possible and we're exiting
    ERROR,
    WARNING,
    INFO,
    DEBUG,
    VERBOSE
    // NOTE: if you add to the enums, update levelStrings below
  };

  /**
   * get a text logger by name
   * If the text logger does not already exist a new one is registered with
   * the default (INFO) logging level
   * @param name the name of the logger
   */
  static TextLogger& get(const std::string& name);

  /**
   * get a text logger by name and also set which logging level is enabled
   * If the text logger does not already exist a new one is registered and the 
   * specified log level is set
   * @param name the name of the logger
   * @param level the enabled logging level for the logger
   */
  static TextLogger& get(const std::string& name, Level level);

  /**
   * The general format of the output is:
   * 
   *   timeMs [threadName] teamNum-robotNum : formattedText
   * 
   * NOTE: this is really an internal use function for use by macros only
   * so don't call it directly
   */
  static void macrosOnlyOutput_(TextLogger& logger, Level level, const MacrosOnlyTextLogMsg_& msg);

private:
  using Ptr = TextLogger*;
  using LoggerMap = std::unordered_map<std::string, Ptr>;

  static constexpr const char* levelStrings[7] = { "OFF", "FATAL", "ERROR", "W", "I", "D", "V" };

  // There is a subtle initialization order issue that requires us to
  // initialize on first use rather than assuming normal static initialization
  // will work.
  // Basically code in other .cpp files written as follows,
  //
  //   static TextLogger& myLogger = TextLogging::get("name");
  //
  // requires the loggerMap (repository of loggers) to be initialized already.
  // However the compiler does not guarantee the initialization
  // order of static variables between files (compilation units).
  // Hence the solution is to construct the loggerMap when needed on first use.
  // (This is really only an issue for log output during the programme startup)

  static LoggerMap& getLoggerMap()
  {
    static LoggerMap map;
    return map;
  }

  // we break this function out so that we can include synchronization later
  // if needed
  static TextLogger& registerLogger(const std::string& name);
};

/**
 * A TextLogger has a name and enabled level.
 * In your code you log using a logger (and the name will be prefixed to every
 * line that you log). The enabledLevel of a logger determines whether it will
 * actually log anything or not. This makes it easy to temporarily switch
 * on or off debugging text while only having to change the single line of
 * code where the TextLogger is obtained.
 * 
 * Generally at the top of a .cpp file you would declare a static Logger, e.g.
 * 
 *     static TextLogger& logger = TextLogging::get("SomeLogger", TextLogging::DEBUG);
 * 
 * For .h files you can either declare a logger as a member in your class or define
 * a private static function in your class something like the following:
 * 
 *    static TextLogger& tlogger()
 *    {
 *      static TextLogger& tlogger = TextLogging::get("BasicCoro", TextLogging::INFO);
 *      return tlogger;
 *    }
 *
 */

class TextLogger
{
public:
  using Level = TextLogging::Level;

  TextLogger() : name("Default"), enabledLevel(Level::INFO) {}
  TextLogger(const std::string &nameIn, Level levelIn = Level::INFO) : name(nameIn), enabledLevel(levelIn) {}

  const std::string& getName() const { return name; }
  Level getEnabledLevel() const { return enabledLevel; }
  bool isEnabled(Level level) const { return enabledLevel >= level; }

private:
  std::string name;
  Level enabledLevel;

  friend class TextLogging;
};


// ============================================================================
// the macros...
// ============================================================================

#ifndef TLOG_DISABLED

#define TLOG_IMPL(logger_, lvl_, fmt_, ...)                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    if (logger_.isEnabled(lvl_))                                                                                       \
    {                                                                                                                  \
      MacrosOnlyTextLogMsg_ msg_;                                                                                      \
      msg_.format((fmt_), ##__VA_ARGS__);                                                                              \
      TextLogging::macrosOnlyOutput_(logger_, lvl_, msg_);                                                                      \
    }                                                                                                                  \
  } while (0)

#define TLOGF(logger_,...) TLOG_IMPL(logger_, TextLogging::FATAL, __VA_ARGS__)
#define TLOGE(logger_,...) TLOG_IMPL(logger_, TextLogging::ERROR, __VA_ARGS__)
#define TLOGW(logger_,...) TLOG_IMPL(logger_, TextLogging::WARNING, __VA_ARGS__)
#define TLOGI(logger_,...) TLOG_IMPL(logger_, TextLogging::INFO, __VA_ARGS__)
#define TLOGD(logger_,...) TLOG_IMPL(logger_, TextLogging::DEBUG, __VA_ARGS__)
#define TLOGV(logger_,...) TLOG_IMPL(logger_, TextLogging::VERBOSE, __VA_ARGS__)

#define TLOG_IMPL_STREAM(logger_, lvl_, insertable_)                                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    if (logger_.isEnabled(lvl_))                                                                                       \
    {                                                                                                                  \
      MacrosOnlyTextLogMsg_ msg_;                                                                                      \
      msg_.tmpStream << insertable_;                                                                                   \
      TextLogging::macrosOnlyOutput_(logger_, lvl_, msg_);                                                             \
    }                                                                                                                  \
  } while (0)

#define TLOGF_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::FATAL, insertable_)
#define TLOGE_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::ERROR, insertable_)
#define TLOGW_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::WARNING, insertable_)
#define TLOGI_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::INFO, insertable_)
#define TLOGD_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::DEBUG, insertable_)
#define TLOGV_STREAM(logger_,insertable_) TLOG_IMPL_STREAM(logger_, TextLogging::VERBOSE, insertable_)

#else // TLOG_DISABLED

#define TLOGF(...) (void)0
#define TLOGE(...) (void)0
#define TLOGW(...) (void)0
#define TLOGI(...) (void)0
#define TLOGD(...) (void)0
#define TLOGV(...) (void)0

#define TLOGF_STREAM(insertable_) (void)0
#define TLOGE_STREAM(insertable_) (void)0
#define TLOGW_STREAM(insertable_) (void)0
#define TLOGI_STREAM(insertable_) (void)0
#define TLOGD_STREAM(insertable_) (void)0
#define TLOGV_STREAM(insertable_) (void)0

#endif // TLOG_DISABLED

/**
 * Use this macro for variables that may be declared but unused if TLOG_DISABLED is defined
 */
#define TLOG_USE(var_) (void)var_

/**
 * Use this macro to declare access to a logger in a .cpp file
 */
#define DECL_TLOGGER(varName_, loggerName_, level_) static TextLogger& varName_ = TextLogging::get(loggerName_, level_)

/**
 * Use this macro declare access to a logger in a .h file at the file level
 * or as a member of a class
 */
#define DECL_TLOGGER_FN(fnName_, loggerName_, level_)                                                                  \
  static TextLogger &fnName_()                                                                                         \
  {                                                                                                                    \
    static TextLogger &tlogger = TextLogging::get(loggerName_, level_);                                                       \
    return tlogger;                                                                                                    \
  }
