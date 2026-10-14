/**
 * @file TextLogging.h
 *
 * This file declares functions to aid with logging text (including added standard
 * information) to text log files. Think of it as a better version of printf/cout
 * 
 * It uses fmtlib (python) style format statements by default.
 * For ostream style output to text logs, use TextLoggingStream.h instead.
 * 
 * Note: this code is thread safe and can operate in synchronous or asynchronous
 * mode. In asynchronous mode, the output to stdout or destination takes place
 * in an async thread (allowing the calling thread to avoid blocking on I/O).
 *
 * @author Rudi Villing
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Platform/Time.h"

#include <fmt/format.h>

#include <unordered_map>
#include <atomic>



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
     * This is slightly inefficient if used in a tight loop but easy to use
     * for infrequent error messges. It can be used as follows:
     *
     *   foo(ErrorStr(errno).str());
     */
    std::string str() const { return std::string(msg); }
};



// don't use this class directly - it's really part of the internal implementation
// but needs to be public for access by the macros
struct TextLogMsgInternal_
{
  fmt::memory_buffer msgFmt;  

  template <typename S, typename... Args>
  void format(const S& formatStr, Args&&... args)
  {
    // note: fmt::runtime does runtime parsing of formatStr - required because
    // we don't know the value of formatStr here at compile time - it will
    // have many different values, depending on who is calling the textlogging functions
    fmt::format_to(std::back_inserter(msgFmt), fmt::runtime(formatStr), std::forward<Args>(args)...);
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

  struct RobotContext
  {
    int teamNumber = -1;
    int playerNumber = -1;
  };

  static void setRobotContext(const RobotContext& ref)
  {
    robotContext = ref;
  }

  /**
   * get the default text logger which always has the INFO level
   * (It does not need to to a lookup each time so slightly more efficient
   * than doing TextLogging::get("Default") if you'll be calling it often.
   */
  static TextLogger& get();

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
   * log the the startup time using a full time stamp so that subsequent
   * textlog messages (which just use second and millisecond timestamps)
   * can be matched to real-world time.
   * 
   * @param tlogger the logger on which to output the message
   * @param level the level at which to output the message 
   */
  static void logStartupTime(TextLogger& tlogger, Level level);

  /**
   * request that text logging switches to synchronous/asynchronous mode
   * 
   * Generally asynchronous mode ensures that real time threads avoid
   * being blocked on textlogging I/O. However the async design does mean that
   * if the rate of textlogging messages gets too high, some messages will
   * be dropped.
   * 
   * @param asyncDesired true implies switch to async mode, otherwise switch to sync mode
   */
  static void requestAsyncOutput(bool asyncDesired = true);

  /**
   * Redirect all text logging output to the specified file handle.
   * Default is stdout.
   * 
   * This function is not thread-safe. It should be called before multiple 
   * threads are starting to log
   */
  static void setOutputFile(FILE* file);


public: // public for macro reasons but consider these private implementation details
  static std::atomic<bool> isLoggingEnabled;

  /**
   * Primary dispatch for fmt-style output from macros.
   * 
   * The general format of the output is:
   * 
   *   timeMs [threadName] teamNum-robotNum : formattedText
   * 
   * NOTE: this is really an internal use function for use by macros only
   * so don't call it directly
   */
  template <typename... Args>
  static inline void doLogMsg_(TextLogger &logger, Level level, fmt::format_string<Args...> fmt, Args &&...args)
  {
    // This is a tiny inline jump. It "packs" the pointers to your args
    // and immediately calls the shared implementation.
    doLogMsgPacked(logger, level, fmt, fmt::make_format_args(args...));
  }

  /**
   * Overload for legacy stream-style
   */
  static void doLogMsgStream_(TextLogger& logger, Level level, fmt::string_view msg);


private:
  using Ptr = TextLogger*;
  using LoggerMap = std::unordered_map<std::string, Ptr>;

  static constexpr const char* levelStrings[7] = { "OFF", "FATAL", "ERROR", "W", "I", "D", "V" };
  static thread_local RobotContext robotContext;

  static void doLogMsgPacked(TextLogger& logger, Level level, fmt::string_view format, fmt::format_args args);
  static void formatHeader(fmt::memory_buffer& buf, TextLogger& logger, Level level);
  static void dispatchAsync(fmt::memory_buffer&& finalBuffer);

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

  friend class TextLoggingHelper;
};

// This must be defined here (outside the class body) to resolve a "definition of enclosing class" error.
inline thread_local TextLogging::RobotContext TextLogging::robotContext;

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

// we optimise for loggers that are disabled in the following
#define TLOG_IMPL(logger_, lvl_, fmtStr_, ...)                                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    if (logger_.isEnabled(lvl_) && TextLogging::isLoggingEnabled.load(std::memory_order_relaxed)) [[unlikely]]         \
      TextLogging::doLogMsg_(logger_, lvl_, FMT_STRING(fmtStr_) __VA_OPT__(, __VA_ARGS__));                                                              \
  } while (0)

#define TLOGF(logger_,...) TLOG_IMPL(logger_, TextLogging::FATAL, __VA_ARGS__)
#define TLOGE(logger_,...) TLOG_IMPL(logger_, TextLogging::ERROR, __VA_ARGS__)
#define TLOGW(logger_,...) TLOG_IMPL(logger_, TextLogging::WARNING, __VA_ARGS__)
#define TLOGI(logger_,...) TLOG_IMPL(logger_, TextLogging::INFO, __VA_ARGS__)
#define TLOGD(logger_,...) TLOG_IMPL(logger_, TextLogging::DEBUG, __VA_ARGS__)
#define TLOGV(logger_,...) TLOG_IMPL(logger_, TextLogging::VERBOSE, __VA_ARGS__)


// in conjunction with BHAssert stuff

/**
 * TextLog and then always abort, regardless of the value of NDEBUG
 */
#define TLOG_ABORT(logger_, ...)                                                                                      \
  do                                                                                                                   \
  {                                                                                                                    \
    TLOGF(logger_, __VA_ARGS__);                                                                                       \
    FAIL_FMT(__VA_ARGS__);                                                                                             \
    Assert::abort();                                                                                                   \
  } while (0)


#ifdef NDEBUG

/**
 * TextLog and then follow the semantics of FAIL which aborts only if
 * NDEBUG is not set
 */
#define TLOG_FAIL(logger_, ...)                                                                                      \
  do                                                                                                                 \
  {                                                                                                                  \
    TLOGE(logger_, __VA_ARGS__);                                                                                     \
    FAIL_FMT(__VA_ARGS__);                                                                                           \
  } while (0)

#define TLOG_VERIFY(logger_, cond)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(cond))                                                                                                       \
      TLOGE(logger_, "VERIFY({}) failed", #cond);                                                                      \
  } while (0)

#define TLOG_VERIFY_ERRNO TLOG_VERIFY

#else // not NDEBUG

#define TLOG_FAIL(logger_, ...)                                                                                      \
  do                                                                                                                 \
  {                                                                                                                  \
    TLOGF(logger_, __VA_ARGS__);                                                                                     \
    FAIL_FMT(__VA_ARGS__);                                                                                           \
  } while (0)

#define TLOG_VERIFY(logger_, cond)                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(cond))                                                                                                       \
      TLOG_ABORT(logger_, "VERIFY({}) failed", #cond);                                                                \
  } while (0)

#define TLOG_VERIFY_ERRNO(logger_, cond)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!(cond))                                                                                                       \
    {                                                                                                                  \
      TLOGF(logger_, "VERIFY({}) failed: {}", #cond);                                                                  \
      TLOG_ABORT(logger_, "  error: {}", ErrorStr().str());                                                           \
    }                                                                                                                  \
  } while (0)

#endif // NDEBUG


#else // TLOG_DISABLED

#define TLOGF(...) (void)0
#define TLOGE(...) (void)0
#define TLOGW(...) (void)0
#define TLOGI(...) (void)0
#define TLOGD(...) (void)0
#define TLOGV(...) (void)0

// even if not logging, we still need to fall back to standard FAIL and VERIFY behaviour
#define TLOG_ABORT(logger_,...) static_cast<void>(logger_, FAIL(fmt::format(__VA_ARGS__)), Assert::abort(), 0)
#define TLOG_FAIL(logger_,...) static_cast<void>(logger_, FAIL(fmt::format(__VA_ARGS__)), 0)
#define TLOG_VERIFY(logger_,cond) static_cast<void>(logger_, VERIFY(cond), 0)
#define TLOG_VERIFY_ERRNO TLOG_VERIFY

#endif // TLOG_DISABLED


/**
 * A simple stopwatch class to support text logging
 * (distinct from the stopwatches associated with debug streaming and SimRobot)
 */
class TextLogStopwatch
{
#ifndef TLOG_DISABLED
public:
  explicit TextLogStopwatch(TextLogger& logger, TextLogging::Level level, const std::string& name)
      : mLogger(logger), mLevel(level), mName(name)
  {
    mStartMicros = Time::getCurrentThreadTime();
  }

  ~TextLogStopwatch()
  {
    unsigned long durationMicros = mStartMicros - Time::getCurrentThreadTime();
    TLOG_IMPL(mLogger, mLevel, "Stopwatch: {} took {}.{:03d} ms", mName, durationMicros/1000, durationMicros%1000);
  }

private:
  TextLogger &mLogger;
  TextLogging::Level mLevel;
  std::string mName;
  unsigned long long mStartMicros;
#endif // TLOG_DISABLED
};

#ifndef TLOG_DISABLED
#define TLOG_STOPWATCH(logger_, lvl_, name_) if (TextLogStopwatch(logger_, lvl_, name_), true)
#else
#define TLOG_STOPWATCH(logger_, lvl_, name_) if (true)
#endif // TLOG_DISABLED





/**
 * Use this macro to declare access to a logger in a .cpp file
 */
#define DECL_TLOGGER(varName_, loggerName_, level_) static TextLogger& varName_ = TextLogging::get(loggerName_, level_)

/**
 * Use this macro to declare access to logger as a member or local variable
 * 
 * This would typically be used to add a member variable to a class that only has a header file. Otherwise
 * the DECL_TLOGGER macro which defines a static file level logger is a better solution
 *
 * This version creates a var per instance and is needed for template classes if they will be instanciated
 * in different translation units
 */
#define DECL_TLOGGER_VAR(varName_, loggerName_, level_)                                                                \
  TextLogger &varName_ = TextLogging::get(loggerName_, level_)

/**
 * Use this macro to declare access to logger as a member or local variable
 * 
 * This would typically be used to add a member variable to a class that only has a header file. Otherwise
 * the DECL_TLOGGER macro which defines a static file level logger is a better solution
 *
 * This uses C++17 static inline to make this a static var rather than one per instance
 *
 * @warning You cannot safely use this in a template class. The compiler does not guarantee that the static
 *          var will be initialized before the constructor runs depending on the translation unit (cpp file)
 *          the class is instanciated in.
 */
#define DECL_TLOGGER_STATIC_VAR(varName_, loggerName_, level_)                                                         \
  static inline TextLogger &varName_ = TextLogging::get(loggerName_, level_)

/**
 * Occasionally you might want to use this macro declare access to a logger in 
 * a .h file at the file level or as a member of a class.
 * However its usage should be rare and generally you should prefer DECL_TLOGGER_VAR/STATIC_VAR
 * unless the addition of a reference variable is impossible/difficult or somehow adds too much 
 * overhead.
 */
#define DECL_TLOGGER_FN(fnName_, loggerName_, level_)                                                                  \
  static TextLogger &fnName_()                                                                                         \
  {                                                                                                                    \
    static TextLogger &tlogger = TextLogging::get(loggerName_, level_);                                                \
    return tlogger;                                                                                                    \
  }
