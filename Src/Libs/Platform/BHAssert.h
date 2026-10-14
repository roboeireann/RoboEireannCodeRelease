/**
 * @file Platform/BHAssert.h
 * This file contains macros for low level debugging
 * @author Thomas Röfer
 * @author Colin Graf
 * @author Rudi Villing (modifications)
 */

#pragma once

#undef ASSERT
#undef FAIL
#undef VERIFY

#include <string>
#include <sstream>

/**
 * Some tools for low level debugging
 */
class Assert
{
public:
  /** FailHelper helps with the implementation of the FAIL, ASSERT, VERIFY macros */
  class FailHelper
  {
  public:
    FailHelper(const char *file, int line);
    ~FailHelper(); // FAIL Logic moved to .cpp
    std::ostream &stream() { return _stream; }

  private:
    const char *_file;
    int _line;
    std::stringstream _stream;
  };

  /**
   * Prints a formatted message to stdout
   * @param file The name of the current file (__FILE__)
   * @param line The current file line (__LINE__)
   * @param format The format of the message to be printed
   */
  static void print(const char* file, int line, const char* format, ...);
  static void print(const char* file, int line, const std::string& message);

  /**
   * Aborts the execution of the program
   */
  static void abort();

#ifdef TARGET_ROBOT
  /**
   * Initializes some log message ring buffers associated to the current thread
   * @param name The name of the current thread
   * @return Whether the initialization was successful
   */
  static bool logInit(const char* name);

  /**
   * Adds a message to the log message ring buffer
   * @param track The id of the used ring buffer
   * @param file The name of the current file (__FILE__)
   * @param line The current file line (__LINE__)
   * @param message The message to be added
   */
  static void logAdd(int track, const char* file, int line, const std::string& message);

  /**
   * Dumps the content of the log message ring buffers to stderr or robotHomeDir/redump.trace
   * @param toStderr Whether to dump the content to stderr or robotHomeDir/redump.log
   * @param termSignal A signal that was emitted to terminate the roboeireann process
   */
  static void logDump(bool toStderr, int termSignal);
#endif // TARGET_ROBOT

#if defined TARGET_ROBOT || defined LINUX
  /**
   * get the signal name corresponding to a signal number
   */
  static const char* getSignalName(int signum);
#endif
};

/**
 * ASSERT prints a message if \c cond is \c false and NDEBUG is not defined.
 * ASSERT does not evaluate \c cond if NDEBUG is defined.
 * @param c The condition to be checked.
 */
#ifdef NDEBUG
  #define ASSERT(cond) static_cast<void>(0)
#else
  // ASSERT only triggers if the condition is false
  #define ASSERT(cond) \
    do { if (!!(cond)) ; else Assert::FailHelper(__FILE__, __LINE__).stream() << "ASSERT(" #cond ") failed"; } while(false)
#endif

/**
 * ASSERTs cond
 * if NDEBUG is defined the safe value will be returned in the case of a failing conditional check
 * it is recommended to use SAFE_ASSERT just for cond that are fast evaluated
 */
#ifdef NDEBUG
#define SAFE_ASSERT(cond, safe) while(!(cond)) return (safe)
#else
#define SAFE_ASSERT(cond, safe) ASSERT(cond)
#endif

/**
 * FAIL is equivalent to ASSERT(false) and additionally prints a text.
 * This text is passed into a std::stringstream, thus FAIL("error " << 1) is a valid expression.
 */
#ifdef NDEBUG

#define FAIL(text) static_cast<void>(0)
#define FAIL_FMT(fmtString,...) static_cast<void>(0)

#else

#define FAIL(text) Assert::FailHelper(__FILE__, __LINE__).stream() << "FAIL: " << text
#define FAIL_FMT(fmtString, ...)                                                                                       \
  Assert::FailHelper(__FILE__, __LINE__).stream() << "FAIL: " << fmt::format(fmtString, ##__VA_ARGS__)

#endif


/**
 * VERIFY prints a message if \cond cond is \c false and NDEBUG is not defined.
 * VERIFY does evaluate \c cond even if NDEBUG is defined.
 * @param c The condition to be checked.
 */
#ifdef NDEBUG
  #define VERIFY(cond) static_cast<void>(cond)
#else
  // VERIFY behaves like ASSERT but the condition is evaluated in NDEBUG too (see above)
  #define VERIFY(cond) \
    do { if (!!(cond)) ; else Assert::FailHelper(__FILE__, __LINE__).stream() << "VERIFY(" #cond ") failed"; } while(false)
#endif

/**
 * Initializes some log message ring buffers associated to the current thread
 * @param name The name of the current thread
 * @return Whether the initialization was successful
 */
#if defined NDEBUG || !defined TARGET_ROBOT
#define BH_TRACE_INIT(name) static_cast<void>(0)
#else
#define BH_TRACE_INIT(name) VERIFY(Assert::logInit(name))
#endif

/**
 * Adds a message to the log message ring buffer
 * @param message The message to be added
 */
#if defined NDEBUG || !defined TARGET_ROBOT
#define BH_TRACE_MSG(message) static_cast<void>(0)
#else
#define BH_TRACE_MSG(message) Assert::logAdd(0, __FILE__, __LINE__, message)
#endif

/**
 * Adds the current line of code to the log message ring buffer
 */
#if defined NDEBUG || !defined TARGET_ROBOT
#define BH_TRACE static_cast<void>(0)
#else
#define BH_TRACE Assert::logAdd(0, __FILE__, __LINE__, "")
#endif

/**
 * Adds a warning to the output of the compiler:
 * Example: WARNING("Replace this later")
 */
#define _WARNING(text) #text
#ifdef WINDOWS
#define _WARNING2(text) _WARNING(text)
#define WARNING(text) _Pragma(_WARNING(message (__FILE__ "(" _WARNING2(__LINE__) "): warning: " text)))
#else
#define WARNING(text) _Pragma(_WARNING(GCC warning text))
#endif
