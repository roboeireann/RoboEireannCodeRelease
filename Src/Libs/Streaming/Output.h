/**
 * @file Output.h
 *
 * Macros and functions for outputting text.
 *
 * @author Martin Lötzsch
 * @author Thomas Röfer
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Streaming/OutStreams.h"

#if defined TARGET_ROBOT && defined NDEBUG
// #include <iostream>

/**
 * Ignore debug requests
 */
#define OUTPUT(type, format, expression) static_cast<void>(0)
#define OUTPUT_TEXT(expression) static_cast<void>(0)
#define OUTPUT_TEXT_FMT(message, ...) static_cast<void>(0)

/**
 * A macro for sending warning messages.
 * @param message A message streamable as text.
 */
// #define OUTPUT_WARNING(message) \
//   do \
//     std::cerr << "Warning: " << message << std::endl; \
//   while(false)

/**
 * A macro for sending error messages.
 * Note that this macro is enabled regardless of which
 * target is used to build.
 * @param message A message streamable as text.
 */
// #define OUTPUT_ERROR(message)                                                                                          \
//   do                                                                                                                   \
//     std::cerr << "Error: " << message << std::endl;                                                                    \
//   while (false)

#else // not TARGET_ROBOT && NDEBUG

#include "Streaming/Global.h"
#include "Streaming/MessageQueue.h"
#include "fmt/core.h"

/**
 * A macro for sending debug messages.
 *
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text)
 * @param expression A streamable expression
 *
 * Examples:
 * <pre>
 * OUTPUT(idCameraImage, bin, *pMyCameraImage);
 * OUTPUT_TEXT("MyObject:myFunction() invoked");
 * OUTPUT_TEXT("i: " << i << ", j:" << j);
 * </pre>
 */
#define OUTPUT(type, format, expression) \
  Global::getDebugOut().format(type) << expression

/**
 * Shortcut for outputting text messages.
 * @param expression A streaming expression to output.
 */
#define OUTPUT_TEXT(expression) OUTPUT(idText, text, expression)
#define OUTPUT_TEXT_FMT(message, ...) Output::LogHelper(Output::text).stream() << fmt::format(message, ##__VA_ARGS__)

#endif // TARGET_ROBOT && NDEBUG

/**
 * A macro for sending warning messages.
 * @param message A message streamable as text.
 */
#ifdef NDEBUG

#define OUTPUT_WARNING(message) static_cast<void>(0)
#define OUTPUT_WARNING_FMT(message, ...) static_cast<void>(0)

#else
// #define OUTPUT_WARNING(message) \
//   do \
//   { \
//     if(Global::debugOutExists()) \
//       OUTPUT_TEXT("Warning: " << message); \
//     OutTextRawMemory _stream; \
//     _stream << "Warning: " << message; \
//     Output::print(_stream.data()); \
//   } \
//   while(false)
#define OUTPUT_WARNING(message) Output::LogHelper(Output::warning).stream() << message
#define OUTPUT_WARNING_FMT(message, ...) Output::LogHelper(Output::warning).stream() << fmt::format(message, ##__VA_ARGS__)

#endif // NDEBUG


/**
 * A macro for sending error messages.
 * Note that this macro is enabled regardless of which
 * target is used to build.
 * @param message A message streamable as text.
 */
// #define OUTPUT_ERROR(message)                                                                                          \
//   do                                                                                                                   \
//   {                                                                                                                    \
//     if (Global::debugOutExists())                                                                                      \
//       OUTPUT_TEXT("Error: " << message);                                                                               \
//     OutTextRawMemory _stream;                                                                                          \
//     _stream << "Error: " << message;                                                                                   \
//     Output::print(_stream.data());                                                                                     \
//   } while (false)
#define OUTPUT_ERROR(message) Output::LogHelper(Output::error).stream() << message
#define OUTPUT_ERROR_FMT(message, ...) Output::LogHelper(Output::error).stream() << fmt::format(message, ##__VA_ARGS__)

namespace Output
{
  /**
   * Prints a message to stderr.
   * @param message The error to print.
   */
  // void print(const char* message);

  enum LogType { text, warning, error };

  class LogHelper {
  public:
    LogHelper(LogType type);
    ~LogHelper(); // Logic strictly in .cpp
    
    // Stream directly into the existing raw memory buffer
    OutTextRawMemory& stream() { return _stream; }

  private:
    LogType _type;
    OutTextRawMemory _stream; 
  };
}
