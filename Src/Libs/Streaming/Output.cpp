#include "Output.h"

#include "Streaming/MessageQueue.h"

#include "TextLogging/TextLoggingStream.h"

#ifdef WINDOWS
#include <Windows.h>
#endif

DECL_TLOGGER(tlogger, "Output", TextLogging::INFO);

// #if !(defined TARGET_ROBOT && defined NDEBUG)
// #ifdef WINDOWS
// #include <Windows.h>
// #endif
// #include <cstdio>
// 
// void Output::print(const char* message)
// {
//   fprintf(stderr, "%s\n", message);
// #ifdef WINDOWS
//   OutputDebugStringA(message);
//   OutputDebugStringA("\n");
// #endif
// }
// #endif


namespace Output {

LogHelper::LogHelper(LogType type)
    : _type(type)
{
}

LogHelper::~LogHelper()
{
  // 1. Grab the pointer to our pre-allocated buffer
  const char *msg = _stream.data();
  TLOG_STREAM_IMPL(tlogger, (_type == error ? TextLogging::ERROR : TextLogging::WARNING), msg);

#if defined TARGET_ROBOT && defined NDEBUG
  // If we want to keep cerr:
  // std::cerr << msg << std::endl;
#else
  // PC Path: Duplicate to Queue and Debugger
  if (Global::debugOutExists())
    Global::getDebugOut().text(idText) << msg;

#ifdef WINDOWS
  OutputDebugStringA(msg);
  OutputDebugStringA("\n");
#endif

#endif
}

} // namespace Output