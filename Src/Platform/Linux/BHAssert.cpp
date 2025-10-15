/**
 * @file Platform/Linux/BHAssert.cpp
 * Some helper functions for low level debugging
 * @author Colin Graf
 */

#include "Platform/BHAssert.h"

#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <iostream>

#include <signal.h>

void Assert::print(const char* file, int line, const char* format, ...)
{
  char data[320];
  int length = std::snprintf(data, sizeof(data) - 2, "%s:%d: ", file, line);
  if(length < 0)
    length = sizeof(data) - 2;
  va_list ap;
  va_start(ap, format);
  int i = vsnprintf(data + length, sizeof(data) - length - 2, format, ap);
  if(i < 0)
    length = sizeof(data) - 2;
  else
    length += i;
  va_end(ap);
  data[length++] = '\n';
  data[length] = '\0';
  fputs(data, stderr);
  fflush(stderr);
}

void Assert::print(const char* file, int line, const std::string& message)
{
  std::cerr << file << ":" << line << ": " << message << std::endl;
}

void Assert::abort()
{
  ::abort();
}

const char* Assert::getSignalName(int signum)
{
  const char* signalNames[] =
  {
    "",
    "sigHUP",
    "sigINT",
    "sigQUIT",
    "sigILL",
    "sigTRAP",
    "sigABRT",
    "sigBUS",
    "sigFPE",
    "sigKILL",
    "sigUSR1",
    "sigSEGV",
    "sigUSR2",
    "sigPIPE",
    "sigALRM",
    "sigTERM"
  };

  if (0 <= signum && signum < int(sizeof(signalNames) / sizeof(*signalNames)))
    return signalNames[signum];
  else
  {
    switch (signum)
    {
    case SIGUSR1:
      return "sigUSR1";

    default:
      static char buf[20];
      sprintf(buf, "signal %d", signum);
      return buf;
    }
  }
}

