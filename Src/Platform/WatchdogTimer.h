/**
 * @file Platform/WatchdogTimer.h
 *
 * Platform dependent watchdog timer - based on Posix in Linux/Nao
 *
 * @author Rudi Villing
 */

#pragma once
#ifndef TARGET_TOOL

#ifdef LINUX
  #include <signal.h>
  #include <time.h>
#endif

class WatchdogTimer
{
public:
  WatchdogTimer();
  ~WatchdogTimer();
  void setTimeout(unsigned ms);
  void cancelTimeout();

private:

#ifdef LINUX
  timer_t mTimerId;
#endif
};
#endif // TARGET_TOOL