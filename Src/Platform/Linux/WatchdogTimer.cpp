/**
 * @file Platform/Linux/WatchdogTimer.cpp
 *
 * Platform dependent watchdog timer - based on Posix in Linux/Nao
 *
 * @author Rudi Villing
 */

#ifndef TARGET_TOOL
#include "Platform/WatchdogTimer.h"

#include "Tools/TextLogging.h"

#include <signal.h>
#include <time.h>



DECL_TLOGGER(tlogger, "WatchdogTimer", TextLogging::WARNING);


// ====================================================================
// interface class just delegates to the implementation

WatchdogTimer::WatchdogTimer()
{
  mTimerId = 0;

  struct sigevent sev;

  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIGALRM;
  sev.sigev_value.sival_int = 0;

  if (timer_create(CLOCK_MONOTONIC, &sev, &mTimerId) == -1)
    TLOGF_ABORT(tlogger, "Failed to create watchdog timer: {}", ErrorStr().str());
}

WatchdogTimer::~WatchdogTimer()
{
  if (mTimerId != 0)
    timer_delete(mTimerId);
}

void WatchdogTimer::setTimeout(unsigned ms)
{
  struct itimerspec itspec;
  // initial expiry
  itspec.it_value.tv_nsec = (ms % 1000) * 1000000;
  itspec.it_value.tv_sec = ms / 1000;
  // subsequent intervals (not relevant to us so set it zero which means don't use it)
  itspec.it_interval.tv_nsec = 0;
  itspec.it_interval.tv_sec = 0;

  if (timer_settime(mTimerId, /*flags: */ 0, &itspec, NULL) == -1)
    TLOGF_ABORT(tlogger, "Failed to set watchdog timer timeout({} ms): {}", ms, ErrorStr().str());
}

void WatchdogTimer::cancelTimeout()
{
  struct itimerspec itspec;
  // disarm the timer by setting all times to zero
  itspec.it_value.tv_nsec = 0;
  itspec.it_value.tv_sec = 0;
  itspec.it_interval.tv_nsec = 0;
  itspec.it_interval.tv_sec = 0;

  if (timer_settime(mTimerId, /*flags: */ 0, &itspec, NULL) == -1)
    TLOGF_ABORT(tlogger, "Failed to cancel watchdog timer timeout: {}", ErrorStr().str());
}
#endif // TARGET_TOOL