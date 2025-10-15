/**
 * @file LogfileLoggerMonitor.cpp
 * 
 * Implements a module that monitors the status of logfile logging - primarily
 * to provide data needed to present that the status via HRI
 * 
 * @author Rudi Villing
 */

#include "LogfileLoggerMonitor.h"

#include "Tools/TextLogging.h"
#include "Tools/Streams/TypeInfo.h"


MAKE_MODULE(LogfileLoggerMonitor, behaviorControl);


DECL_TLOGGER(tlogger, "LogfileLoggerMonitor", TextLogging::INFO);

void LogfileLoggerMonitor::update(LogfileLoggerStatus& logfileLoggerStatus)
{
  // this "function" will be called by the Cognition execution unit
  // before any modules run to provide the most recent LogfileLogger status
  logfileLoggerStatus.updateStatus = [this](LogfileLogger::State loggerStateIn, unsigned buffersToWriteIn)
  {
    // this function will be called sometime before update, so cache the relevant values
    loggerState.update(loggerStateIn);
    buffersToWrite = buffersToWriteIn;
  };

  // update the representation from the cached values
  logfileLoggerStatus.state = loggerState.value;
  logfileLoggerStatus.buffersToWrite = buffersToWrite; // xyz

  if (loggerState.valueChanged())
  {
    if (loggerState.value == LogfileLogger::FINISH_LOGGING)
      finishingBuffersToWrite = std::max(1u, buffersToWrite); // we set the min at 1 to prevent divide by zero errors in edge cases

    TLOGI(tlogger, "update: detected logger state change to {}", TypeRegistry::getEnumName(loggerState.value));

    if ((loggerState.value == LogfileLogger::IDLE) && (loggerState.prevValue == LogfileLogger::FINISH_LOGGING))
      TLOGI(tlogger, "update: detected log file completely written");
  }

  if (loggerState.value == LogfileLogger::FINISH_LOGGING)
  {
    logfileLoggerStatus.finishProgress =
        1.f - std::min(1.f, static_cast<float>(buffersToWrite) / finishingBuffersToWrite);

    TLOGV(tlogger, "update: finish logging, {} buffersToWrite (initially {}, so {}% done)", buffersToWrite,
          finishingBuffersToWrite, 100.f * logfileLoggerStatus.finishProgress);
  }
  else
    logfileLoggerStatus.finishProgress = 0.f;
}


