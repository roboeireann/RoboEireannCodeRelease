/**
 * @file LogfileLoggerMonitor.h
 * 
 * Implements a module that monitors the status of logfile logging - primarily
 * to provide data needed to present that the status via HRI
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Infrastructure/LogfileLoggerStatus.h"

#include "Tools/Logging/LogfileLogger.h"
#include "Tools/EdgeTrigger.h"


MODULE(LogfileLoggerMonitor,
{,
  PROVIDES(LogfileLoggerStatus),
});

class LogfileLoggerMonitor : public LogfileLoggerMonitorBase
{
private:
  EdgeTrigger<LogfileLogger::State> loggerState { LogfileLogger::IDLE };
  unsigned buffersToWrite = 0;
  unsigned finishingBuffersToWrite = 0;

  void update(LogfileLoggerStatus& loggerStatus) override;
};


