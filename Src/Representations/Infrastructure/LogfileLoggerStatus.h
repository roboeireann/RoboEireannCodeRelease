/**
 * @file Representations/Infrastructure/LogfileLoggerStatus.h
 *
 * Defines a representation to keep track of the logfile logger status
 * (particularly when a game/logging period finishes)

 * @author Rudi Villing
 */

#pragma once

#include "Tools/Logging/LogfileLogger.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"

STREAMABLE(LogfileLoggerStatus,
{
  FUNCTION(void(LogfileLogger::State loggerState, unsigned buffersToWrite)) updateStatus;

  /* ----------------- separate header from streamable members (note comma at end) ------------------ */,

  (LogfileLogger::State) state, ///< the recent state of the logger
  (unsigned) buffersToWrite, ///< the number of buffers remaining to be written
  (float) finishProgress, ///< while finishing this is a value between 0 and 1, otherwise 0
});