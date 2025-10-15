/**
 * @file Logger.h
 *
 * This file declares a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Platform/Semaphore.h"
#include "Platform/Thread.h"
#include "Tools/Framework/Configuration.h"
#include "Tools/MessageQueue/MessageQueue.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InStreams.h"
#include <deque>
#include <stack>

struct GameInfo; // forward declaration to avoid include

STREAMABLE(LogfileLogger,
{
  ENUM(State, 
  {,
    IDLE, 
    LOGGING, 
    FINISH_LOGGING,
  });  

  /** Which representations will be logged for a certain thread? */
  STREAMABLE(RepresentationsPerThread,
  {,
    (std::string) thread,
    (std::vector<std::string>) representations,
  });

protected:
  LogfileLogger() {}; // only derived classes can construct this - prevents client code accidentally creating this directly

public:
  /** 
   * create a new logger instance
   */
  static LogfileLogger* init(const Configuration& config);
  virtual ~LogfileLogger() {}

  /**
   * Execute the logger for this thread.
   * @param threadName The name of this thread.
   */
  virtual void execute(const std::string& threadName) = 0;

  /**
   * get some status information that can be presented via the robot HRI
   * 
   * @param outState output parameter to receive the logger state
   * @param outBuffersToWrite output parameter to receive the number of buffers remaining to write
   */
  virtual void getStatus(State& outState, size_t& outBuffersToWrite) = 0;

protected:

  /* ----------------- separate header from streamable members (note comma at end) ------------------ */,

  (bool) enabled, /**< Is logging enabled? */
  (std::string) path, /**< The directory that will contain the log file. */
  (unsigned) numOfBuffers, /**< The number of buffers allocated. */
  (unsigned) sizeOfBuffer, /**< The size of each buffer in bytes. */
  (int) writePriority, /**< The scheduling priority of the writer thread. */
  (unsigned) minFreeDriveSpace, /**< Logging will stop if less MB are available to the target device. */
  (std::vector<RepresentationsPerThread>) representationsPerThread, /**< Representations to log per thread. */
});
