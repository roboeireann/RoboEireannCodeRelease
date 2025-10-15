/**
 * @file LogfileLoggerImpl.cpp
 *
 * This file implements a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 * 
 * @author Thomas Röfer
 */

#include "LogfileLogger.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/File.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Debugging/AnnotationManager.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Debugging/Stopwatch.h"
#include "Tools/Global.h"
#include "Tools/Logging/LoggingTools.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Settings.h"
#include "Tools/Streams/TypeInfo.h"

#include "Tools/TextLogging.h"

#include <cstring>

#ifdef LINUX
#include <unistd.h>
#endif


#undef PRINT
#ifndef TARGET_ROBOT
#define PRINT(message) OUTPUT_WARNING(message)
#elif defined NDEBUG
#include <cstdlib>
#define PRINT(message) \
  do \
  { \
    OUTPUT_ERROR(message); \
    abort(); \
  } \
  while(false)
#else
#define PRINT(message) FAIL(message)
#endif


DECL_TLOGGER(tlogger, "LogfileLogger", TextLogging::INFO);




// ============================================================================
// LogfileLoggerImpl declaration
// ============================================================================

class LogfileLoggerImpl : public LogfileLogger
{
private:
  /** A team number and the corresponding team name. */
  STREAMABLE(Team,
  {,
    (uint8_t)(0) number,
    (std::string) name,
  });

  /** The map from team numbers to team names. */
  STREAMABLE(TeamList,
  {,
    (std::vector<Team>) teams,
  });

  DECLARE_SYNC;
  OutBinaryMemory typeInfo; /**< Streamed type information created in main thread and used in logger thread. */
  TeamList teamList; /**< The list of all teams for naming the log file after the opponent. */
  std::vector<MessageQueue> buffers; /**< All buffers to write log data to. */
  std::stack<MessageQueue*> buffersAvailable; /**< The buffers currently available to fill with log data. */
  std::deque<MessageQueue*> buffersToWrite; /**< The buffers already filled that need to be written. */
  // char gameInfoThreadName[32]; /**< The thread that started logging and decides to stop it. */
  const GameInfo* theGameInfo = nullptr;
  std::string gameInfoThreadName;
  std::string baseFilename; /**< The base name of the log file. */

  enum State loggingState = IDLE;
  bool hasLoggedSomething = false; /**< Have we logged anything since last reset (reset when not logging and buffersToWrite is empty)? */
  bool prevBufferAvailable = true; // was there a buffer available the last time we tried to get one?

  Thread writerThread; /**< The thread that is writing the logged data to a file. */
  Semaphore framesToWrite; /**< How many frames the writer thread should write? */
  unsigned numBuffersLogged = 0; // used by writerThread only

  void updateBaseFilename(const GameInfo& gameInfo);
  bool populateBufferForWriter(const std::string& threadName);


  /** The method runs in a separate thread and writes the logged data to a file. */
  void writerThreadMain();
  bool openLogFile(OutBinaryFile* &file, std::string& generatedFilename);

public:
  /**
   * The constructor reads the configuration file and checks it against the module configuration.
   * @param config The initial module configuration.
   */
  LogfileLoggerImpl(const Configuration& config);

  /** The destructor signals the writer thread to stop. */
  ~LogfileLoggerImpl() override;

  /**
   * Execute the logger for this thread.
   * @param threadName The name of this thread.
   */
  void execute(const std::string& threadName) override;

  /**
   * get some status information that can be presented via the robot HRI
   * 
   * @param outState output parameter to receive the logger state
   * @param outBuffersToWrite output parameter to receive the number of buffers remaining to write
   */
  void getStatus(State& outState, size_t& outBuffersToWrite) override;
};


// ============================================================================
// LogfileLogger implementation
// ============================================================================

LogfileLogger* LogfileLogger::init(const Configuration& config)
{
  return new LogfileLoggerImpl(config);
}



// ============================================================================
// LogfileLoggerImpl implementation
// ============================================================================

LogfileLoggerImpl::LogfileLoggerImpl(const Configuration& config)
  : typeInfo(200000)
{
  TypeInfo::initCurrent();
  // std::memset(gameInfoThreadName, 0, sizeof(gameInfoThreadName));
  InMapFile stream("logger.cfg");
  ASSERT(stream.exists());
  stream >> *this;
  if(!path.empty() && path.back() != '/')
    path += "/";

  // Report wrong logger configuration (even in the simulator).
  // This check is only executed for the initial configuration, because on the robot that is usually the one that is used.
  if(enabled)
    for(const auto& rpt : representationsPerThread)
    {
      for(const auto& thread : config.threads)
        if(thread.name == rpt.thread)
        {
          for(const std::string& loggerRepresentation : rpt.representations)
          {
            for(const std::string& defaultRepresentation : config.defaultRepresentations)
              if(loggerRepresentation == defaultRepresentation)
              {
                PRINT("LogfileLogger: Thread " << rpt.thread << " should not log default representation " << defaultRepresentation);
                goto representationFound;
              }

            for(const auto& representationProvider : thread.representationProviders)
              if(loggerRepresentation == representationProvider.representation)
                goto representationFound;
            PRINT("LogfileLogger: Thread " << rpt.thread << " does not contain representation " << loggerRepresentation);
          representationFound:;
          }
          goto threadFound;
        }
      PRINT("LogfileLogger: Thread " << rpt.thread << " not found");
    threadFound:;
    }

#ifndef TARGET_ROBOT
  enabled = false;
  path = "Logs/";
#endif

  if(enabled)
  {
    typeInfo << *TypeInfo::current;
    InMapFile stream("teamList.cfg");
    if(stream.exists())
      stream >> teamList;

    buffers.resize(numOfBuffers);
    for(MessageQueue& buffer : buffers)
    {
      buffer.setSize(sizeOfBuffer);
      buffersAvailable.push(&buffer);
    }

    writerThread.setPriority(writePriority);
    writerThread.start(this, &LogfileLoggerImpl::writerThreadMain);
  }
}


LogfileLoggerImpl::~LogfileLoggerImpl()
{
  writerThread.announceStop();
  framesToWrite.post();
  writerThread.stop();
}


void LogfileLoggerImpl::getStatus(State& outState, size_t& outBuffersToWrite)
{
  SYNC;
  outState = loggingState;
  outBuffersToWrite = buffersToWrite.size();
}

// There is one LogfileLoggerImpl (Robot::LogfileLoggerImpl) that is shared by all module container threads
// (e.g. Cognition, Motion, etc).
// Each module container thread calls execute once per cycle after it has
// executed its modules
void LogfileLoggerImpl::execute(const std::string& threadName)
{
  if(!enabled)
    return;

  // THREAD SAFETY NOTE: only the gameInfoThread can change the loggingState, so safe to read it from
  // the gameInfoThread without a SYNC block. However, we need a SYNC block for writes, or reads outside 
  // the gameInfoThread.
  //
  // Other data that may be modified by different threads and hence that needs SYNC access includes:
  // - hasLoggedSomething
  // - buffersAvailable, buffersToWrite
  // - prevBufferAvailable

  {
    SYNC;
  
    // have we identified the gameInfoThread yet?
    if (gameInfoThreadName.empty() && Blackboard::getInstance().exists("GameInfo"))
    {
      gameInfoThreadName = threadName;
      theGameInfo = static_cast<const GameInfo *>(&Blackboard::getInstance()["GameInfo"]);
    }
  }
  const bool gameInfoThread = (threadName == gameInfoThreadName);

  if (gameInfoThread)
  {
    const GameInfo& gameInfo = *theGameInfo;    
    const bool shouldLog = (gameInfo.state != STATE_INITIAL && gameInfo.state != STATE_FINISHED);


    // do we need to start or stop logging
    switch (loggingState)
    {
    case IDLE:
      if (shouldLog)
      {
        // set the logfile name based on opponent team and game phase (if a game controller is running)
        updateBaseFilename(gameInfo);

        TLOGI(tlogger, "Logging initialised by thread {} with base filename \"{}\"", threadName, baseFilename);

        {
          SYNC;
          loggingState = LOGGING;
        }
      }
      return; // IDLE and nothing more to do on this cycle

    case LOGGING:
      if (!shouldLog)
      {
        // gracefully finish logging
        {
          SYNC;
          if (hasLoggedSomething)
            loggingState = FINISH_LOGGING;
          else
            loggingState = IDLE; // we haven't logged anything yet so no need to stop logging

          buffersToWrite.push_back(nullptr);
        }
        // signal the writerThread without adding a buffer to indicate the end
        // of the log file
        TLOGD(tlogger, "Signal end of logging to the writer thread by not adding a buffer");
        framesToWrite.post();
      }
      break;

    case FINISH_LOGGING:
      {
        bool finishedLogging = false;
        {
          SYNC;
          finishedLogging = buffersToWrite.empty();
          if (finishedLogging)
          {
            loggingState = IDLE;
            hasLoggedSomething = false;
          }
        }

        if (finishedLogging)
        {
          TLOGI(tlogger, "End of log file writing detected - log file written");
          SystemCall::say("Log file written");
        }
      }
      return;

    default:
      ASSERT(false); // we should never be in any other state here
    }
  }

  // execute whether gameInfoThread or not
  if (loggingState == LOGGING) // we can check without a SYNC block since it needs to be checked again inside populate...
    populateBufferForWriter(threadName);
}


bool LogfileLoggerImpl::populateBufferForWriter(const std::string& threadName)
{
  for (const RepresentationsPerThread &rpt : representationsPerThread)
  {
    if (rpt.thread != threadName)
      continue;
    else if (rpt.representations.empty())
      return false;
    else // correct thread and it has representations to log
    {
      bool prevBufferAvailableCached;

      MessageQueue* buffer = nullptr;
      {
        SYNC;
        if(!buffersAvailable.empty())
        {
          buffer = buffersAvailable.top();
          buffersAvailable.pop();
        }

        prevBufferAvailableCached = prevBufferAvailable;
        prevBufferAvailable = (buffer != nullptr);
      }

      if (!buffer)
      {
        if (prevBufferAvailableCached)
          OUTPUT_WARNING("LogfileLogger: No buffer available!");

        return false;
      } 
      else if (!prevBufferAvailableCached)
      {
        OUTPUT_WARNING("LogfileLogger: buffer available again");
      }

      STOPWATCH("LogfileLogger")
      {
        buffer->out.bin << threadName;
        buffer->out.finishMessage(idFrameBegin);

        for (const std::string &representation : rpt.representations)
        {
#ifndef NDEBUG
          if (!Blackboard::getInstance().exists(representation.c_str()))
            OUTPUT_WARNING("LogfileLogger: Representation " << representation << " does not exist!");
          else
#endif
          {
            buffer->out.bin << Blackboard::getInstance()[representation.c_str()];
            if (!buffer->out.finishMessage(static_cast<MessageID>(
                    TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + representation))))
              OUTPUT_WARNING("LogfileLogger: Representation " << representation << " did not fit into buffer!");
          }
        }

        Global::getAnnotationManager().getOut().copyAllMessages(*buffer);
      }

      Global::getTimingManager().getData().copyAllMessages(*buffer);
      buffer->out.bin << threadName;
      buffer->out.finishMessage(idFrameFinished);

      // the following code looks a little complex due to potential race conditions and
      // logging state potentially changing (in another thread) between when we start the 
      // function and get this far
      bool postedBuffer = false;
      bool firstBuffer = false;
      {
        SYNC;
        if (loggingState == LOGGING)
        {
          buffersToWrite.push_back(buffer);
          postedBuffer = true;
          if (!hasLoggedSomething)
          {
            hasLoggedSomething = true;
            firstBuffer = true;
          }
        }
        else // not LOGGING - must have changed since we entered the function...
          buffersAvailable.push(buffer); // ..so mark buffer available again, don't post to the writerThread
      }

      if (firstBuffer)
        TLOGD(tlogger, "Logging started with first buffer");

      if (postedBuffer)
      {
        framesToWrite.post();        
        return true; // we posted a buffer
      }
      else
        return false;
    }
  }
  return false;
}

void LogfileLoggerImpl::updateBaseFilename(const GameInfo& gameInfo)
{
  std::string description = "Testing";
  if (gameInfo.packetNumber || gameInfo.secsRemaining != 0) // Packet from GameController
  {
    // find opponent team to create description
    for (const auto &team : teamList.teams)
      if (team.number == gameInfo.opponentTeam().teamNumber)
      {
        description = team.name + "_"
                      + (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT ? "shootout"
                          : gameInfo.firstHalf ? "half1" : "half2");
        break;
      }
  }

  {
    SYNC;
    baseFilename = path + LoggingTools::createName("", Global::getSettings().headName, Global::getSettings().bodyName,
                                              Global::getSettings().scenario, Global::getSettings().location,
                                              description, Global::getSettings().playerNumber);
  }

}

// ============================================================================
// LogfileLoggerImpl writerThread
// ============================================================================

void LogfileLoggerImpl::writerThreadMain()
{
  Thread::nameCurrentThread("LogfileWriter");
  BH_TRACE_INIT("LogfileWriter");
  TLOGI(tlogger, "*** writerThread started");

  OutBinaryFile* logfile = nullptr;
  std::string completeFilename;
  bool canLogToFile = true;

  // the thread loop for the writerThread
  while (true)
  {
    // wait for a semaphore signal indicating (a) a new buffer to write, (b) no buffer indicating logging is done,
    // or (c) the thread is stopped
    framesToWrite.wait();

    TLOGV(tlogger, "writerThreadMain: sem signalled, thread running = {}", writerThread.isRunning());

    // check to see if the thread is still running
    if (!writerThread.isRunning())
      break; // this will exit the thread loop and begin exiting the thread

    // check if we have enough disk space to keep logging
    if (!completeFilename.empty() && 
        (SystemCall::getFreeDiskSpace(completeFilename.c_str()) < static_cast<unsigned long long>(minFreeDriveSpace) << 20))
      canLogToFile = false;

    // Get the next buffer to write.
    // (There must be one since framesToWrite was signalled.)
    MessageQueue* buffer = nullptr;
    {
      SYNC;
      buffer = buffersToWrite.front();
    }

    // do we have a buffer to write?
    if (buffer)
    {
      if (canLogToFile)
      {
        // if we do not have a log file yet, we need to open one first
        if (!logfile)
        {
          if (!openLogFile(logfile, completeFilename))
            canLogToFile = false; // disable logging for now
          else
          {
            TLOGI(tlogger, "writerThreadMain: opened log file \"{}\"", completeFilename);

            *logfile << LoggingTools::logFileMessageIDs;
            buffer->writeMessageIDs(*logfile);
            *logfile << LoggingTools::logFileTypeInfo;
            logfile->write(typeInfo.data(), typeInfo.size());
            *logfile << LoggingTools::logFileUncompressed;
            buffer->writeAppendableHeader(*logfile);

            numBuffersLogged = 0;
          }
        }

        if (logfile && canLogToFile) // we need to recheck since we might not have successfully opened a logfile
        {
          // now write the buffer contents to the logfile
          buffer->append(*logfile);
          buffer->clear();

          // Thread::sleep(10); // FOR TESTING ONLY: slow down the writer thread temporarily so that buffersToWrite increases

          ++numBuffersLogged;

          TLOGD(tlogger, "logged buffer {}", numBuffersLogged);
        }
      }
    }
    else // null buffer means we're finished logging for now (E.g. half time in game), so close this logfile
    {
      if (logfile)
      {
#ifdef LINUX
      ::fsync(::fileno(static_cast<FILE*>(logfile->getFile()->getNativeFile()))); // sync the file to storage
#endif
        delete logfile;
        logfile = nullptr;
        TLOGI(tlogger, "WriterThread got null buffer, logging ended and closed log file \"{}\"", completeFilename);
      }
      else
        TLOGI(tlogger, "WriterThread got null buffer, logging ended (no open file to close)");
    }


    // regardless of whether we could append the buffer to the logfile or not, recover it for reuse
    size_t numBuffersRemaining;
    {
      SYNC;
      buffersToWrite.pop_front();
      numBuffersRemaining = buffersToWrite.size();

      if (buffer)
        buffersAvailable.push(buffer);
    }

    TLOGD(tlogger, "There are now {} buffersToWrite", numBuffersRemaining);
  }

  if (logfile)
    delete logfile;


  TLOGI(tlogger, "*** writerThread ended");
}



bool LogfileLoggerImpl::openLogFile(OutBinaryFile* &file, std::string& generatedFilename)
{
  ASSERT(!file); // file must be nullptr initially

  // @author Rudi Villing
  // We only write the new RoboEireann style log file names now. It won't
  // duplicate old style names and any old style names clearly refer
  // to older log files so there shouldn't be any ambiguity

  // we base part of the number on the suffix (nnn) to roboeireannd_nnn.tlog so that
  // we can easily match text logs to corresponding log files

  // find the current primary suffix for the log filename
  int suffix1 = 0;
  for ( ; suffix1 < 300; ++suffix1)
  {
    // completeFilename = filename + (i ? "_(" + ((i < 10 ? "0" : "") + std::to_string(i)) + ")" : "") + ".log";

    std::string completeFilename = fmt::format("{}roboeireannd_{:03d}.tlog", path, suffix1 + 1);
    InBinaryFile stream(completeFilename);
    if(!stream.exists())
      break;
  }

  if (suffix1 == 300)
  {
    OUTPUT_WARNING(
        fmt::format("LogfileLogger: Could not find an available suffix for {}roboeireannd", path));
    return false;
  }

  // find the next free secondary suffix for the log filename
  int suffix2 = 0;
  for ( ; suffix2 < 100; ++suffix2)
  {
    generatedFilename = fmt::format("{}__{:03d}_{:02d}.log", baseFilename, suffix1, suffix2);
    InBinaryFile stream(generatedFilename);
    if (!stream.exists())
      break;
  }

  if (suffix2 == 100)
  {
    OUTPUT_WARNING(
        fmt::format("LogfileLogger: Could not find an available secondary suffix for {}__{:03d}", baseFilename, suffix1));
    return false;
  }

  file = new OutBinaryFile(generatedFilename);
  if(!file->exists())
  {
    OUTPUT_WARNING("LogfileLogger: File " << generatedFilename << " could not be created!");
    delete file;
    file = nullptr;
    return false;
  }

  return true;
}
