/**
 * @file Logger.cpp
 *
 * This file implements a class that writes a subset of representations into
 * log files. The representations can stem from multiple parallel threads.
 * The class maintains a buffer of message queues that can be claimed by
 * individual threads, filled with data, and given back to the logger for
 * writing them to the log file.
 * 
 * @author Thomas Röfer
 */

#include "Logger.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
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


DECL_TLOGGER(tlogger, "Logger", TextLogging::INFO);


Logger::Logger(const Configuration& config)
  : typeInfo(200000)
{
  TypeInfo::initCurrent();
  std::memset(gameInfoThreadName, 0, sizeof(gameInfoThreadName));
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
                PRINT("Logger: Thread " << rpt.thread << " should not log default representation " << defaultRepresentation);
                goto representationFound;
              }

            for(const auto& representationProvider : thread.representationProviders)
              if(loggerRepresentation == representationProvider.representation)
                goto representationFound;
            PRINT("Logger: Thread " << rpt.thread << " does not contain representation " << loggerRepresentation);
          representationFound:;
          }
          goto threadFound;
        }
      PRINT("Logger: Thread " << rpt.thread << " not found");
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
    writerThread.start(this, &Logger::writer);
  }
}

void Logger::execute(const std::string& threadName)
{
  if(!enabled)
    return;

  const bool wasLogging = logging;
  bool gameInfoThread = false;

  // check if there is a game controller active and if there is, get the
  // opponent team and set our description to the opponent and game phase
  if((!*gameInfoThreadName || threadName == gameInfoThreadName)
     && Blackboard::getInstance().exists("GameInfo"))
  {
    gameInfoThread = true;

    const GameInfo& gameInfo = static_cast<const GameInfo&>(Blackboard::getInstance()["GameInfo"]);
    const bool loggingNow = gameInfo.state != STATE_INITIAL && gameInfo.state != STATE_FINISHED;
    if(loggingNow && !*gameInfoThreadName)
    {
      std::string description = "Testing";
      if(gameInfo.packetNumber || gameInfo.secsRemaining != 0) // Packet from GameController
      {
        for(const auto& team : teamList.teams)
          if(team.number == gameInfo.opponentTeam().teamNumber)
          {
            description = team.name + "_"
                          + (gameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT ? "shootout"
                             : gameInfo.firstHalf ? "half1" : "half2");
            break;
          }
      }

      {
        SYNC;
        filename = path + LoggingTools::createName("", Global::getSettings().headName, Global::getSettings().bodyName,
                                                  Global::getSettings().scenario, Global::getSettings().location,
                                                  description, Global::getSettings().playerNumber);
        std::strncpy(gameInfoThreadName, threadName.c_str(), sizeof(gameInfoThreadName) - 1);
      }

      TLOGI(tlogger, "Logging started in thread {}, filename \"{}\"", gameInfoThreadName, filename);
    }
    logging = loggingNow;
  }

  if (logging || wasLogging)
  {
    for(const RepresentationsPerThread& rpt : representationsPerThread)
    {
      if(rpt.thread == threadName && !rpt.representations.empty())
      {
        MessageQueue* buffer = nullptr;
        {
          SYNC;
          if(!buffersAvailable.empty())
          {
            buffer = buffersAvailable.top();
            buffersAvailable.pop();
          }
        }
        if (!buffer)
        {
          OUTPUT_WARNING("Logger: No buffer available!");
          return;
        }

        if (logging)
        {
          STOPWATCH("Logger")
          {
            buffer->out.bin << threadName;
            buffer->out.finishMessage(idFrameBegin);

            for (const std::string &representation : rpt.representations)
#ifndef NDEBUG
              if (Blackboard::getInstance().exists(representation.c_str()))
#endif
              {
                buffer->out.bin << Blackboard::getInstance()[representation.c_str()];
                if (!buffer->out.finishMessage(static_cast<MessageID>(
                        TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + representation))))
                  OUTPUT_WARNING("Logger: Representation " << representation << " did not fit into buffer!");
              }
#ifndef NDEBUG
              else
                OUTPUT_WARNING("Logger: Representation " << representation << " does not exists!");
#endif

            Global::getAnnotationManager().getOut().copyAllMessages(*buffer);
          }
          Global::getTimingManager().getData().copyAllMessages(*buffer);
          buffer->out.bin << threadName;
          buffer->out.finishMessage(idFrameFinished);
        }
        else
        {
          // wasLogging must be true, so we don't add any representations
          // to the buffer and instead send an empty buffer to signal end of logging
          TLOGI(tlogger, "Signal end of logging to the writer thread with an empty buffer");
        }

        {
          SYNC;
          buffersToWrite.push_back(buffer);
        }
        framesToWrite.post();
        hasLoggedAnything = true;
        break;
      }
    }
  }

  if (gameInfoThread && !logging && hasLoggedAnything && buffersToWrite.empty())
  {
    TLOGI(tlogger, "End of log file writing detected - log file written");
    SystemCall::say("Log file written");
    hasLoggedAnything = false; // reset back to initial state
  }
}

Logger::~Logger()
{
  writerThread.announceStop();
  framesToWrite.post();
  writerThread.stop();
}

void Logger::writer()
{
  Thread::nameCurrentThread("LoggerWriter");
  BH_TRACE_INIT("LoggerWriter");

  OutBinaryFile* file = nullptr;
  std::string completeFilename;

  while(true)
  {
    framesToWrite.wait();
    if(!writerThread.isRunning()
       || (!completeFilename.empty()
           && SystemCall::getFreeDiskSpace(completeFilename.c_str()) < static_cast<unsigned long long>(minFreeDriveSpace) << 20))
      break;

    // This assumes that reading the front is threadsafe.
    MessageQueue* buffer = buffersToWrite.front();

    if (!file && !buffer->isEmpty())
    {
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

        completeFilename = fmt::format("{}roboeireannd_{:03d}.tlog", path, suffix1+1);
        InBinaryFile stream(completeFilename);
        if(!stream.exists())
          break;
      }

      // find the next free secondary suffix for the log filename
      int suffix2 = 0;
      for ( ; suffix2 < 100; ++suffix2)
      {
        completeFilename = fmt::format("{}__{:03d}_{:02d}.log", filename, suffix1, suffix2);
        InBinaryFile stream(completeFilename);
        if (!stream.exists())
          break;
      }

      if (suffix2 == 100)
      {
        OUTPUT_WARNING(
            fmt::format("Logger: Could not find an available secondary suffix for {}__{:03d}", filename, suffix1));
        break;
      }

      file = new OutBinaryFile(completeFilename);
      if(!file->exists())
      {
        OUTPUT_WARNING("Logger: File " << completeFilename << " could not be created!");
        break;
      }

      TLOGI(tlogger, "WriterThread created log file \"{}\"", completeFilename);


      *file << LoggingTools::logFileMessageIDs;
      buffer->writeMessageIDs(*file);
      *file << LoggingTools::logFileTypeInfo;
      file->write(typeInfo.data(), typeInfo.size());
      *file << LoggingTools::logFileUncompressed;
      buffer->writeAppendableHeader(*file);
    }

    if (!buffer->isEmpty())
    {
      buffer->append(*file); // write buffer contents to file
      buffer->clear();
    }
    else // buffer isEmpty which means close the file, we're done
    {
      TLOGI(tlogger, "WriterThread closed log file \"{}\"", completeFilename);
      delete file;
      file = nullptr;
    }

    {
      SYNC;
      buffersToWrite.pop_front();
      buffersAvailable.push(buffer);
    }
  }

  if (file)
    delete file;
}
