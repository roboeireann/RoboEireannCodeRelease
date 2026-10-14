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
#include "Debugging/AnnotationManager.h"
#include "Debugging/Debugging.h"
#include "Debugging/Stopwatch.h"
#include "Framework/Blackboard.h"
#include "Framework/LoggingTools.h"
#include "Framework/Settings.h"
#include "Platform/BHAssert.h"
#include "Platform/File.h"
#include "Platform/SystemCall.h"
#include "Streaming/Global.h"
#include "Streaming/TypeInfo.h"
#include "TextLogging/TextLogging.h"

#include <cstdio>
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

DECL_TLOGGER(tlogger, "Logger", TextLogging::INFO);

Logger::Logger(const Configuration& config) :
  typeInfo(200000),
  settings(200)
{
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
            if(loggablePerThread.end() == std::find(loggablePerThread.begin(), loggablePerThread.end(), loggerRepresentation))
              PRINT("Logger: Thread " << rpt.thread << " does not contain representation " << loggerRepresentation);
          representationFound:
            if(TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + loggerRepresentation) == -1)
              PRINT("Logger: Representation " << loggerRepresentation << " does not have a message id");
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
    TypeInfo::initCurrent();
    typeInfo << *TypeInfo::current;
    LoggingTools::writeSettings(settings, Global::getSettings());

    buffers.resize(numOfBuffers);
    for(MessageQueue& buffer : buffers)
    {
      buffer.reserve(sizeOfBuffer);
      buffersAvailable.push(&buffer);
    }

    writerThread.setPriority(writePriority);
    writerThread.start(this, &Logger::writer);
  }
}

void Logger::update(const LoggingController& controller)
{
  if(!enabled)
    return;

  const bool shouldLog = controller.shouldLog(logging.load(std::memory_order_relaxed));
  if(shouldLog != logging.load(std::memory_order_relaxed))
  {
    if(shouldLog)
    {
      TLOGI(tlogger, "Prepare to start logging to log file");

      // Tell the writer thread to start a new logging period.
      SYNC;
      if(!buffersAvailable.empty())
      {
        MessageQueue* buffer = buffersAvailable.top();
        buffersAvailable.pop();
        buffer->bin(undefined) << (path + LoggingTools::createName(Global::getSettings().headName, Global::getSettings().bodyName,
                                                                   Global::getSettings().scenario, Global::getSettings().location,
                                                                   controller.getDescription(), Global::getSettings().playerNumber));
        buffersToWrite.push_back(buffer);
      }
      else
        return; // We couldn't start the writer, so \c logging stays false.
    }
    else
    {
      TLOGI(tlogger, "Tell writer thread that logging should end");

      // Tell the writer thread that this logging period has ended.
      SYNC;
      buffersToWrite.push_back(nullptr);
    }
    framesToWrite.post();
    logging.store(shouldLog, std::memory_order_relaxed);
  }
}

bool Logger::execute(const std::string& threadName)
{
  if(!enabled)
    return true; // true because if the logger is not enabled, it doesn't make sense to keep data for later.

  // If not logging, stop here.
  if(!logging.load(std::memory_order_relaxed))
    return false;

  bool bufferAvailabilityChanged = false;
  for(const RepresentationsPerThread& rpt : representationsPerThread)
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
        const bool bufferIsAvailable = buffer != nullptr;
        bufferAvailabilityChanged = bufferWasAvailable != bufferIsAvailable;
        bufferWasAvailable = bufferIsAvailable;
      }
      if(!buffer)
      {
        if(bufferAvailabilityChanged)
          OUTPUT_WARNING("Logger: No buffer available!");
        return false;
      }
      else if(bufferAvailabilityChanged)
        OUTPUT_WARNING("Logger: Buffer available again!");

      STOPWATCH("Logger")
      {
        buffer->bin(idFrameBegin) << threadName;

        for(const std::string& representation : rpt.representations)
#ifndef NDEBUG
          if(Blackboard::getInstance().exists(representation.c_str()))
#endif
          {
            MessageQueue::OutBinary stream = buffer->bin(static_cast<MessageID>(TypeRegistry::getEnumValue(typeid(MessageID).name(), "id" + representation)));
            stream << Blackboard::getInstance()[representation.c_str()];
            if(stream.failed())
              OUTPUT_WARNING("Logger: Representation " << representation << " did not fit into buffer!");
          }
#ifndef NDEBUG
          else
            OUTPUT_WARNING("Logger: Representation " << representation << " does not exist!");
#endif

        *buffer << Global::getAnnotationManager().getOut();
      }
      *buffer << Global::getTimingManager().getData();
      buffer->bin(idFrameFinished) << threadName;
      {
        SYNC;
        buffersToWrite.push_back(buffer);
        bufferWasAvailable = true;
      }
      framesToWrite.post();
      break;
    }

  return true;
}

Logger::~Logger()
{
  writerThread.announceStop();
  framesToWrite.post();
  writerThread.stop();
}

void Logger::writer()
{
  Thread::nameCurrentThread("Logger");
  BH_TRACE_INIT("Logger");

  OutBinaryFile* file = nullptr;
  MessageQueue* buffer = nullptr;
  std::string filename, completeFilename;

  while(true)
  {
    // Wait for new data to log to arrive.
    framesToWrite.wait();

    // Terminate thread if it is told so or there is no disk space left.
    if(!writerThread.isRunning()
       || (!completeFilename.empty()
           && SystemCall::getFreeDiskSpace(completeFilename.c_str()) < static_cast<unsigned long long>(minFreeDriveSpace) << 20))
      break;

    // Get next buffer to write (there must be one).
    {
      SYNC;
      buffer = buffersToWrite.front();
    }

    if(!buffer)
    {
      // Sync the current file to disk and close it.
      ASSERT(file);
#ifdef LINUX
      ::fsync(::fileno(static_cast<FILE*>(file->getFile()->getNativeFile())));
#endif
      delete file;
      file = nullptr;
      SystemCall::say("Log file written");
    }
    else if(++buffer->begin() == buffer->end())
    {
      // All "real" buffers have at least two messages (idFrameBegin and idFrameFinished).

      // Read filename from message queue.
      (*buffer->begin()).bin() >> filename;
      buffer->clear();

      // find next free log filename
      // for(int i = 0; i < 100; ++i)
      // {
      //   completeFilename = filename + (i ? "_(" + ((i < 10 ? "0" : "") + std::to_string(i)) + ")" : "") + ".log";
      //   InBinaryFile stream(completeFilename);
      //   if(!stream.exists())
      //     break;
      // }

      // find the next free log filename
      auto findFreeLogFilename = [&]() -> bool
      {
        // We only write the new RoboEireann style log file names now. These won't
        // duplicate old style names and any old style names clearly refer
        // to older log files so there shouldn't be any ambiguity

        // we base part of the number (suffix1) on the nnn suffix to roboeireannd_nnn.tlog so that
        // we can easily match text logs to corresponding log files

        // find the current primary suffix for the log filename
        int suffix1 = 0;
        for (; suffix1 < 300; ++suffix1)
        {
          std::string tlogName = fmt::format("{}roboeireannd_{:03d}.tlog", path, suffix1 + 1);
          if (!File::exists(tlogName)) // have we found an unused suffix1?
            break;
        }

        if (suffix1 == 300)
        {
          OUTPUT_WARNING(fmt::format("Logger: Could not find the last used {}roboeireannd suffix", path));
          return false;
        }
        else
        {
          TLOGD(tlogger, "writer: found available suffix1 {}", suffix1);
        }

        // find the next free secondary suffix for the log filename
        int suffix2 = 0;
        for (; suffix2 < 100; ++suffix2)
        {
          completeFilename = fmt::format("{}__{:03d}_{:02d}.log", filename, suffix1, suffix2);
          if (!File::exists(completeFilename)) // have we found an available suffix2
            break;
        }

        if (suffix2 == 100)
        {
          OUTPUT_WARNING(
              fmt::format("Logger: Could not find an available secondary suffix for {}__{:03d}", filename, suffix1));
          return false;
        }
        else
        {
          TLOGD(tlogger, "writer: found available suffix2 {}", suffix2);
        }

        return true;
      };

      ASSERT(!file);
      if (findFreeLogFilename())
        file = new OutBinaryFile(completeFilename);

      if (!file || !file->exists())
      {
        OUTPUT_WARNING("Logger: File " << completeFilename << " could not be created!");
        break;
      }

      // On the robot, we want this to end up in our roboeireannd tlog file.
      TLOGI(tlogger, "writer: Start logging to \"{}\"", completeFilename);
      if (sayWhenLoggingStarts)
        SystemCall::say("logging started");

      *file << LoggingTools::logFileSettings;
      file->write(settings.data(), settings.size());
      *file << LoggingTools::logFileMessageIDs << static_cast<unsigned char>(numOfMessageIDs);
      FOREACH_ENUM(MessageID, i, numOfMessageIDs)
        *file << TypeRegistry::getEnumName(i);
      *file << LoggingTools::logFileTypeInfo;
      file->write(typeInfo.data(), typeInfo.size());
      *file << LoggingTools::logFileUncompressed << -1 << -1;

      // Turn off userspace buffering.
      std::setvbuf(static_cast<std::FILE*>(file->getFile()->getNativeFile()), nullptr, _IONBF, 0);
    }
    else
    {
      // Write buffered frame to file.
      if(file)
        buffer->append(*file);
      buffer->clear();
    }

    // Return the buffer.
    {
      SYNC;
      buffersToWrite.pop_front();
      if(buffer)
        buffersAvailable.push(buffer);
    }
  }

  // Delete file before thread ends.
  delete file;
}
