/**
 * @file TextLogging.cpp
 *
 * Implementation file for text logging (see .h file)
 *
 * @author Rudi Villing
 */

#include "TextLogging.h"

#include "Platform/Thread.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

#include <mutex>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;

// #include "fmt/compile.h"



// if the following flag is true, then the a mutex will be used to
// ensure that individual textLog messages will be output atomically.
//  If the flag is false, then textLog messages will be output best effort.
// Usually this is fine, but occasionally partial logMessages can interleave
static constexpr bool SERIALIZE_SYNC_OUTPUT = true;


// =========================================================================
// =========================================================================
// =========================================================================

class AsyncTextLogMsg
{
private:
  AsyncTextLogMsg(const AsyncTextLogMsg& other) = delete; // not copyable
  AsyncTextLogMsg& operator=(const AsyncTextLogMsg& other) = delete; // not copy assignable

public:
  fmt::memory_buffer prefixBuffer;
  fmt::memory_buffer msgBuffer;
  std::stringstream msgStream;

  AsyncTextLogMsg() {}

  AsyncTextLogMsg(fmt::memory_buffer &&prefixBuffer, MacrosOnlyTextLogMsg_ &&msg)
      : prefixBuffer(std::move(prefixBuffer)),
        msgBuffer(std::move(msg.msgFmt)), msgStream(std::move(msg.msgStream))
  {
  }

  AsyncTextLogMsg(AsyncTextLogMsg&& other) = default;
  AsyncTextLogMsg& operator=(AsyncTextLogMsg&& other) = default;
};

// =========================================================================
// =========================================================================
// =========================================================================

// simple bounded, power of 2 queue, with move semantics for enqueue and dequeue
// and no overwriting of existing elements if full

// TODO: this queue may create a bottleneck due to thread contention. If it does,
// consider a switch to a lock free queue instead.

template <typename T>
class BoundedQueue
{
public:
  BoundedQueue(size_t capacity) : mask(capacity - 1), head(0), count(0)
  {
    ASSERT((capacity >= 2) && ((capacity & (capacity-1)) == 0)); // must be a power of 2

    data.resize(capacity);
  }

  bool enqueue(T&& value)
  {
    std::lock_guard<std::mutex> lock(syncMutex);

    if (count == data.capacity()) // full?
      return false; // not enqueued

    data[(head + count) & mask] = std::move(value); // move
    ++count;
    return true;
  }

  bool dequeue(T& value)
  {
    std::lock_guard<std::mutex> lock(syncMutex);

    if (count == 0) // empty?
      return false;

    value = std::move(data[head]);
    head = (head + 1) & mask;
    --count;
    return true;
  }

  size_t size()
  {
    std::lock_guard<std::mutex> lock(syncMutex);
    return count;
  }

private:
  std::mutex syncMutex;
  const size_t mask;
  size_t head;
  size_t count;
  std::vector<T> data;
};


// =========================================================================
// =========================================================================
// =========================================================================

class TextLoggingHelper
{
public:
  using AsyncQueue = BoundedQueue<AsyncTextLogMsg>;
  using ClockType = std::chrono::steady_clock;

  // to avoid interleaving of partial log messages when in sync mode.
  // Whether it is used or not depends on the SERIALIZATION_SYNC macro definition above.
  std::mutex serializationMutex;


  // async mode 

  std::atomic_bool isAsyncMode { false }; // determines whether text log messages are logged async or sync

  AsyncQueue asyncQ { 1024 * 1 }; // queue of async text log msgs to be output in async thread - must be a power of 2 size
  std::unique_ptr<Thread> asyncThread; // manages output of async text log msgs
  

  void output(fmt::memory_buffer &prefixBuffer, fmt::memory_buffer &msgBuffer, std::stringstream &msgStream,
                     bool serializeOutput = SERIALIZE_SYNC_OUTPUT);

  // see comment on TextLogging::requestAsyncMode
  void requestAsyncOutput(bool asyncDesired);

private:
  void asyncThreadMain();
  void lockFreeWait(std::chrono::time_point<ClockType> startTime);
};


static TextLoggingHelper helper;


// =========================================================================
// =========================================================================
// =========================================================================

DECL_TLOGGER(tlogger, "TextLogging", TextLogging::INFO);


TextLogger& TextLogging::get(const std::string& name)
{
    LoggerMap& loggerMap(getLoggerMap());

    // does logger already exist?
    LoggerMap::const_iterator found = loggerMap.find(name);
    if (found == loggerMap.end())
      return registerLogger(name);
    else // existing logger with specified name was found
        return *(found->second);
}


TextLogger& TextLogging::get(const std::string& name, Level level)
{
    TextLogger& logger = get(name);
    logger.enabledLevel = level;
    return logger;
}

TextLogger& TextLogging::get()
{
    static TextLogger& logger = get("Default");
    return logger;
}



TextLogger& TextLogging::registerLogger(const std::string& name)
{
    LoggerMap& loggerMap(getLoggerMap());

    // TODO: there could be a race condition here if multiple
    // threads try to update the loggerMap simultaneously.
    // Currently this doesn't seem to be a problem, but we may need to
    // add synchronisation here

    // loggers are never deleted, so there shouldn't be any memory leaks
    // and hence no need for smart pointers.

    TextLogger* logger = new TextLogger(name);
    loggerMap[name] = logger;

    return *logger;
}


void TextLogging::macrosOnlyOutput_(TextLogger& logger, Level level, MacrosOnlyTextLogMsg_&& msg)
{
  fmt::memory_buffer prefixBuffer;

  unsigned timeMs = Time::getCurrentSystemTime();

#ifndef TARGET_TOOL

  if (Global::settingsExist())
    fmt::format_to(std::back_inserter(prefixBuffer), 
                    // FMT_COMPILE("{}.{:03d} [{}] : "), 
                    "{}.{:03d} {}/ [{}] t{}-r{} {} : ", 
                    (timeMs / 1000), (timeMs % 1000),
                    levelStrings[level],
                    Thread::getCurrentThreadCachedName(/*fullName*/ true),
                    Global::getSettings().teamNumber,
                    Global::getSettings().playerNumber,
                    logger.getName());
  else
    fmt::format_to(std::back_inserter(prefixBuffer), 
                    // FMT_COMPILE("{}.{:03d} [{}] : "), 
                    "{}.{:03d} {}/ [{}] {} : ", 
                    (timeMs / 1000), (timeMs % 1000),
                    levelStrings[level],
                    Thread::getCurrentThreadCachedName(/*fullName*/ true),
                    logger.getName());

#else // TARGET_TOOL is defined (e.g. bush)

  fmt::format_to(std::back_inserter(prefixBuffer), 
                  "{}.{:03d} {}/ {} : ", 
                  (timeMs / 1000), (timeMs % 1000),
                  levelStrings[level],
                  logger.getName());

#endif

  // actually do the output
  {
    if (!helper.isAsyncMode.load())
      helper.output(prefixBuffer, msg.msgFmt, msg.msgStream);
    else // async mode is active
      helper.asyncQ.enqueue(AsyncTextLogMsg(std::move(prefixBuffer), std::move(msg)));
  }
}

void TextLogging::requestAsyncOutput(bool asyncDesired)
{
  helper.requestAsyncOutput(asyncDesired);
}

// =========================================================================
// TextLoggingHelper & async functionality
// =========================================================================

void TextLoggingHelper::output(fmt::memory_buffer &prefixBuffer, fmt::memory_buffer &msgBuffer, std::stringstream &msgStream,
                         bool serializeOutput)
{
  FILE *outFile = stdout;

  std::unique_lock<std::mutex> optionalLocker(serializationMutex, std::defer_lock);
  if (serializeOutput)
    optionalLocker.lock(); // will unlock at end of scope


  fwrite(prefixBuffer.data(), sizeof(char), prefixBuffer.size(), outFile);

  if (msgBuffer.size())
    fwrite(msgBuffer.data(), sizeof(char), msgBuffer.size(), outFile);
  else
  {
    std::string s(msgStream.str());
    fwrite(s.data(), sizeof(char), s.size(), outFile);
  }

  fputc('\n', outFile);
}


/**
 * this uses a busy-wait/polling approach where the delay between
 * polls varies according to how long we've already been waiting
 *
 * The lack of any signalling mechanism means that we may wait
 * longer than is strictly necessary for something to happen
 */
void TextLoggingHelper::lockFreeWait(std::chrono::time_point<ClockType> startTime)
{
  auto now = ClockType::now();
  const std::chrono::microseconds elapsed {std::chrono::duration_cast<std::chrono::microseconds>(now - startTime)};

  if (elapsed < 100us)
    std::this_thread::yield();
  else if (elapsed < 20ms)
    std::this_thread::sleep_for(1ms);
  else if (elapsed < 200ms)
    std::this_thread::sleep_for(10ms);
  else
    std::this_thread::sleep_for(100ms);
}


void TextLoggingHelper::asyncThreadMain()
{
  Thread::nameCurrentThread("AsyncTextLogging");

  AsyncTextLogMsg asyncMsg;
  std::chrono::time_point<ClockType> lastDequeueTime;


  // the main thread loop for processing async messages

  while (asyncThread->isRunning())
  {    
    if (asyncQ.dequeue(asyncMsg)) // remove msg from queue using move semantics
    {
      lastDequeueTime = ClockType::now(); // we've successfully dequeued a message at this time

      // only the async thread outputs text log messages in async mode, so don't attempt to
      // serialize the output in the output function
      output(asyncMsg.prefixBuffer, asyncMsg.msgBuffer, asyncMsg.msgStream, false);
    }
    else // lock free wait for next message to arrive on the asyncQ
    {
      lockFreeWait(lastDequeueTime);
    }
  }

  // the main async processing has stopped running - so transition to sync processing
  // once isAsyncMode is false (just below), no more messages will be added to the asyncQ.
  // (There will be a short time now, while the queue is being drained, where log messages
  // may appear out of sequence. For this reason we try to make the changeover obvious)

  isAsyncMode = false;

  TLOGI(tlogger, "**************************************************************");
  TLOGI(tlogger, "***** Starting switch to synchronous text logging - draining {} queued async messages", asyncQ.size());
  TLOGI(tlogger, "**************************************************************");

  // now drain any async messages that were added
  while (asyncQ.dequeue(asyncMsg))
    output(asyncMsg.prefixBuffer, asyncMsg.msgBuffer, asyncMsg.msgStream);

  // thread exits here so that a any outstanding thread join will complete
}


/**
 * activate or deactivate asyncMode for text logging output
 * @param desiredMode true (default) - start async mode if not already started
 *                    false - resume sync mode (i.e. textlog output in the callers thread)
 *                          if not already resumed
 */
void TextLoggingHelper::requestAsyncOutput(bool asyncDesired)
{
  bool currentMode = !asyncDesired; // we assume that this function should only be called if we want to change mode

  // fprintf(stderr, "TextLoggingHelper::requestAsyncOutput %d\n", asyncDesired);

  if (asyncDesired)
  {
    if (isAsyncMode.compare_exchange_strong(currentMode, asyncDesired)) // was the value changed from sync to async?
    {
      TLOGI(tlogger, "**************************************************************");
      TLOGI(tlogger, "***** Switched to asynchronous logging mode");
      TLOGI(tlogger, "**************************************************************");

      asyncThread = std::make_unique<Thread>();
      asyncThread->setPriority(0);
      asyncThread->start(this, &TextLoggingHelper::asyncThreadMain);
    }
  }
  else // async not desired, i.e. resume sync mode if needed
  {
    // resuming sync mode after async is a rare operation, likely just happening
    // during shutdown, so for now we are not particularly worried about race
    // conditions here
    if (isAsyncMode.load()) // are we currently in async mode?
    {
      // fprintf(stderr, "TextLoggingHelper::requestAsyncOutput was in asyncMode, so stopping thread\n");

      // the async thread itself will switch the asyncMode back to sync
      asyncThread->announceStop();
      asyncThread->stop(); // gives thread 10000ms to end gracefully and join
      asyncThread.reset(); // delete the async Thread object - we'll create a new one if starting async again

      // fprintf(stderr, "TextLoggingHelper::requestAsyncOutput thread stopped\n");

      TLOGI(tlogger, "**************************************************************");
      TLOGI(tlogger, "***** Completed switch back to synchronous text logging");
      TLOGI(tlogger, "**************************************************************");

      fflush(NULL); // try to flush all streams
    }
  }
}