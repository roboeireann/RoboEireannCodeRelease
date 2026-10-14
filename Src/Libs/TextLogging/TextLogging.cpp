/**
 * @file TextLogging.cpp
 *
 * Implementation file for text logging (see .h files)
 *
 * @author Rudi Villing
 */

#include "TextLogging.h"



#include "Platform/Thread.h"
#include "Platform/Time.h"

#include <fmt/compile.h>
#include <fmt/chrono.h>

#include <mutex>
#include <chrono>
#include <atomic>
#include <condition_variable>




using namespace std::chrono_literals;



// if the following flag is true, then a mutex will be used to
// ensure that individual textLog messages will be output atomically.
// There is a small performance cost if multiple threads are outputting many text logs.
//  If the flag is false, then textLog messages will be output best effort.
// Sometimes this is fine, but occasionally partial logMessages can interleave
// when multiple threads outputting many text logs.
//  For now, we choose serialized output as the least worst option.
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
  fmt::memory_buffer buffer;

  AsyncTextLogMsg() {}

  AsyncTextLogMsg(fmt::memory_buffer &&inputBuf)
      : buffer(std::move(inputBuf))
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

// Note: The queue requires external thread sync e.g. with a mutex lock.
// It may create a bottleneck due to thread contention. If it does,
// consider a switch to a lock free queue instead, but that will involve rewriting
// other pieces also.

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
    if (count == data.capacity()) // full?
      return false; // not enqueued

    data[(head + count) & mask] = std::move(value); // move
    ++count;
    return true;
  }

  bool dequeue(T& value)
  {
    if (count == 0) // empty?
      return false;

    value = std::move(data[head]);
    head = (head + 1) & mask;
    --count;
    return true;
  }

  bool isEmpty() const { return count == 0; }
  bool isFull() const { return count == data.capacity(); }

  size_t size()
  {
    return count;
  }


private:
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
  std::atomic<size_t> droppedLogsCount{0};
  std::mutex queueMutex;
  

  /**
   * call this to modify the output file for text log output
   * (This is mainly useful for unit testing where you may want to direct output to
   * a specific file -- otherwise leaving the default is usually appropriate)
   */
  void setOutputFile(FILE *f)
  {
    ASSERT(outFile);
    outFile = f;
  }


  // async mode 

  std::atomic_bool isAsyncMode { false }; // determines whether text log messages are logged async or sync

  AsyncQueue asyncQ { 1024 * 1 }; // queue of async text log msgs to be output in async thread - must be a power of 2 size
  std::unique_ptr<Thread> asyncThread; // manages output of async text log msgs

  void output(fmt::memory_buffer &buf, bool serializeOutput = SERIALIZE_SYNC_OUTPUT);

  // see comment on TextLogging::requestAsyncMode
  void requestAsyncOutput(bool asyncDesired);


private:
  FILE* outFile = stderr;

  // std::atomic<bool> isLoggingEnabled{true};
  std::atomic<std::thread::id> lockOwner{std::thread::id()};

  void asyncThreadMain();
  void lockFreeWait(std::chrono::time_point<ClockType> startTime);
  void writeToFile(const char* data, size_t size, FILE* outf);
  void flushQueueInternal(); // must lock queueMutex first
};


static TextLoggingHelper helper;


// =========================================================================
// =========================================================================
// =========================================================================

DECL_TLOGGER(tlogger, "TextLogging", TextLogging::INFO);

// The following flag is used to disable logging if a disk full file output
// error is encountered
std::atomic<bool> TextLogging::isLoggingEnabled{true};


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


void TextLogging::logStartupTime(TextLogger& tloggerClient, Level level)
{
  auto now = std::chrono::system_clock::now();

  TLOG_IMPL(tloggerClient, level, "----------------------------------------------------------------------------");
  TLOG_IMPL(tloggerClient, level, ">>>>>      Startup time is: {:%F %T}       <<<<<", now);
  TLOG_IMPL(tloggerClient, level, "----------------------------------------------------------------------------");
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

inline void TextLogging::formatHeader(fmt::memory_buffer &buf, TextLogger &logger, Level level)
{
  unsigned timeMs = Time::getCurrentSystemTime();

  if (robotContext.teamNumber != -1)
  {
    // static constexpr auto fmtSpec = FMT_COMPILE("{}.{:03d} {}/ [{}] t{}-r{} {} : ");
    fmt::format_to(std::back_inserter(buf),
                   // fmtSpec,
                   "{}.{:03d} {}/ [{}] t{}-r{} {} : ", (timeMs / 1000), (timeMs % 1000), levelStrings[level],
                   Thread::getCurrentThreadCachedName(/*fullName*/ true), robotContext.teamNumber,
                   robotContext.playerNumber, logger.getName());
  }
  else
  {
    // static constexpr auto fmtSpec = FMT_COMPILE("{}.{:03d} {}/ [{}] {} : ");
    fmt::format_to(std::back_inserter(buf),
                   // fmtSpec,
                   "{}.{:03d} {}/ [{}] {} : ", (timeMs / 1000), (timeMs % 1000), levelStrings[level],
                   Thread::getCurrentThreadCachedName(/*fullName*/ true), logger.getName());
  }
}

inline void TextLogging::dispatchAsync(fmt::memory_buffer &&finalBuffer)
{
  if (helper.isAsyncMode.load(std::memory_order_relaxed))
  {
    bool enqueued = false;
    {
      std::lock_guard<std::mutex> lock(helper.queueMutex);
      enqueued = helper.asyncQ.enqueue(AsyncTextLogMsg(std::move(finalBuffer)));
    }

    if (!enqueued)
      helper.droppedLogsCount.fetch_add(1, std::memory_order_relaxed);
  }
  else // Fallback to sync output if async mode is disabled
  {
    helper.output(finalBuffer, true);
  }
}

void TextLogging::doLogMsgPacked(TextLogger &logger, Level level, fmt::string_view format, fmt::format_args args)
{
  fmt::memory_buffer finalBuffer;
  formatHeader(finalBuffer, logger, level); // Likely inlined by compiler
  fmt::vformat_to(std::back_inserter(finalBuffer), format, args);
  finalBuffer.push_back('\n');

  dispatchAsync(std::move(finalBuffer));
}

// Called by TLOGI_STREAM
void TextLogging::doLogMsgStream_(TextLogger &logger, Level level, fmt::string_view msg)
{
  fmt::memory_buffer finalBuffer;
  formatHeader(finalBuffer, logger, level); // Compiler inlines this
  finalBuffer.append(msg.data(), msg.data() + msg.size());
  finalBuffer.push_back('\n');

  dispatchAsync(std::move(finalBuffer)); // Compiler inlines this
}

void TextLogging::requestAsyncOutput(bool asyncDesired)
{
  helper.requestAsyncOutput(asyncDesired);
}

void TextLogging::setOutputFile(FILE* f)
{
  helper.setOutputFile(f);
}

// =========================================================================
// TextLoggingHelper & async functionality
// =========================================================================

void TextLoggingHelper::output(fmt::memory_buffer &buffer,
                         bool serializeOutput)
{
  if (!TextLogging::isLoggingEnabled.load(std::memory_order_relaxed))
    return;
    
  // for async output, only one thread is writing so no need to serialize
  // -- this is the fast path
  if (!serializeOutput)
  {
    writeToFile(buffer.data(), buffer.size(), outFile);
  }
  else // serializeOutput is true
  {
    std::unique_lock<std::mutex> lock(serializationMutex, std::defer_lock);
    if (!lock.try_lock())
    {
      // Crash/Self-deadlock detection
      if (lockOwner.load(std::memory_order_relaxed) == std::this_thread::get_id())
      {
        writeToFile(buffer.data(), buffer.size(), stderr);
        return;
      }
      lock.lock(); 
    }
    
    // track the thread with the mutex
    lockOwner.store(std::this_thread::get_id(), std::memory_order_relaxed);

    // Handle dropped log warnings
    // NOTE: because we only track the messages were dropped and not where in the
    // sequence of messages they were dropped (for async messages), you should not
    // assume that dropped messages should have appeared before the current message 
    // being output
    size_t dropped = droppedLogsCount.exchange(0, std::memory_order_relaxed);
    if (dropped > 0)
    {
      fmt::memory_buffer warnBuf;
      fmt::format_to(std::back_inserter(warnBuf), "\n--- WARNING: DROPPED {} LOG MESSAGES ---\n", dropped);
      writeToFile(warnBuf.data(), warnBuf.size(), outFile);
    }

    // Handle flushing of the async queue only if we are in sync mode.
    // This ensures that when we switch from async to sync, the very first
    // sync message "pushes" the remainder of the async queue out.
    //
    // NOTE: This might take a while if there are a lot of async messages queued up,
    // but we are willing to tolerate this in a shutdown/crash situation
    if (!isAsyncMode.load(std::memory_order_relaxed))
    {
      std::lock_guard<std::mutex> queueLock(queueMutex);
      flushQueueInternal();
    }

    // finally write the current message to output
    writeToFile(buffer.data(), buffer.size(), outFile);

    lockOwner.store(std::thread::id(), std::memory_order_relaxed);    
  }
}

void TextLoggingHelper::writeToFile(const char* data, size_t size, FILE* outf)
{
  if (size == 0)
    return;

  if (fwrite(data, 1, size, outf) < size)
  {
    if (errno == ENOSPC) 
    {
      TextLogging::isLoggingEnabled.store(false, std::memory_order_relaxed);

      // this might fail also, but we've got to try
      const char* msg = "\n--- DISK FULL!!! ---\n";
      fwrite(msg, 1, strlen(msg), stderr);
    }
  }
}

// WARNING: must be called while the queueMutex is held for save access to asyncQ
void TextLoggingHelper::flushQueueInternal()
{
  AsyncTextLogMsg asyncMsg;

  // Drain the queue to ensure chronological order
  while (asyncQ.dequeue(asyncMsg))
    writeToFile(asyncMsg.buffer.data(), asyncMsg.buffer.size(), outFile);
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

  // if (elapsed < 100us)
  //   std::this_thread::yield();
  if (elapsed < 20ms)
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
    AsyncTextLogMsg asyncMsg;
    bool msgValid = false;
    {
      std::unique_lock<std::mutex> lock(queueMutex);
      msgValid = asyncQ.dequeue(asyncMsg);
    }

    if (msgValid)
    {
      lastDequeueTime = ClockType::now(); // we've successfully dequeued a message at this time
      // Async thread writes without serialization because it is the only writer in this mode
      output(asyncMsg.buffer, false);
    }
    else // lock free wait for next message to arrive on the asyncQ
    {
      lockFreeWait(lastDequeueTime);
    }
  }

  // Thread exiting: final drain happens in requestAsyncOutput or subsequent sync calls
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
    // if (isAsyncMode.load()) // are we currently in async mode?
    if (isAsyncMode.exchange(false)) // Flip to sync mode immediately
    {
      // fprintf(stderr, "TextLoggingHelper::requestAsyncOutput was in asyncMode, so stopping thread\n");

      if (asyncThread)
      {
        // the async thread itself will switch the asyncMode back to sync
        asyncThread->announceStop();
        asyncThread->stop(); // gives thread 10000ms to end gracefully and join
        asyncThread.reset(); // delete the async Thread object - we'll create a new one if starting async again
      }

      // This call triggers the barrier logic in output() to drain the queue 
      // synchronously before finishing the mode switch. It works whether tlogger
      // is enabled at the INFO level or not, which is why we don't just rely on the
      // TLOGI statements which follow it.

      fmt::memory_buffer empty;
      output(empty, true);

      TLOGI(tlogger, "**************************************************************");
      TLOGI(tlogger, "***** Completed switch back to synchronous text logging");
      TLOGI(tlogger, "**************************************************************");

      fflush(NULL); // try to flush all streams
    }
  }
}