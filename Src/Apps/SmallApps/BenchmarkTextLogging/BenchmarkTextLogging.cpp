/**
 * @file BenchmarkTextLogging.cpp
 *
 * USAGE & COMMAND LINE OPTIONS
 *
 * This benchmark evaluates the performance overhead of various text logging schemes.
 * * Execution:
 *   ./BenchmarkTextLogging [Options]
 * * Options:
 *   --varied : Cycles through 64 different LogContexts (timestamps, levels, thread names).
 *              Use this to measure the impact of the CPU Branch Predictor. If ns/op increases
 *              significantly vs. static data, the logging logic contains too many branches.
 *   --cold   : Forces a 64KB cache-pollution task between every log statement.
 *              Use this to simulate "Real World" robot performance where logging code and
 *              format strings are likely evicted from L1/L2 caches by vision/motion code.
 *
 * WHAT WE ARE MEASURING
 *
 * This file compares several optimization strategies across two main axes:
 *
 * * 1. RAW THROUGHPUT (The RUN_BENCH Blocks):
 * - Standard fmt::format: Baseline performance of the library.
 * - FMT_COMPILE: Measures the gain (if any) from parsing format strings at compile-time
 *      rather than runtime.
 * - AlignedBuffer + SharedBuf: Measures the impact of avoiding heap allocations
 *      by using 64-byte (cache-line) aligned thread-local storage in comparison
 *      to stack allocated, non-aligned storage


 * * 2. ARCHITECTURAL SENSITIVITY:
 * - Branch Prediction (--varied): On the Nao (Intel Atom), high branch misprediction
 *      can stall the pipeline significantly. On the Booster (Jetson), the penalty
 *      is lower but still measurable.
 * - Cache Locality (--cold): This is the most "honest" metric. Look for the "Delta"
 *      between warm and cold runs. A smaller delta indicates a "thin" logging
 *      implementation that doesn't bloat the instruction cache.
 *
 * * 3. FORMATTING OVERHEAD:
 * - We measure the "Header" (metadata) separately from the "Payload" (the actual
 *      message). This helps distinguish if the bottleneck is in the logging framework
 *      structure or the complex formatting of floating-point data (e.g., Ball coordinates).
 *
 * WHAT TO LOOK FOR
 *
 * - ns/op: The primary metric. Lower is better.
 * - Stability: The --cold --varied run should be used for regression testing as it
 *      represents a stable "worst-case" scenario that won't be artificially inflated
 *      by the CPU predicting the benchmark loop.
 *
 * ----------------------------------------------------------------------------
 * RESULTS SO FAR
 * 
 * Nao V6
 * ======
 * 
 * Cache limitations mean that using a raw fmt string (not FMT_COMPILE) is faster,
 * and using an unaligned stack allocated buffer is faster than bothering to align it
 * or reuse a shared buffer. Furthermore, even if there was a gain for FMT_COMPILE it
 * is dwarfed by the cost of IO to a file. 
 * 
 * We tried condition_variable synchronisation with the async queue but it was very
 * expensive (5x) compared to using the lock free approach.
 * 
 * Final TLOGI cost to file was about 14000 ns sync / 3000 ns async
 */

#include "Platform/SystemCall.h"

#include "TextLogging/TextLogging.h"
#include "TextLogging/TextLoggingStream.h"

#include <chrono>
#include <cstdlib> // For posix_memalign and free
#include <fmt/compile.h>
#include <fmt/format.h>
#include <iostream>
#include <new> // For std::bad_alloc
#include <string>
#include <vector>
// #include <cmath>
// #include <ctime>
// #include <cassert>

#include <condition_variable>

#ifdef __linux__
#include <sched.h> // for CPU core affinity
#endif

SystemCall::Mode SystemCall::getMode()
{
#ifdef TARGET_ROBOT
  return physicalRobot;
#else
  return simulatedRobot;
#endif
}


// This is the portable way (GCC/Clang) to tell the compiler:
// "I am using this pointer, so don't optimize away its creation or state."
template <typename T> void doNotOptimize(T *p) { asm volatile("" : : "g"(p) : "memory"); }

template <typename T> struct AlignedAllocator
{
  using value_type = T;
  // T* fixed_storage_ptr = nullptr;
  // The "Hook" - Architecture is ugly, but performance is king here.
  static thread_local T *fixed_storage_ptr;

  AlignedAllocator() noexcept = default;

  // Standard rebind constructor - does nothing because the state is in the TLS
  template <typename U> AlignedAllocator(const AlignedAllocator<U> &) noexcept {}

  // // Crucial: Copy constructor ensures that if fmt copies the allocator,
  // // the pointer to the "safe" TLS zone is preserved.
  // template <typename U>
  // AlignedAllocator(const AlignedAllocator<U> &other) noexcept
  //     : fixed_storage_ptr(reinterpret_cast<T *>(other.fixed_storage_ptr))
  // {
  // }

  T *allocate(std::size_t n)
  {
    if (n == 0)
      return nullptr;

    void *ptr = nullptr;
    // 64 is the cache-line size for Intel Atom
    if (posix_memalign(&ptr, 64, n * sizeof(T)) != 0)
      throw std::bad_alloc();

    return static_cast<T *>(ptr);
  }

  void deallocate(T *p, std::size_t) noexcept
  {
    if (p != fixed_storage_ptr)
      std::free(p);
  }

  // Boilerplate for older C++ standards often found in robot toolchains
  bool operator==(const AlignedAllocator &) const { return true; }
  bool operator!=(const AlignedAllocator &) const { return false; }
};

// Initialize the static thread_local member
template <typename T> thread_local T *AlignedAllocator<T>::fixed_storage_ptr = nullptr;

template <size_t SIZE = 2048> class AlignedBuffer : public fmt::basic_memory_buffer<char, 1, AlignedAllocator<char>>
{
public:
  // This is the actual memory we will use.
  alignas(64) char aligned_storage[SIZE];

  AlignedBuffer()
  {
    // Set the hook BEFORE calling set() or doing anything else
    AlignedAllocator<char>::fixed_storage_ptr = aligned_storage;
    // Point fmt to our aligned memory and set capacity to SIZE
    this->set(aligned_storage, SIZE);
  }
};

// The allocator acts as a "guard dog"
template <typename T = char> struct StackGuardAllocator
{
  using value_type = T;
  T *expected_ptr = nullptr;

  T *allocate(std::size_t)
  {
    // If this hits, the log message was > 2048 bytes.
    // On the Nao, we'd rather see a crash than a slow heap alloc.
    assert(false && "Log message exceeded stack buffer capacity!");
    throw std::bad_alloc();
  }

  void deallocate(T *, std::size_t) noexcept
  {
    // Do nothing. The stack will be reclaimed automatically.
    // We don't even need to check the pointer because we assert in allocate.
  }
};

template <size_t SIZE = 2048>
class StackAlignedBuffer : public fmt::basic_memory_buffer<char, 1, StackGuardAllocator<char>>
{
public:
  // This is the actual memory on the stack.
  alignas(64) char storage[SIZE];

  StackAlignedBuffer()
  {
    // We are inside a derived class, so we can call the protected set()
    this->set(storage, SIZE);
  }
};

// --- Compile-time Validation ---
// Verify the buffer class itself is aligned to a cache line
static_assert(alignof(AlignedBuffer<2048>) == 64, "AlignedBuffer class must be 64-byte aligned");
// Verify the internal storage starts at offset 0 (or at least an aligned offset)
static_assert(offsetof(AlignedBuffer<2048>, aligned_storage) % 64 == 0,
              "Internal storage must be at an aligned offset");

// Global volatile sink to prevent compiler from optimizing out "work" loops
volatile float optimizationSink = 0.0f;

// this is used to pollute the L1 cache and possibly pressure the L2 cache
// to give somewhat realistic cache situation encountered by logging calls
// (which are a minor part of the overall work of the robot code)
void simulateRobotWork()
{
  static float workBuffer[16384];
  static bool initialized = false;

  if (!initialized)
  {
    // Use a basic LCG or just a time-seeded loop to fill the buffer
    float seed = static_cast<float>(std::time(nullptr));
    for (int i = 0; i < 16384; ++i)
      workBuffer[i] = seed + i;
    initialized = true;
  }

  float sum = 0;
  for (int i = 0; i < 16384; i += 16)
  {
    // Square root and a dependency chain are very hard to optimize out
    workBuffer[i] = std::sqrt(workBuffer[i] + 0.1f);
    sum += workBuffer[i];
  }
  // Writing to a global volatile sink ensures the loop must execute
  // to produce the 'sum' value.
  optimizationSink = sum;
}

// --- New: Data Contexts to fool Branch Prediction ---
struct LogContext
{
  int timeMs;
  const char *lvl;
  const char *thdName;
  const char *lgrName;
  float x;
  float y;
};

static LogContext contexts[64];

void populateContexts()
{
  const char *levels[] = {"I", "D", "V", "WARN", "ERROR"};
  const char *threadNames[] = {"Cognition", "Perception1", "Perception2", "Audio", "Sensing"};
  const char *loggers[] = {"WalkingEngine", "Motion", "Behavior", "NaoProvider", "Vision"};

  // Simple pseudo-random seed
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  for (int i = 0; i < 64; ++i)
  {
    contexts[i].timeMs = 100000 + i * 10;
    contexts[i].lvl = levels[std::rand() % 5];
    contexts[i].thdName = threadNames[std::rand() % 5];
    contexts[i].lgrName = loggers[std::rand() % 5];
    // Vary the floats to ensure the float-to-string conversion work varies
    contexts[i].x = static_cast<float>(std::rand() % 1000) / 10.0f;
    contexts[i].y = static_cast<float>(std::rand() % 1000) / 10.0f;
  }
}

// A variable to store the baseline cost of the simulated work
double simulatedRobotWorkBaselineNs = 0.0;

int team = 0;
int player = 0;


// Use a macro to force the code into the loop with NO function call overhead
// #define RUN_BENCH(name, iterations, ...)                                                                        \
//   {                                                                                                                    \
//     auto start = std::chrono::high_resolution_clock::now();                                                            \
//     for (int i = 0; i < iterations; ++i)                                                                               \
//     {                                                                                                                  \
//       __VA_ARGS__;                                                                                                      \
//     }                                                                                                                  \
//     auto end = std::chrono::high_resolution_clock::now();                                                              \
//     std::chrono::duration<double, std::nano> elapsed = end - start;                                                    \
//     double avgNs = (elapsed.count() / iterations) - simulatedRobotWorkBaselineNs;                                      \
//     fmt::print(benchmarkOut, "{:<60} : {:10.2f} ns/op\n", name, avgNs);                                                \
//   }

// Updated RUN_BENCH: Measures each iteration individually to exclude setup/pollution.
#define RUN_BENCH(name, iterations, ...)                                                                               \
  {                                                                                                                    \
    std::chrono::nanoseconds totalDuration(0);                                                                         \
    for (int i = 0; i < iterations; ++i)                                                                               \
    {                                                                                                                  \
      if (cold)                                                                                                        \
        simulateRobotWork();                                                                                           \
      auto start = std::chrono::high_resolution_clock::now();                                                          \
      {                                                                                                                \
        __VA_ARGS__;                                                                                                   \
      }                                                                                                                \
      auto end = std::chrono::high_resolution_clock::now();                                                            \
      totalDuration += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);                              \
    }                                                                                                                  \
    double avgNs = static_cast<double>(totalDuration.count()) / iterations;                                            \
    fmt::print(benchmarkOut, "{:<70} : {:10.2f} ns/op\n", name, avgNs);                                                \
  }

// common log formatting code used, regardless of buffering strategy
#define FORMAT_LOG_OUTPUT(buf_, fmtspec_)                                                                              \
  const auto &ctx = contexts[varied ? (i % 64) : 0];                                                                   \
                                                                                                                       \
  fmt::memory_buffer msg;                                                                                              \
  fmt::format_to(std::back_inserter(msg), "Ball pos x={:.1f} y={:.1f}", ctx.x, ctx.y);                                 \
  fmt::format_to(std::back_inserter(buf_), fmtspec_, (ctx.timeMs / 1000), (ctx.timeMs % 1000), ctx.lvl, ctx.thdName,   \
                 team, player, ctx.lgrName);                                                                           \
  buf_.append(msg.data(), msg.data() + msg.size());                                                                    \
  buf_.push_back('\n');                                                                                                \
  std::fwrite(buf_.data(), 1, buf_.size(), logOut);

#define FORMAT_LOG_OUTPUT_PREFORMATTED_MSG(msg_, buf_, fmtspec_)                                                       \
  const auto &ctx = contexts[0];                                                                                       \
                                                                                                                       \
  fmt::format_to(std::back_inserter(buf_), fmtspec_, (ctx.timeMs / 1000), (ctx.timeMs % 1000), ctx.lvl, ctx.thdName,   \
                 team, player, ctx.lgrName);                                                                           \
  buf_.append(msg_.data(), msg_.data() + msg_.size());                                                                 \
  buf_.push_back('\n');                                                                                                \
  std::fwrite(buf_.data(), 1, buf_.size(), logOut);



#define hdrFmtString "{}.{:03d} {}/ [{}] t{}-r{} {} : "


// This helper simulates the proposed optimized doLogMsg_
// It accepts type-erased arguments, just like the real library would.
void __attribute__((noinline)) doLogMsgPackedArgs(const LogContext& ctx, 
                                                 fmt::string_view msgFmt, 
                                                 fmt::format_args args, 
                                                 FILE* out) 
{
    fmt::memory_buffer finalBuffer;

    // 1. Format Header directly into the final buffer
    fmt::format_to(std::back_inserter(finalBuffer), hdrFmtString, 
                   (ctx.timeMs / 1000), (ctx.timeMs % 1000), 
                   ctx.lvl, ctx.thdName, team, player, ctx.lgrName);

    // 2. Format User Message directly into the SAME buffer (Zero-Copy)
    fmt::vformat_to(std::back_inserter(finalBuffer), msgFmt, args);
    finalBuffer.push_back('\n');

    doNotOptimize(finalBuffer.data());

    std::fwrite(finalBuffer.data(), 1, finalBuffer.size(), out);
}

void __attribute__((noinline)) doLogMsgPreformatted(const LogContext& ctx, 
                                                 fmt::memory_buffer& msg, 
                                                 FILE* out) 
{
    fmt::memory_buffer finalBuffer;

    // 1. Format Header directly into the final buffer
    fmt::format_to(std::back_inserter(finalBuffer), hdrFmtString, 
                   (ctx.timeMs / 1000), (ctx.timeMs % 1000), 
                   ctx.lvl, ctx.thdName, team, player, ctx.lgrName);

    finalBuffer.append(msg.data(), msg.data() + msg.size());                                                                 \
    finalBuffer.push_back('\n');                                                                                                \

    doNotOptimize(finalBuffer.data());

    std::fwrite(finalBuffer.data(), 1, finalBuffer.size(), out);
}

// -------------------------------------------------------------------
// main code
// -------------------------------------------------------------------

DECL_TLOGGER(tlogger, "BenchmarkTextLogging", TextLogging::INFO);

int main(int argc, char * argv[])
{
#ifdef __linux__
  // ensure this code sticks to one core for the benchmark duration
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset); // Pin to the first core (CPU 0)
  if (sched_setaffinity(0, sizeof(cpu_set_t), &cpuset) != 0)
  {
    perror("sched_setaffinity");
  }
#endif

  setvbuf(stdout, nullptr, _IONBF, 0);
  setvbuf(stderr, nullptr, _IONBF, 0);

  FILE *benchmarkOut = stdout;
  FILE *logOut = stderr;
  const int iterations = 50000;

  // Command line flags
  bool cold = false;
  bool varied = false;
  bool async = false;
  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if (arg == "--cold")
      cold = true;
    if (arg == "--varied")
      varied = true;
    if (arg == "--async")
      async = true;
  }

  fmt::print(benchmarkOut, "--- TextLogging Benchmark (Cold cache: {}, Varied messages: {}, Async logging: {}) ---\n",
             cold ? "YES" : "NO", varied ? "YES" : "NO", async ? "YES" : "NO");

  if (async)
    TextLogging::requestAsyncOutput(true);

  populateContexts();
  team = (std::rand() % 15) + 5;
  player = (std::rand() % 7) + 1;

  // the compiled format string - might be faster on some platforms but it generates a lot more
  // code so it may not be faster if it is cache unfriendly on the target processor (E.g. the Nao's Atom)
  static constexpr auto compiled = FMT_COMPILE(hdrFmtString);
  

  // using SharedBuffer = fmt::basic_memory_buffer<char, 2048>;
  using SharedBuffer = AlignedBuffer<2048>;
  alignas(64) thread_local SharedBuffer sharedBuf;

  using AlignedStackBuffer = StackAlignedBuffer<512>;

  // --- Compile-Time Assert ---
  // This ensures the compiler aligns the start of the object to 64 bytes.
  // static_assert(alignof(decltype(sharedBuf)) == 64,
  //               "SharedBuffer object itself is not aligned to 64 bytes!");

  // --- Runtime Address Check ---
  // We check the address of the actual data pointer.
  // In fmt v11, for a fresh buffer, sharedBuf.data() should point to its internal store.
  uintptr_t dataAddr = reinterpret_cast<uintptr_t>(sharedBuf.data());
  if (dataAddr % 64 != 0)
  {
    fmt::print(benchmarkOut, "FAILURE: sharedBuf.data() is NOT aligned ({}). Offset is {}\n", (void *)dataAddr,
               (size_t)(dataAddr % 64));
  }

  // Warm up: Force the initialization and prime the CPU cache
  for (int i = 0; i < 100; ++i)
  {
    sharedBuf.clear();
    fmt::format_to(std::back_inserter(sharedBuf), compiled, 1, 1, "I", "WarmUpThread", 1, 1, "WarmUpLogger");
  }

  for (int i=0; i < 100; i++)
    TLOGI(tlogger, "Warming up text logging");


  // fmt::memory_buffer staticMsgBuffer;
  // fmt::format_to(std::back_inserter(staticMsgBuffer), "Ball pos x={:.1f} y={:.1f}", 207.3, 125.4);

  std::string staticMsgBuffer = "Ball pos x=207.3 y=125.4";

  // warm up the processor core for 100 ms - we want frequency scaling to be 
  // high and stable to simulate running under full load conditions

  auto wakeUpStart = std::chrono::high_resolution_clock::now();
  while(std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - wakeUpStart).count() < 100)
  {
    simulateRobotWork();
  }  


  // -----------------------------------------------------------------------
  // MEASUREMENT
  // -----------------------------------------------------------------------

  fmt::print(benchmarkOut, "--- Starting Benchmark ---\n");

  // --- IONBF (Unbuffered) ---
  std::setvbuf(stdout, nullptr, _IONBF, 0);

  RUN_BENCH("i/o: IONBF, msg: fmt_string, log: fmt_string, buf: 2 stack, append", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];                                                                   
    
    // simulate TLOG macro    
    fmt::memory_buffer msg;                                                                                              
    fmt::format_to(std::back_inserter(msg), "Ball pos x={:.1f} y={:.1f}", ctx.x, ctx.y);

    doLogMsgPreformatted(ctx, msg, logOut);
  });

  RUN_BENCH("i/o: IONBF, msg: fmt_string, log: fmt_string, buf: 1 stack", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];                                                                   
    
    // simulate TLOG macro
    // 1. Pack arguments (This is the Macro overhead)
    auto args = fmt::make_format_args(ctx.x, ctx.y);

    // 2. Call the simulated doLogMsg_
    doLogMsgPackedArgs(ctx, "Ball pos x={:.1f} y={:.1f}", args, logOut);
  });


  RUN_BENCH("i/o IONBF, msg fmt_string, log fmt_string, buf stack, move value", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];                                                                   
    
    // simulate TLOG macro
    
    fmt::memory_buffer msg;                                                                                              
    fmt::memory_buffer step1;                                                                                              
    fmt::format_to(std::back_inserter(msg), "Ball pos x={:.1f} y={:.1f}", ctx.x, ctx.y);                                 
    fmt::format_to(std::back_inserter(step1), hdrFmtString, (ctx.timeMs / 1000), (ctx.timeMs % 1000), ctx.lvl, ctx.thdName,   
                  team, player, ctx.lgrName);                                                                           
    step1.append(msg.data(), msg.data() + msg.size());                                                                    
    step1.push_back('\n');
    doNotOptimize(&step1);                                                                                                

    // simulate passing to doLogMsg_
    // Moves step1 to step2. If step1 < 500 bytes, this is a memcpy.    
    fmt::memory_buffer step2 = std::move(step1);
    doNotOptimize(&step2);

    // Moves into the vector. Again, potentially a memcpy.
    std::vector<fmt::memory_buffer> queue;
    queue.push_back(std::move(step2));
    doNotOptimize(queue.data());

    std::fwrite(queue.back().data(), 1, queue.back().size(), logOut);
  });

  RUN_BENCH("i/o IONBF, msg fmt_string, log fmt_string, buf stack, move unique_pointer", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];                                                                   
    
    // simulate TLOG macro
    
    fmt::memory_buffer msg;                                                                                              
    auto step1 = std::make_unique<fmt::memory_buffer>();                                                                                              
    fmt::format_to(std::back_inserter(msg), "Ball pos x={:.1f} y={:.1f}", ctx.x, ctx.y);                                 
    fmt::format_to(std::back_inserter(*step1), hdrFmtString, (ctx.timeMs / 1000), (ctx.timeMs % 1000), ctx.lvl, ctx.thdName,   
                  team, player, ctx.lgrName);                                                                           
    step1->append(msg.data(), msg.data() + msg.size());                                                                    
    step1->push_back('\n');
    doNotOptimize(step1.get());                                                                                                

    // simulate passing to doLogMsg_
    auto step2 = std::move(step1);
    doNotOptimize(step2.get());

    // Moves into the vector (simulates async queue).
    std::vector<std::unique_ptr<fmt::memory_buffer>> queue;
    queue.push_back(std::move(step2));
    doNotOptimize(queue.data());

    std::fwrite(queue.back()->data(), 1, queue.back()->size(), logOut);
  });

  RUN_BENCH("i/o IONBF, msg fmt_string, log fmt_string, buf stack, move unique_pointer, mtx+cv", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];

    // 1. Formatting Logic
    fmt::memory_buffer msg;
    auto step1 = std::make_unique<fmt::memory_buffer>();
    fmt::format_to(std::back_inserter(msg), "Ball pos x={:.1f} y={:.1f}", ctx.x, ctx.y);
    fmt::format_to(std::back_inserter(*step1), hdrFmtString, (ctx.timeMs / 1000), (ctx.timeMs % 1000), ctx.lvl,
                   ctx.thdName, team, player, ctx.lgrName);
    step1->append(msg.data(), msg.data() + msg.size());
    step1->push_back('\n');
    doNotOptimize(step1.get());

    // 2. The "Async Tax" Simulation - Primitives are REUSED (static)
    static std::mutex simMutex;
    static std::condition_variable simCV;
    static std::vector<std::unique_ptr<fmt::memory_buffer>> simQueue;

    {
      // This measures the cost of atomic synchronization (the lock)
      std::lock_guard<std::mutex> lock(simMutex);

      auto step2 = std::move(step1);
      doNotOptimize(step2.get());

      // This measures the cost of resizing/inserting into an existing vector
      simQueue.push_back(std::move(step2));
      doNotOptimize(simQueue.data());
    }

    // This measures the 'Syscall Tax' (futex wake)
    simCV.notify_one();

    // 3. I/O Simulation
    // We clear the queue here so that the NEXT iteration finds a
    // vector that is already allocated but empty (reusing the capacity).
    if (!simQueue.empty())
    {
      std::fwrite(simQueue.back()->data(), 1, simQueue.back()->size(), logOut);
      simQueue.clear();
    }
  });

  // these next 2 are commented out for now because no realistic implementation of text logging can do this.
  // They are only useful to measure low level differences between string and FMT_COMPILE in the absence of
  // all other delays.
//   RUN_BENCH("i/o IONBF, msg static, log fmt_string, buf stack", iterations, {
//     fmt::memory_buffer buf;
//     FORMAT_LOG_OUTPUT_PREFORMATTED_MSG(staticMsgBuffer, buf, "{}.{:03d} {}/ [{}] t{}-r{} {} : ");
//   });
// 
//   RUN_BENCH("i/o IONBF, msg static, log FMT_COMPILE, buf stack", iterations, {
//     fmt::memory_buffer buf;
//     FORMAT_LOG_OUTPUT_PREFORMATTED_MSG(staticMsgBuffer, buf, compiled);
//   });

  RUN_BENCH("i/o IONBF, msg fmt_string, log fmt_string, buf stack", iterations, {
    fmt::memory_buffer buf;
    FORMAT_LOG_OUTPUT(buf, "{}.{:03d} {}/ [{}] t{}-r{} {} : ");
  });

  RUN_BENCH("i/o IONBF, msg fmt_string, log FMT_COMPILE, buf stack", iterations, {
    fmt::memory_buffer buf;
    FORMAT_LOG_OUTPUT(buf, compiled);
  });

  RUN_BENCH("i/o IONBF, msg fmt_string, log FMT_COMPILE, buf aligned_stack", iterations, {
    AlignedStackBuffer alignedStackBuf;
    FORMAT_LOG_OUTPUT(alignedStackBuf, compiled);
  });

  RUN_BENCH("i/o IONBF, msg fmt_string, log FMT_COMPILE, buf aligned_tls_reused", iterations, {
    sharedBuf.clear();
    FORMAT_LOG_OUTPUT(sharedBuf, compiled);
  });

  // -- the textlogging library --
  
  RUN_BENCH("i/o IONBF, TLOGI", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];
    TLOGI(tlogger, "Ball pos x={} y={}", ctx.x, ctx.y);
  });

  RUN_BENCH("i/o IONBF, TLOGI_STREAM", iterations, {
    const auto &ctx = contexts[varied ? (i % 64) : 0];
    TLOGI_STREAM(tlogger, "Ball pos x=" << ctx.x << ", y=" << ctx.y);
  });

  // --- IOLBF (Line Buffered) ---
  // Commented out because it was slower than IONBF

//   static char lineBuf[1024];
//   std::setvbuf(logOut, lineBuf, _IOLBF, sizeof(lineBuf));
// 
//   RUN_BENCH("i/o IOLBF, msg fmt_string, log fmt_string, buf stack", iterations, {
//     fmt::memory_buffer buf;
//     FORMAT_LOG_OUTPUT(buf, "{}.{:03d} {}/ [{}] t{}-r{} {} : ");
//   });
// 
//   RUN_BENCH("i/o IOLBF, msg fmt_string, log FMT_COMPILE, buf stack", iterations, {
//     fmt::memory_buffer buf;
//     FORMAT_LOG_OUTPUT(buf, compiled);
//   });

  fmt::print(benchmarkOut, "--- Benchmark Complete ---\n");
  std::exit(0);
}