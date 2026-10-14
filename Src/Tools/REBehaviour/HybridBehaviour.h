/**
 * @file HybridBehaviour.h
 *
 * A framework that supports lightweight stackless coroutine behaviours,
 * hierarchical state machine (HSM) behaviours, and reactive Behaviour Tree
 * combinators (sequence and fallback). Collectively we refer to 
 * coroutines (coros) and HSMs as tasks.
 *
 * Features:
 * - tasks look like resumable methods contained within some arbitrary class
 *   (No class hierarchy or base class type is imposed)
 * - tasks have access to all the members of the container class
 * - Implicit context management (auto-reset on task switch).
 * - Explicit separation of Inputs (ARGS) and State (VARS).
 * - Concise syntax with optional ARGS/VARS/DEFINES_PARAMS.
 * - Inline and out-of-class definition support via DECLARE/DEFINE.
 *
 * Acknowledgement:
 * This framework draws on ideas and some structural patterns from CABSL
 * (C-based Agent Behavior Specification Language) by Thomas Röfer, B-Human.
 * Specifically: the option-as-class-method architecture, the inline/out-of-class
 * split syntax, the args/vars/defs three-way parameter distinction, and the
 * goto-label-based HSM state entry mechanism are inspired by CABSL. 
 * The coroutine system, BT combinators, and general Env/Context design
 * are original additions.
 *
 * @author Rudi Villing
 */

#pragma once

#include "HbsMacroUtils.h"
#include "HbsTypes.h"
#include "TickTrace.h"

#include "Platform/BHAssert.h"
#include "TextLogging/TextLogging.h"

#include <memory>
#include <new>
// #include <fmt/core.h>

// configurable values and macros that can be overridden externally
#ifndef HBS_MAX_BT_SLOTS
#define HBS_MAX_BT_SLOTS 8
#endif

// #ifndef HBS_ASSERT
// #include <cassert>
// #define HBS_ASSERT(cond_, ...) assert((cond_) __VA_OPT__(&& (__VA_ARGS__)))
// #endif

#define HBS_ASSERT  ASSERT
#define HBS_FAIL    FAIL_FMT


namespace Hbs
{
inline DECL_TLOGGER_FN(tlogger, "Hbs", TextLogging::DEBUG);


template<class T>
T testAndSet(T& var, T value)
{
    T oldVal = var;
    var = value;
    return oldVal;
}


/**
 * Central environment for a group of coro or HSM tasks.
 * Tracks time and tick count for proper continuity checking and auto-reset.
 *
 * Usage:
 *   1. Declare in container class via HBS_ENV() macro
 *   2. Call HBS_BEGIN_TICK(currentTime) before executing tasks each frame
 */
class Env
{
public:
  // no default constructor
  explicit Env(TickTrace *trace = nullptr, const std::string &configPrefix = "hbs_")
      : mTickTrace(trace),
        mConfigPrefix(configPrefix)
  {}

  inline Time currentTickTime() const { return mCurrentTickTime; }
  inline Time prevTickTime() const { return mPrevTickTime; }
  inline unsigned currentTick() const { return mCurrentTick; }
  inline unsigned prevTick() const { return mPrevTick; }

  TickTrace* tickTrace() { return mTickTrace; }

  const std::string& configPrefix() { return mConfigPrefix; }

  /**
   * Call at the start of each cognition cycle before executing any coroutines.
   * Updates time and tick count for continuity checking.
   */
  void beginTick(Time timeNow)
  {
    mPrevTickTime = mCurrentTickTime;
    mPrevTick = mCurrentTick;
    mCurrentTickTime = timeNow;
    mCurrentTick++;

    if (mTickTrace)
      mTickTrace->beginTick();
  }

private:
  Time mCurrentTickTime = 0;
  Time mPrevTickTime = 0;
  unsigned mCurrentTick = 0;
  unsigned mPrevTick = 0;

  TickTrace* mTickTrace;
  std::string mConfigPrefix;
};

/**
 * Context class used to remember state of a task (Coroutine or HSM).
 * There is a context instance for each task, but this is hidden behind 
 * the macro implementations
 */
class Context
{
public:
  static constexpr int RESUME_NEVER_SET = -2;
  static constexpr int RESUME_RESET = -1;
  static constexpr int RESUME_START = 0;
  static constexpr const char* STATE_NAME_NEVER_SET = "StateNameWasNeverSet";
  static constexpr const char* STATE_NAME_START = "start_";

  enum class CycleMode { TRY, DEFAULT=TRY, NO_CYCLE, FORCE_CYCLE }; // TRY (try cycle and fallback to autoreset if not) is the default

  Context(Env& env, const char *name)
      : mEnv(env), mTaskName(name)
  {
  }

  Status lastStatus() const { return mLastStatus; }
  CycleMode cycleMode() const { return mCycleMode; }

  Time currentTickTime() { return mEnv.currentTickTime(); }
  unsigned currentTick() { return mEnv.currentTick(); }

  Time taskDuration() const { return mEnv.currentTickTime() - mTaskStartTime; }
  unsigned taskTicks() const { return mEnv.currentTick() - mTaskStartTick; }
  Time stateDuration() const { return mEnv.currentTickTime() - mStateStartTime; }
  unsigned stateTicks() const { return mEnv.currentTick() - mStateStartTick; }

  Time timeSince(Time before) { return mEnv.currentTickTime() - before; }
  unsigned ticksSince(unsigned before) { return mEnv.currentTick() - before; }

  const char* stateName() const { return mStateName; }
  const char* taskName() const { return mTaskName; }

  /// true whenever we are starting the current state (including re-entering the initial state after auto-reset)
  bool isStateEntry() const { return stateTicks() == 0; }

  /// true for the tick in which a genuine state change occurred (i.e. false if re-entering the stableStateName
  /// unlike isStateEntry)
  bool stateChanged() const { return mStateChanged; }
  const char* stableStateName() const { return mStableStateName; }

  bool isInitialOrRunning() const { return mLastStatus == INITIAL || mLastStatus == RUNNING;}
  bool isRunning() const { return mLastStatus == RUNNING; }
  bool finished() const { return (mLastStatus == SUCCESS) || (mLastStatus == FAILURE); }
  bool succeeded() const { return mLastStatus == SUCCESS; }
  bool failed() const { return mLastStatus == FAILURE; }
  
  void reset()
  {
    mResumePoint = RESUME_RESET;
    mVarsNeedReset = true;
    mLastStatus = Status::INITIAL; // the status after the task has executed
    mTaskStartTick = 0; // when this is 0 on beginTick, we reset the times also
    mStateStartTick = 0;

  }

  // add a string to the tick trace - for information before the state executes
  // (e.g. args, other info that causes a particular transition to be selected, etc.)
  template <typename... Args> 
  void traceContext(fmt::format_string<Args...> fmt, Args &&...args)
  {
    auto *t = mEnv.tickTrace();
    if (t)
      t->addContextFrameString(mTraceFrameIndex, fmt::format(fmt, std::forward<Args>(args)...));
  }

  // add a string to the tick trace - nominally during the execution of a state
  // describing progress or outcomes in the state
  template <typename... Args> 
  void trace(fmt::format_string<Args...> fmt, Args &&...args)
  {
    auto *t = mEnv.tickTrace();
    if (t)
      t->addFrameString(mTraceFrameIndex, fmt::format(fmt, std::forward<Args>(args)...));
  }

  // ---------------------------------------------------------------------------
  // the following methods (named with a '_' suffix) are internal implementation 
  // exposed for macro use only - do not call directly!
  // ---------------------------------------------------------------------------

  int resumePoint_() const { return mResumePoint; }
  void setResumePoint_(int resumePoint) { mResumePoint = resumePoint; } // used in YIELD etc.

  bool isAtStart_() const { return mResumePoint == RESUME_START; }
  bool isAtState_(int resumePoint) const { return mResumePoint == resumePoint; }
  bool transitionsEnabled_() const { return !isStateEntry() || mTransitionsOnEntry; }

  void setLastStatus_(Status status) { mLastStatus = status; }

  void setCycleMode_(CycleMode mode) { mCycleMode = mode; }

  bool holdIfPossible_()
  {
    if (!finished())
      return false;
    if (skippedTickAutoResetNeeded())
      return false;                       // gap: let normal reset happen on next real call
    mLastTick = mEnv.currentTick();       // keep alive to prevent gap-reset next tick
    return true;
  }

  // helper to begin a tick of the associated task
  void beginTick_()
  {
    if (skippedTickAutoResetNeeded())
    {
       reset();
    }
    else if (finished()) // handle cycling the terminal state
    {
      bool canCycle = (mResumePoint != RESUME_RESET);

      if (mCycleMode == CycleMode::FORCE_CYCLE)
        HBS_ASSERT(canCycle && "cycling required but task not capable"); // fail immediatedly if requested cycling not possible

      // cycle if possible and NO_CYCLE not explicitly specified
      const bool doCycle = canCycle && (mCycleMode != CycleMode::NO_CYCLE);
      if (!doCycle)
        reset(); // fall back to auto-reset if cycling not possible or NO_CYCLE requested
    }

    mCycleMode = CycleMode::DEFAULT; // consume the cycle mode — must be re-set each tick by the wrapper if non-default needed

    // If reset happened or first run -- TODO check this logic
    if ((mResumePoint == RESUME_RESET) && (mLastTick != mEnv.currentTick()))
    {
      mTaskStartTime = mEnv.currentTickTime();
      mTaskStartTick = mEnv.currentTick();

      mStateStartTime = mEnv.currentTickTime();
      mStateStartTick = mEnv.currentTick();
      mStateName = STATE_NAME_NEVER_SET; // we should never see this state - it should be overwritten in Coro and HSM
    }

    mStateChanged = false; // will be set true if a real state change does take place
    mLastTick = mEnv.currentTick();
  }

  // helper to add to the tick trace for the associated task
  void beginTrace_()
  {
    auto* t = mEnv.tickTrace();
    mTraceFrameIndex = t ? t->beginTaskFrame(mTaskName, taskDuration(), taskTicks()) : -1;
  }

  // helper to finalise the tick trace for the associated task
  void endTrace_()
  {
    auto *t = mEnv.tickTrace();
    if (t)
      t->endTaskFrame(mTraceFrameIndex, mLastStatus, mStateName, stateDuration(), stateTicks());
  }

  void setState_(const char* stateName)
  {
    // has the stable state changed?
    if (stateName != mStableStateName)
    {
      mStateChanged    = true; // (mStableStateName != nullptr); // first settle of a run is an entry, not a change
      mStableStateName = stateName;
    }

    // has the state name changed - might just be part of the auto-reset mechanism
    // (not a stable state change) but still requires us to reset timers etc
    if (stateName != mStateName)
    {
      mStateName      = stateName;
      mStateStartTime = mEnv.currentTickTime();
      mStateStartTick = mEnv.currentTick();
    }
  }

  // mark a new "state" or stage of progress in a coro
  // can be safely and cheaply called on each tick (E.g. in a loop)
  void setCoroState_(const char* stateName, int resumePoint)
  {
    // fast check for repeat of the current checkpoint
    if (resumePoint != mResumePoint)
    {
      mResumePoint = resumePoint;
      setState_(stateName);
    }
  }

  // mark the first entry into a state (usually after a transition)
  // note that values are only set if this is a new resumePoint (i.e. after changing state)
  void enterHsmState_(int resumePoint, const char *stateName, Status statusFromStateType, bool transitionsOnEntry = false)
  {
    if (resumePoint != mResumePoint)
    {
      mResumePoint = resumePoint;
      setState_(stateName); // also updates timers

      // this will be the status after we have executed the state code. It is set
      // here because the type of state (initial state, normal state, success state or failure state)
      // determines the mLastStatus that will be reported for this tasks and the macro calling
      // this function knows what type of state and hence the status to use.
      mLastStatus = statusFromStateType;
      mTransitionsOnEntry = transitionsOnEntry;
    }
  }

  /** ensure the VarsType is ready for the current frame, reinitialising it if needed */
  template <typename VarsType>
  void prepareVars_()
  {
    if (!mVars)
    {
      mVars.reset(new VarsType());
      mVarsNeedReset = false;
    }
    else if (mVarsNeedReset)
    {
      // Reconstruct in place
      auto* ptr = static_cast<VarsType*>(mVars.get());
      ptr->~VarsType();
      new (ptr) VarsType();
      mVarsNeedReset = false;
    }
  }

  template <typename VarsType>
  VarsType* getVars_() { return static_cast<VarsType*>(mVars.get()); }

  // ----------------------------------------------------------------------
  // BT specific macro support
  // ----------------------------------------------------------------------

  // Called at the top of each BT combinator lambda.
  // Invalidates slots when the active combinator changes; always resets cursor.
  void beginBtCombinator_(uint16_t combId)
  {
    if (combId != mActiveBtCombId)
    {
      mActiveBtCombId = combId;
      mBtSlotCount = 0;
    }
    mBtCursor = 0;
  }

  // Backs HBS_HOLD and HBS_DEFER. Delegates all logic here to keep the macro minimal.
  // HBS_DEFER is HBS_HOLD with the additional step of returning RUNNING one more time
  // when the terminal status is first encountered to prevent immediate progression to the next
  // child in the combinator.
  // Note that fn is a lambda that has already captured the real function/task to be invokved and its args
  template <typename Fn> Status applyHold_(Fn &&fn, bool defer)
  {
    BtSlot& slot = nextBtSlot_();
    if (slot.heldOrPending)
      return slot.cached;
    Status s = fn();
    if (s != RUNNING)
    {
      slot.heldOrPending = true;
      slot.cached = s;
      if (defer)
        return RUNNING; // defer the terminal status for one tick
    }
    return s;
  }

private:
  int mResumePoint = RESUME_RESET;
  unsigned mLastTick = 0;
  Status mLastStatus = Status::INITIAL;
  bool mTransitionsOnEntry = false;
  CycleMode mCycleMode = CycleMode::DEFAULT;

  Env &mEnv;
  
  Time mTaskStartTime = 0;
  Time mStateStartTime = 0;
  const char* mTaskName = "unknownTask";
  const char *mStateName = "start";
  unsigned int mTaskStartTick = 0;
  unsigned int mStateStartTick = 0;

  // Last DISTINCT state the task executed. PERSISTS across auto-resets
  // (reset() does not clear it). We can use this to distinguish between "re-entered the same
  // state after a reset" (not a state change) to executed a different state (a
  // change). nullptr => nothing committed yet. Pointer compare only.
  const char* mStableStateName = nullptr;
  bool mStateChanged = false;   // true only in the tick where change was committed


  bool mVarsNeedReset = false;
  std::unique_ptr<VarStorage> mVars;

  int mTraceFrameIndex = -1;

  // implement our skipped tick semantics which should automatically reset a
  // task back to its initial condition
  bool skippedTickAutoResetNeeded()
  {
    return ((mLastTick != mEnv.currentTick() - 1) && (mLastTick != mEnv.currentTick()));
  }

  // ----------------------------------------------------------------------
  // BT specific support
  // ----------------------------------------------------------------------
private:
  struct BtSlot
  {
    Status cached = INITIAL;
    bool heldOrPending = false; // true if status held or terminal status being deferred to next tick
  };

  static constexpr int MAX_BT_SLOTS = HBS_MAX_BT_SLOTS;

  BtSlot mBtSlots[MAX_BT_SLOTS] = {};
  uint8_t mBtSlotCount = 0;     // high-water mark of allocated slots
  uint8_t mBtCursor = 0;        // position during current combinator evaluation
  uint16_t mActiveBtCombId = 0; // identity of currently active combinator

  BtSlot &nextBtSlot_()
  {
    if (mBtCursor == mBtSlotCount)
    {
      HBS_ASSERT(mBtSlotCount < MAX_BT_SLOTS);
      if (mBtSlotCount < MAX_BT_SLOTS)
        mBtSlots[mBtSlotCount++] = BtSlot{};
      else
        return mBtSlots[MAX_BT_SLOTS - 1]; // release mode only: reuse last slot - programming error if we get here and
                                         // likely breaks the sequence behaviour
    }
    return mBtSlots[mBtCursor++];
  }
};


// helper function
inline bool finished(Status s) { return s == Hbs::SUCCESS || s == Hbs::FAILURE; }



// The config file helper

struct ConfigFile : public InMapFile
{
  ConfigFile(const std::string& prefix, const std::string& name) : InMapFile(prefix + name + ".cfg")
  {
    if (!exists())
      TLOG_ABORT(tlogger(), "Failed to load config file: {}{}.cfg", prefix, name);
  }
};


// ---------------------------------------------------------------------------
// EnclosingClass trait - detail to help registering tasks by name
//
// Extracts the class type from a pointer-to-member type.
// e.g. given:   Hbs::Context SomeEnclosingClass::*
//      yields:  SomeEnclosingClass
//
// Primary template is intentionally undefined — produces a readable
// static_assert if you accidentally pass a non-pointer-to-member type.
// ---------------------------------------------------------------------------
// template<typename PtrToMemberType>
// struct EnclosingClass
// {
//   static_assert(sizeof(PtrToMemberType) == 0,
//       "EnclosingClass requires a pointer-to-member type, "
//       "e.g. decltype(&MyClass::myMember)");
// };
// 
// template<typename MemberType, typename ContainerClass>
// struct EnclosingClass<MemberType ContainerClass::*>
// {
//   using type = ContainerClass;
// };
// 
// // Convenience alias so callers don't need ::type everywhere
// template<typename PtrToMemberType>
// using EnclosingClass_t = typename EnclosingClass<PtrToMemberType>::type;


// The type-erased function pointer stored in the registry.
// Receives the executor as void* — cast back to concrete type happens
// inside the lambda written at class scope where the type is known.
// using TaskFn = Hbs::Status (*)(void*);
// using RegistrationFn = void (*)();

template <typename Container>
struct TaskRegistry : public std::unordered_map<std::string, Hbs::Status (Container::*)()>
{
  using TaskFn = Hbs::Status (Container::*)();
  using RegisterFn = void (*)(TaskRegistry &);

  void registerTask(const char* name, TaskFn fn)
  {
    TLOGD(tlogger(), "registered task: {}", name);
    (*this)[name] = fn;
  }

  TaskFn findRequiredTask(const std::string& name) const
  {
    auto it = this->find(name);
    if (it == this->end())
    {
      HBS_FAIL("task \"{}\" not registered", name);
      return nullptr;
    }
    return it->second;
  }

  bool isRegistered(const std::string& name) const
  {
    return this->find(name) != this->end();
  }

  void addPendingRegistration(RegisterFn regFn)
  {
    mPendingRegistrations.push_back(regFn);
  }

  void runPendingRegistrations()
  {
    if (mAllRegistered)
      return; // TODO ASSERT??
    for (auto regFn : mPendingRegistrations)
      regFn(*this);
    mPendingRegistrations.clear();
    mAllRegistered = true;
  }

private:
  std::vector<RegisterFn> mPendingRegistrations;
  bool mAllRegistered = false; 
};

// A struct that will invoke runPendingRegistrations  at construction time.
// Place as a member after sTaskRegistry_
template <typename Container>
struct TaskRegistryInit
{
  TaskRegistryInit(TaskRegistry<Container> &registry)
  {
    registry.runPendingRegistrations();
  }
};

} // namespace Hbs

// ===========================================================================
// top level Macros
// ===========================================================================

/**
 * HBS_ENV: Macro to declare the standard Env member in a container class.
 *
 * Usage:
 * @code
 *   class MyBehaviour {
 *   public:
 *     HBS_ENV();
 *
 *     CORO_DEFINE((task1), ...) { ... }
 *     HSM_DEFINE((task2), ...) { ... }
 *
 *     void execute(Time currentTime) {
 *       HBS_BEGIN_TICK(currentTime);
 *       myTask();
 *     }
 *   };
 * @endcode
 *
 * This declares the shared env that all coroutines in this class will
 * reference. Call HBS_BEGIN_TICK(currentTime) at the start of each cognition
 * cycle.
 */
#define HBS_ENV(...) \
  Hbs::Env hbsEnv{__VA_ARGS__}

/**
 * HBS_BEGIN_TICK: Call at the start of each cognition cycle before executing
 * coroutines. Updates the shared environment's time and tick count.
 *
 * @param timeNow Current time value (Time)
 */
#define HBS_BEGIN_TICK(timeNow) \
  hbsEnv.beginTick(timeNow)

// ===========================================================================
// Macros for a task registry to register top level tasks by string
// ===========================================================================

/**
 * HBS_TASK_REGISTRY: Place once in the executor class body alongside HBS_ENV().
 *
 * Emits:
 *   - a static map from name to type-erased TaskFn
 *   - executeTask(name) — ticks the named task on this instance
 *   - registerTask_() — called by HBS_REGISTER_TASK at static init time
 *
 * No class name argument needed.
 */
#define HBS_TASK_REGISTRY(className_)                                                                                  \
  using HbsContainer_ = className_;                                                                                    \
  inline static Hbs::TaskRegistry<className_> sTaskRegistry_;                                                          \
                                                                                                                       \
  /* drains pending registrations when first instance is constructed */                                                \
  /* declared after sTaskRegistry_ so it's initialised after it */                                                     \
  Hbs::TaskRegistryInit<className_> sTaskRegistryInit_{sTaskRegistry_};                                                \
                                                                                                                       \
  Hbs::Status executeTask_(const std::string &name)                                                                    \
  {                                                                                                                    \
    auto pMemberFn = sTaskRegistry_.findRequiredTask(name);                                                            \
    return pMemberFn ? (this->*pMemberFn)() : Hbs::INITIAL;                                                            \
  }

/**
 * HBS_REGISTER_TASK: Place immediately after a CORO_DECLARE / HSM_DECLARE /
 * inline CORO_DEFINE / inline HSM_DEFINE for any task that should be
 * invocable by name via runTask().
 *
 * Only tasks whose _Args is default-constructible can be registered — the
 * zero-arg wrapper taskName_() will not exist otherwise and this will fail
 * to compile. That is intentional: tasks with mandatory args cannot be
 * top-level entry points without a separate argument-passing mechanism.
 */
#define HBS_REGISTER_TASK(taskName_)                                                                                   \
  static void HBS__JOIN(taskName_, _register_)(Hbs::TaskRegistry<HbsContainer_> & reg)                                 \
  {                                                                                                                    \
    reg.registerTask(#taskName_, &HbsContainer_::taskName_);                                                           \
  }                                                                                                                    \
                                                                                                                       \
  static inline const bool HBS__JOIN(taskName_, _registered_) =                                                        \
      (sTaskRegistry_.addPendingRegistration(&HBS__JOIN(taskName_, _register_)), true)

/**
 * check if a task is registered
 */
#define HBS_TASK_IS_REGISTERED(taskName_) sTaskRegistry_.isRegistered(taskName_)

/**
 * Execute a registered task
 */
#define HBS_EXECUTE(taskName_) executeTask_(taskName_)

// --- Naming Convention ---
// HBS__ : Prefix for "Hbs" internal macros.
// M    : "Mapper" macro. The macro to apply to each item in a list.
// D    : "Delimiter". The token to inject between items (e.g., a comma).


// ===========================================================================
// Macros common to all tasks
// ===========================================================================

/// access the context object related functions of a task
#define TASK(methodName_) HBS__JOIN(methodName_, _ctx)

/// own context
#define THIS_TASK ctx

/**
 * Wrapper/decorator to specify if a task MUST cycle (repeat) its terminal
 * state. (Without this wrapper, the framework tries to cycle the task but 
 * falls back to auto-reset if the task cannot.)
 *
 * Works for tasks that take or do not take arguments
 * @code
 * HBS_CYCLE(someTask);
 * HBS_CYCLE(someOtherTask, {.firstArg=1, .secondArg="A string"});
 * @endcode 
 *
 * ASSERTs if the
 * task is not capable of cycling. A HSM with a SUCCESS/FAILURE state, can cycle
 * by default. A Coro requires a block like while (cond...) { code...;
 * CORO_YIELD_SUCCESS(); } or similar in order to cycle. CORO_SUCCESS/FAILURE or
 * CORO_END_SUCCESS/FAILURE do not support cycling.
 */
#define HBS_CYCLE(taskName_, ...) \
    (HBS__JOIN(taskName_, _ctx).setCycleMode_(Hbs::Context::CycleMode::FORCE_CYCLE), taskName_(__VA_ARGS__))

/**
 * Variant of HBS_CYCLE that forces auto-reset after the terminal state, even if
 * cycling would have been possible. This is rarely needed in practice.
 */
#define HBS_NO_CYCLE(taskName_, ...) \
    (HBS__JOIN(taskName_, _ctx).setCycleMode_(Hbs::Context::CycleMode::NO_CYCLE), taskName_(__VA_ARGS__))


#define HBS_SUCCESS(expr_) ((expr_) == Hbs::SUCCESS)
#define HBS_FAILURE(expr_) ((expr_) == Hbs::FAILURE)
#define HBS_FINISHED(expr_) (finished(expr_))
#define HBS_RUNNING(expr_) ((expr_) == Hbs::RUNNING)

// ===========================================================================
// Coro Control Flow Macros
// ===========================================================================

/**
 * @brief Mark the beginning of a coroutine body.
 *
 * Must be matched by a corresponding @c CORO_END, @c CORO_END_SUCCESS,
 * @c CORO_END_FAILURE, or @c CORO_END_WITH at the end of the coroutine.
 */
#define CORO_BEGIN()                                                                                                   \
  switch (ctx.resumePoint_())                                                                                          \
  {                                                                                                                    \
  case Hbs::Context::RESUME_RESET: ctx.setState_(Hbs::Context::STATE_NAME_START);

// internal common yield helper
#define HBS__CORO_YIELD_STATUS(status_, id_) HBS__STATEMENT(ctx.setResumePoint_(id_); ctx.setLastStatus_(status_); return status_; case id_:)

/**
 * Yield with a status passed in as a parameter.
 *
 * This form is very unlikely to be used in normal code and would usually
 * only be used if the status of some called function was being passed through.
 *
 * Caution: yielding SUCCESS or FAILURE implies the coro can cycle so you need
 * to ensure that you can resume immediately after the yield statement in such
 * circumstances.
 */
#define CORO_YIELD_STATUS(status_) HBS__CORO_YIELD_STATUS(status_, __COUNTER__)

/**
 * Yield control back to the caller after performing one tick's worth of work.
 *
 * this is the standard yield and by far the most common form used
 */ 
#define CORO_YIELD() CORO_YIELD_STATUS(Hbs::RUNNING)

/**
 * @brief Yield control back to the caller with a SUCCESS status.
 *
 * This form exists to support the case where a behaviour signals success but
 * must be able to re-execute the terminal success code if ticked again on the
 * next cycle. Typical usage:
 * @code
 * if (failureCondition)
 * {
 *   while (true)
 *   {
 *     failureStateCodeToRepeat;
 *     CORO_YIELD_FAILURE();
 *   }
 * }
 * @endcode
 */
#define CORO_YIELD_SUCCESS() CORO_YIELD_STATUS(Hbs::SUCCESS)

/**
 * @brief Yield control back to the caller with a FAILURE status.
 *
 * This form exists to support the case where a behaviour signals failure but
 * must be able to re-execute the terminal failure code if ticked again on the
 * next cycle. See @c CORO_YIELD_SUCCESS for a usage example.
 */
#define CORO_YIELD_FAILURE() CORO_YIELD_STATUS(Hbs::FAILURE)

// internal helpers
#define HBS__ASSERT_NOT_RUNNING(status_) { auto tmpStatus_ = (status_); HBS_ASSERT(tmpStatus_ != Hbs::RUNNING); }
#define HBS__CORO_RETURN(guard_, status_)                                                                              \
  HBS__STATEMENT(guard_ ctx.setResumePoint_(Hbs::Context::RESUME_RESET); ctx.setLastStatus_(status_); return status_)

/**
 * @brief Return SUCCESS from the coroutine immediately.
 *
 * May be used anywhere in the coroutine body, but not after @c CORO_END.
 */
#define CORO_SUCCESS() HBS__CORO_RETURN( , Hbs::SUCCESS)

/**
 * @brief Return FAILURE from the coroutine immediately.
 *
 * May be used anywhere in the coroutine body, but not after @c CORO_END.
 */
#define CORO_FAILURE() HBS__CORO_RETURN( , Hbs::FAILURE)

/**
 * @brief Return a dynamic status value from the coroutine immediately.
 *
 * Useful for propagating a status received from a child or helper function.
 * Only SUCCESS or FAILURE are valid; passing RUNNING is a runtime assertion
 * failure — use @c CORO_YIELD instead.
 */
#define CORO_RETURN(status_) HBS__CORO_RETURN(HBS__ASSERT_NOT_RUNNING(status_), Hbs::FAILURE)


// internal helper
#define HBS__CORO_END(guardCode_, status_)                                                                             \
  }                                                                                                                    \
  guardCode_ ctx.setResumePoint_(Hbs::Context::RESUME_RESET);                                                          \
  ctx.setLastStatus_(status_);                                                                                         \
  return status_;

/**
 * @brief Close the coroutine body and return SUCCESS.
 *
 * Must appear at the very end of the coroutine body to match @c CORO_BEGIN.
 * @note Must be used as a standalone statement — not inside an @c if or
 *       similar construct.
 */
#define CORO_END_SUCCESS() HBS__CORO_END( , Hbs::SUCCESS)

/**
 * @brief Close the coroutine body and return FAILURE.
 *
 * Must appear at the very end of the coroutine body to match @c CORO_BEGIN.
 * @note Must be used as a standalone statement — not inside an @c if or
 *       similar construct.
 */
#define CORO_END_FAILURE() HBS__CORO_END( , Hbs::FAILURE)

/**
 * @brief Close the coroutine body and return a dynamic status value.
 *
 * Must appear at the very end of the coroutine body to match @c CORO_BEGIN.
 * Only SUCCESS or FAILURE are valid; RUNNING triggers a runtime assertion.
 * @note Must be used as a standalone statement — not inside an @c if or
 *       similar construct.
 */
#define CORO_END_WITH(status_) HBS__CORO_END(HBS__ASSERT_NOT_RUNNING(status_), status_)

/**
 * @brief Close the coroutine body and return SUCCESS.
 *
 * Synonym for @c CORO_END_SUCCESS. Must appear at the very end of the
 * coroutine body to match @c CORO_BEGIN.
 * @note Must be used as a standalone statement — not inside an @c if or
 *       similar construct.
 */
#define CORO_END() CORO_END_SUCCESS()

/**
 * @brief Mark a named stage of progress within a coroutine.
 *
 * Records the current coroutine "state" — a named stage that may last for
 * several ticks and can have an associated duration for diagnostics and
 * tracing purposes.
 */
#define CORO_STATE(stateIdent_) ctx.setCoroState_(#stateIdent_,__LINE__)

// CORO idioms

/**
 * @brief Tick a child task repeatedly, yielding each tick, until it completes.
 *
 * A yield is inserted after every tick of the child, including the final tick
 * that returns SUCCESS or FAILURE (the "tail suspend always" pattern). This
 * prevents accidentally performing two units of work — the awaited task and
 * the statement following @c CORO_AWAIT — within a single behaviour tick.
 *
 * If the extra tail yield is undesirable, use @c CORO_AWAIT_NO_TAIL_YIELD
 * instead.
 */
#define CORO_AWAIT(task_) HBS__STATEMENT(while ((task_) == Hbs::RUNNING) { CORO_YIELD(); } CORO_YIELD();)

/**
 * @brief Tick a child task once for each caller tick, yielding each time, until it completes,
 *        but without yielding after the terminal status.
 *
 * Like @c CORO_AWAIT but without the trailing yield after the child returns
 * SUCCESS or FAILURE. Use this only when you are certain that skipping the
 * tail yield is safe.
 */
#define CORO_AWAIT_NO_TAIL(task_) HBS__STATEMENT(while ((task_) == Hbs::RUNNING) CORO_YIELD();)

/**
 * @brief Tick a statement or block repeatedly until a condition becomes true.
 *
 * Executes @p stmtOrBlock_ and yields on each tick for as long as @p cond_
 * is false. Once @p cond_ is true the body does not execute and control
 * passes immediately to the next statement — there is no tail yield.
 *
 * This is a convenience macro that automatically inserts @c CORO_YIELD. It
 * is suitable for short code blocks; longer blocks may affect line-number
 * attribution in a debugger.
 */
#define CORO_UNTIL(cond_, stmtOrBlock_)                                                                                \
  HBS__STATEMENT(while (!(cond_)) {                                                                                    \
    stmtOrBlock_;                                                                                                      \
    CORO_YIELD();                                                                                                      \
  })

/**
 * @brief Tick a statement or block once for each caller tick while a condition remains true.
 *
 * Executes @p stmtOrBlock_ and yields on each tick for as long as @p cond_
 * is true. Once @p cond_ is false the body does not execute and control
 * passes immediately to the next statement — there is no tail yield.
 *
 * This is a convenience macro that automatically inserts @c CORO_YIELD. It
 * is suitable for short code blocks; longer blocks may affect line-number
 * attribution in a debugger.
 */
#define CORO_WHILE(cond_, stmtOrBlock_)                                                                                \
  HBS__STATEMENT(while (cond_) {                                                                                       \
    stmtOrBlock_;                                                                                                      \
    CORO_YIELD();                                                                                                      \
  })



// ===========================================================================
// Section-tag macros
// ===========================================================================
//
// ARGS, VARS, PARAMS, LOADS_PARAMS are all optional in CORO_DEFINE /
// HSM_DEFINE. ARGS optional in CORO_DECLARE / HSM_DECLARE.  
// Each macro wraps its content in a tagged tuple so the outer macro can detect
// which sections are present.
//
// Elements inside each section are bare tuples:
//   (Type, Name, DefaultVal)   — with default (ARGS in DECLARE and inline DEFINE only, 
//                                VARS, PARAMS/LOADS_PARAMS in any DEFINE)
//   (Type, Name)               — without default (ARGS in out-of-class DEFINE)
//
// The HBS__ARG_FIELD / HBS__VAR_STRUCT_FIELD / HBS__PARAM_STRUCT_FIELD helpers
// in HbsMacroUtils.h unpack these tuples and emit the
// initialiser only when a DefaultVal is present.

#define ARGS(...)           (HBS_ARGS_TAG,   __VA_ARGS__)
#define VARS(...)           (HBS_VARS_TAG,   __VA_ARGS__)
#define PARAMS(...)         (HBS_PARAMS_TAG, __VA_ARGS__)
#define LOADS_PARAMS(...)   (HBS_LPARAMS_TAG, __VA_ARGS__)


// ===========================================================================
// Shared building-block macros (used by both CORO and HSM DECLARE/DEFINE)
// ===========================================================================

// HBS__TASK_MEMBERS_DECLARE - class members for the DECLARE (split) pattern.
// Only ARGS are known here; VARS and PARAMS are defined in the out-of-class
// DEFINE. _Vars and _Params are empty placeholders; _paramsPtr holds the real
// params lazily.

#define HBS__TASK_MEMBERS_DECLARE(taskName_, ...)                              \
  /* 1. Context Member */                                                      \
  Hbs::Context HBS__JOIN(taskName_, _ctx){hbsEnv, #taskName_};                 \
                                                                               \
  /* 2. Args Struct (from ARGS section only) */                                \
  struct HBS__JOIN(taskName_, _Args) {                                         \
    HBS__MAP_ARGS(HBS__ARG_FIELD, , __VA_ARGS__)                               \
  };                                                                           \
                                                                               \
  /* 3. Vars placeholder - real _Vars defined in hbs_detail namespace by       \
   * out-of-class DEFINE */                                                    \
  struct HBS__JOIN(taskName_, _Vars) : public Hbs::VarStorage {};              \
                                                                               \
  /* 4. Params placeholder + opaque storage ptr - real _Params in hbs_detail namespace */ \
  struct HBS__JOIN(taskName_, _Params) {};                                     \
  std::unique_ptr<Hbs::VarStorage> HBS__JOIN(taskName_, _paramsPtr)

// HBS__TASK_MEMBERS — class members for the inline DEFINE pattern.
// Full VARS and PARAMS structs with fields; direct _params member.

#define HBS__TASK_MEMBERS(taskName_, ...)                                                                              \
  /* 1. Context Member */                                                                                              \
  Hbs::Context HBS__JOIN(taskName_, _ctx){hbsEnv, #taskName_};                                                         \
                                                                                                                       \
  /* 2. Args Struct */                                                                                                 \
  struct HBS__JOIN(taskName_, _Args)                                                                                   \
  {                                                                                                                    \
    HBS__MAP_ARGS(HBS__ARG_FIELD, , __VA_ARGS__)                                                                       \
  };                                                                                                                   \
                                                                                                                       \
  /* 3. Vars Struct */                                                                                                 \
  struct HBS__JOIN(taskName_, _Vars) : public Hbs::VarStorage                                                          \
  {                                                                                                                    \
    HBS__MAP_VARS(HBS__VAR_STRUCT_FIELD, , __VA_ARGS__)                                                                \
  };                                                                                                                   \
                                                                                                                       \
  /* 4. Params Struct & Member */                                                                                      \
  struct HBS__JOIN(taskName_, _Params) : public Hbs::VarStorage                                                        \
  {                                                                                                                    \
    HBS__MAP_PARAMS(HBS__PARAM_STRUCT_FIELD, , __VA_ARGS__)                                                            \
    /* remove the following line if LOAD_PARAMS is not supported */                                                    \
    HBS__PARAMS_LOAD_CONFIG(HBS__HAS_LPARAMS(__VA_ARGS__), __VA_ARGS__)                                                \
  };                                                                                                                   \
  std::unique_ptr<Hbs::VarStorage> HBS__JOIN(taskName_, _params)

// HBS__TASK_ZERO_ARG_WRAPPER — emits the zero-arg template wrapper (item 5).
// Identical for CORO and HSM, and for DECLARE vs inline DEFINE.

#define HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                          \
  template <typename ArgsType_ = HBS__JOIN(taskName_, _Args)>                                                                     \
  Hbs::Status taskName_()                                                                                              \
    requires std::is_default_constructible_v<ArgsType_>                                                                \
  {                                                                                                                    \
    return taskName_(ArgsType_{});                                                                                     \
  }

// helper for initializing Params struct (loading from config if appropriate)
#define HBS__PARAMS_LAZY_INIT(taskName_, ParamsType_)                                                                  \
  if (!HBS__JOIN(taskName_, _paramsPtr))                                                                               \
  {                                                                                                                    \
    auto *_p = new ParamsType_();                                                                                      \
    _p->loadConfigFile(hbsEnv.configPrefix(), #taskName_);                                                                                          \
    HBS__JOIN(taskName_, _paramsPtr).reset(_p);                                                                        \
  }                                                                                                                    \
  [[maybe_unused]] auto &params = *static_cast<ParamsType_ *>(HBS__JOIN(taskName_, _paramsPtr).get())

// for declaring a delegator task that calls a delegate (See below). Delegators are useful when tasks should reuse the
// same code but need different contexts.
// (Like TASK_MEMBERS_DECLARE with no Vars)
#define HBS__DELEGATOR_MEMBERS(taskName_, ...)                                                                         \
  /* 1. Context Member */                                                                                              \
  Hbs::Context HBS__JOIN(taskName_, _ctx){hbsEnv, #taskName_};                                                                    \
                                                                                                                       \
  /* 2. Args Struct */                                                                                                 \
  struct HBS__JOIN(taskName_, _Args)                                                                                              \
  {                                                                                                                    \
    HBS__MAP_ARGS(HBS__ARG_FIELD, , __VA_ARGS__)                                                                       \
  };                                                                                                                   \
                                                                                                                       \
  /* 3. No _Vars — deliberately omitted to prevent conflict with delegate Vars */                                      \
                                                                                                                       \
  /* 4. Params placeholder + storage ptr */                                                                            \
  struct HBS__JOIN(taskName_, _Params) {};                                                                                        \
  std::unique_ptr<Hbs::VarStorage> HBS__JOIN(taskName_, _paramsPtr)

// for declaring the delegate task that a delegator calls
// (Like TASK_MEMBERS_DECLARE with no context)
#define HBS__DELEGATE_MEMBERS(taskName_, ...)                                                                          \
  /* 1. No Context Member — delegate borrows ctx from its wrapper caller */                                            \
                                                                                                                       \
  /* 2. Args Struct */                                                                                                 \
  struct HBS__JOIN(taskName_, _Args)                                                                                              \
  {                                                                                                                    \
    HBS__MAP_ARGS(HBS__ARG_FIELD, , __VA_ARGS__)                                                                       \
  };                                                                                                                   \
                                                                                                                       \
  /* 3. Vars placeholder — real _Vars defined in hbs_detail by DEFINE_DELEGATE */                                      \
  struct HBS__JOIN(taskName_, _Vars) : public Hbs::VarStorage                                                                     \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  /* 4. Params placeholder + storage ptr */                                                                            \
  struct HBS__JOIN(taskName_, _Params)                                                                                            \
  {                                                                                                                    \
  };                                                                                                                   \
  std::unique_ptr<Hbs::VarStorage> HBS__JOIN(taskName_, _paramsPtr)



// HBS__TASK_WRAPPER — emits the public taskName_(const Args&) wrapper for all tasks.
// Optional codeBeforeTrace_ is used to differentiate CORO from HSM where needed.
#define HBS__TASK_WRAPPER(taskName_, codeBeforeTrace_, ...)                                                            \
  Hbs::Status taskName_(const HBS__JOIN(taskName_, _Args) &args)                                                       \
  {                                                                                                                    \
    HBS__JOIN(taskName_, _ctx).beginTick_();                                                                           \
    codeBeforeTrace_                                                                                                   \
    HBS__JOIN(taskName_, _ctx).beginTrace_();                                                                          \
    HBS__JOIN(taskName_, _bodyCode)(HBS__JOIN(taskName_, _ctx), args __VA_OPT__(,) __VA_ARGS__);                       \
    HBS__JOIN(taskName_, _ctx).endTrace_();                                                                            \
    return HBS__JOIN(taskName_, _ctx).lastStatus();                                                                    \
  }


// HBS__CORO_WRAPPER — emits the public taskName_(const Args&) wrapper for coroutines.
// Used by CORO_DECLARE (out-of-class split pattern).
// Does NOT call prepareVars_ or pass params — the out-of-class _bodyCode shim does both.
// Calls _bodyCode with the fixed 2-parameter signature (ctx, args) — no VARS, no params.
#define HBS__CORO_WRAPPER(taskName_)                                                                                   \
  HBS__TASK_WRAPPER(                                                                                                   \
      taskName_,                                                                                                       \
      /* Coro Hold-Terminal-Status Semantics: Once terminal, return cached status without re-executing */              \
      /* if (HBS__JOIN(taskName_, _ctx).finished()) return HBS__JOIN(taskName_, _ctx).lastStatus(); */                 \
      )

// HBS__CORO_WRAPPER_INLINE — wrapper for inline CORO_DEFINE (no split declaration).
// Calls prepareVars_<HBS__JOIN(taskName_, _Vars)>() here because the inline _Vars struct is
// defined with the correct fields by HBS__TASK_MEMBERS in the same expansion.
#define HBS__CORO_WRAPPER_INLINE(taskName_)                                                                            \
  HBS__TASK_WRAPPER(                                                                                                   \
      taskName_,                                                                                                       \
      /* Coro Hold-Terminal-Status Semantics: Once terminal, return cached status without re-executing */              \
      /* if (HBS__JOIN(taskName_, _ctx).finished()) return HBS__JOIN(taskName_, _ctx).lastStatus(); */                 \
      HBS__JOIN(taskName_, _ctx).prepareVars_<HBS__JOIN(taskName_, _Vars)>();                                          \
      [[maybe_unused]] auto *varsPtr = HBS__JOIN(taskName_, _ctx).getVars_<HBS__JOIN(taskName_, _Vars)>();             \
      HBS__PARAMS_LAZY_INIT(taskName_, HBS__JOIN(taskName_, _Params));,                                                \
      params)

// HBS__HSM_WRAPPER — emits the public taskName_(const Args&) wrapper for HSMs.
// Used by HSM_DECLARE (out-of-class split pattern).
// Does NOT call prepareVars_ or pass params — the out-of-class _bodyCode shim does both.
// Calls _bodyCode with the fixed 2-parameter signature (ctx, args) — no VARS, no params.

#define HBS__HSM_WRAPPER(taskName_)                                                                                    \
  HBS__TASK_WRAPPER(taskName_, )

// HBS__HSM_WRAPPER_INLINE — wrapper for inline HSM_DEFINE (no split declaration).
// Calls prepareVars_<HBS__JOIN(taskName_, _Vars)>() here because the inline _Vars struct is correct.

#define HBS__HSM_WRAPPER_INLINE(taskName_)                                                                             \
  HBS__TASK_WRAPPER(taskName_,                                                                                         \
                    HBS__JOIN(taskName_, _ctx).prepareVars_<HBS__JOIN(taskName_, _Vars)>();                            \
                    [[maybe_unused]] auto *varsPtr = HBS__JOIN(taskName_, _ctx).getVars_<HBS__JOIN(taskName_, _Vars)>(); \
                    HBS__PARAMS_LAZY_INIT(taskName_, HBS__JOIN(taskName_, _Params));,                                  \
                    params)

// HBS__TASK_BODYCODE_SIG — fixed 2-parameter _bodyCode signature for coroutines and HSMs.
// Used by CORO/HSM_DECLARE (forward declaration) and by the out-of-class dispatch shim.
// No VARS or params — both are handled inside the shim using hbs_detail namespace types.
// No trailing semicolon — caller appends either ";" (DECLARE) or "{ body }" (shim def).

#define HBS__TASK_BODYCODE_SIG(taskName_, retType_)                                                                    \
  retType_ HBS__JOIN(taskName_, _bodyCode)(Hbs::Context &ctx, [[maybe_unused]] const HBS__JOIN(taskName_, _Args) &args)

// HBS__TASK_BODYCODE_SIG_INLINE — full VARS-expanded _bodyCode signature for inline DEFINE.
// Used only by HBS__TASK_DEFINE_INLINE where declaration and definition are in the same place,
// so no forward-declaration mismatch is possible.

#define HBS__TASK_BODYCODE_SIG_INLINE(taskName_, retType, ...)                                                         \
  retType_ HBS__JOIN(taskName_, _bodyCode)(                                                                                       \
      Hbs::Context &ctx, [[maybe_unused]] const HBS__JOIN(taskName_, _Args) &args,                                                \
      [[maybe_unused]] const HBS__JOIN(taskName_, _Params) &params HBS__MAP_VARS(HBS__VAR_ARG, , __VA_ARGS__))


// --- Out-of-class TASK (CORO/HSM) DEFINE implementation ---
//
// Wrapper dispatch technique inspired by CABSL (https://github.com/bhuman/CABSL).
//
// Problem: XXX_DECLARE (in the .h) only knows ARGS, so it forward-declares
// _bodyCode with a fixed 2-parameter signature (ctx, args).  VARS and PARAMS
// are unknown at DECLARE time and cannot appear in the forward declaration.
// But the user's body needs VARS as individual typed references and PARAMS as
// a typed struct.
//
// Solution: emit three things in the out-of-class DEFINE —
//
//   1. A private namespace hbs_detail_##className_ containing:
//      - HBS__JOIN(taskName_, _Vars)  (inherits VarStorage, has the real VARS fields)
//      - HBS__JOIN(taskName_, _Params) (inherits VarStorage, has the real PARAMS fields)
//      - HBS__JOIN(taskName_, _Wrapper) (inherits className_, declares _impl with full sig)
//
//   2. The definition of className_::HBS__JOIN(taskName_, _bodyCode) (the fixed 2-param
//      method forward-declared by CORO_DECLARE).  It lazily initialises
//      _paramsPtr with the namespace _Params type, calls prepareVars_ with the
//      namespace _Vars type, then reinterpret_casts `this` to the wrapper struct
//      and calls _impl, forwarding params and all VARS by reference.
//
//   3. Opens the wrapper struct's _impl signature for the user's { body }.
//      Uses unqualified HBS__JOIN(taskName_, _Args) (resolved via _Wrapper's base class)
//      to avoid protected-access errors when DECLARE is in a protected section.
//
// The reinterpret_cast is "safe" with current compilers because the wrapper struct adds no data members;
// it only adds methods, so its layout is identical to className_.

#define HBS__TASK_DEFINE_OUTOFCLASS(className_, taskName_, retType_, varsDecl_, varsSetup_, ...)                       \
  /* 1. Private namespace: _Vars, _Params with real fields, plus wrapper struct */                                     \
  namespace HBS__JOIN(hbs_detail_, className_)                                                                         \
  {                                                                                                                    \
    varsDecl_                                                                                                          \
                                                                                                                       \
    struct HBS__JOIN(taskName_, _Params) : public Hbs::VarStorage                                                      \
    {                                                                                                                  \
      HBS__MAP_PARAMS(HBS__PARAM_STRUCT_FIELD, , __VA_ARGS__)                                                          \
      /* remove the following line if LOAD_PARAMS is not supported */                                                  \
      HBS__PARAMS_LOAD_CONFIG(HBS__HAS_LPARAMS(__VA_ARGS__), __VA_ARGS__)                                              \
    };                                                                                                                 \
    struct HBS__JOIN(taskName_, _Wrapper) : public className_                                                          \
    {                                                                                                                  \
      retType_ HBS__JOIN(taskName_,_impl)(Hbs::Context & ctx,                                                          \
                                [[maybe_unused]] const className_::HBS__JOIN(taskName_, _Args) & args,                 \
                                [[maybe_unused]] HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Params) &   \
                                params HBS__MAP_VARS(HBS__VAR_ARG, , __VA_ARGS__));                                    \
    };                                                                                                                 \
  }                                                                                                                      \
  /* 2. Definition of the fixed-signature _bodyCode declared by CORO_DECLARE */                                          \
  /* Lazily initialises _paramsPtr and prepareVars_ with the correct namespace types */                                  \
  retType_ className_::HBS__JOIN(taskName_, _bodyCode)(                                                                  \
      Hbs::Context & ctx, [[maybe_unused]] const className_::HBS__JOIN(taskName_, _Args) & args)                         \
  {                                                                                                                      \
    if (!HBS__JOIN(taskName_, _paramsPtr))                                                                               \
    {                                                                                                                    \
      auto* p = new HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Params)();                                 \
      p->loadConfigFile(hbsEnv.configPrefix(), #taskName_);                                                              \
      HBS__JOIN(taskName_, _paramsPtr).reset(p);                                                                         \
    }                                                                                                                    \
    auto &params = *static_cast<HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Params) *>(                    \
        HBS__JOIN(taskName_, _paramsPtr).get());                                                                         \
                                                                                                                         \
    varsSetup_                                                                                                           \
                                                                                                                         \
    return reinterpret_cast<HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Wrapper) *>(this)                  \
            ->HBS__JOIN(taskName_, _impl)(ctx, args, params HBS__MAP_VARS(HBS__VAR_REF, , __VA_ARGS__));                 \
  }                                                                                                                      \
  /* 3. Open _impl signature — user's { body } attaches here */                                                        \
  /* Use unqualified HBS__JOIN(taskName_, _Args)/_Params: resolved via _Wrapper's base class, */                       \
  /* avoiding protected-access errors from explicit className_:: qualification */                                      \
  retType_ HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Wrapper)::HBS__JOIN(taskName_, _impl)(            \
      Hbs::Context & ctx,                                                                                              \
      [[maybe_unused]] const HBS__JOIN(taskName_, _Args) & args,                                                       \
      [[maybe_unused]] HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Params) & params                      \
      HBS__MAP_VARS(HBS__VAR_ARG, , __VA_ARGS__))


// Normal task — varsDecl_ emits _Vars struct, varsSetup_ calls prepareVars_
#define HBS__VARS_DECL(taskName_, ...)                                                                                 \
  struct HBS__JOIN(taskName_, _Vars) : public Hbs::VarStorage                                                                     \
  {                                                                                                                    \
    HBS__MAP_VARS(HBS__VAR_STRUCT_FIELD, , __VA_ARGS__)                                                                \
  };


#define HBS__VARS_SETUP(className_, taskName_)                                                                         \
  ctx.prepareVars_<HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Vars)>();                                                       \
  [[maybe_unused]] auto *varsPtr = ctx.getVars_<HBS__JOIN(hbs_detail_, className_)::HBS__JOIN(taskName_, _Vars)>();



/**
 * HBS_INVOKE_DELEGATE: Call a delegate from within a wrapper body.
 *
 * Hides the _bodyCode/_Args naming convention from the wrapper author.
 * ctx is passed through automatically — it is the wrapper's own ctx,
 * which the delegate uses for all state, Vars, and timing.
 *
 * For delegates with no args:
 *   HBS_INVOKE_DELEGATE(armObstacleAvoidanceArm)
 *
 * For delegates with args (designated initialisers for the args not
 * fixed by the wrapper itself):
 *   HBS_INVOKE_DELEGATE(armObstacleAvoidanceArm, {.arm = Arms::left})
 *   HBS_INVOKE_DELEGATE(armObstacleAvoidanceArm, {.arm = Arms::right,
 *                                                 .distance = args.distance})
 */
#define HBS_INVOKE_DELEGATE(delegateName_, ...)                                                                        \
  HBS__JOIN(delegateName_, _bodyCode)(ctx, HBS__JOIN(delegateName_, _Args) __VA_ARGS__ )

// ===========================================================================
// CORO_DECLARE and CORO_DEFINE
// ===========================================================================
//

/**
 * Declares a coro inside the executor/container class.
 *
 * Emits everything needed inside the executor class body except
 * the _bodyCode function body opening. Use this when the body will be defined
 * separately out-of-class via CORO_DEFINE((ClassName,taskName),...).
 *
 * @note CORO_DECLARE accepts only ARGS — never VARS or DEFINES_PARAMS. VARS and
 * DEFINES_PARAMS belong exclusively in the out-of-class CORO_DEFINE.
 *
 * Usage (inside class body):
 * @code
 *   CORO_DECLARE( myTask, ARGS( (float, speed, 1.0f) ) )
 * @endcode
 *
 * The ARGS section is optional.
 */

#define CORO_DECLARE(taskName_, ...)                                                                                   \
  HBS__TASK_MEMBERS_DECLARE(taskName_, __VA_ARGS__);                                                                   \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__CORO_WRAPPER(taskName_)                                                                                         \
  HBS__TASK_BODYCODE_SIG(taskName_, Hbs::Status);


/**
 * Defines the variables, configurable parameters, and body of the Coro.
 *
 * Two forms depending on the first (tuple) argument:
 *
 * @code
 *   CORO_DEFINE((taskName), ...)
 * @endcode
 * Inline definition inside the executor class body. Equivalent to CORO_DECLARE
 * followed immediately by the _bodyCode function signature opening. The { body }
 * follows the macro.
 *
 * @code
 *   CORO_DEFINE((ClassName, taskName), ...)
 * @endcode
 * Out-of-class definition (e.g. in a .cpp file). Emits only the
 * ClassName::taskName_bodyCode(...) signature opening. The { body } follows the
 * macro. Defaults should be omitted from element tuples here.
 *
 * All section arguments (ARGS, VARS, DEFINES_PARAMS) are optional.
 */

#define CORO_DEFINE(nameTuple_, ...)  HBS__CORO_DEFINE_DISPATCH(nameTuple_, __VA_ARGS__)

// --- Inline CORO_DEFINE implementation ---
// Emits all members + wrappers + _bodyCode signature opening (no semicolon).
// Uses the full VARS-expanded inline signature — no forward-declaration mismatch possible.

#define HBS__CORO_DEFINE_INLINE(taskName_, ...)                                                                        \
  HBS__TASK_MEMBERS(taskName_, __VA_ARGS__);                                                                           \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__CORO_WRAPPER_INLINE(taskName_)                                                                                  \
  HBS__TASK_BODYCODE_SIG_INLINE(taskName_, Hbs::Status, __VA_ARGS__)

// out of class define for coroutines
#define HBS__CORO_DEFINE_OUTOFCLASS(className_, taskName_, ...)                                                        \
  HBS__TASK_DEFINE_OUTOFCLASS(className_, taskName_, Hbs::Status, \
                              HBS__VARS_DECL(taskName_, __VA_ARGS__),  \
                              HBS__VARS_SETUP(className_, taskName_), __VA_ARGS__)

/**
 * CORO_DECLARE_DELEGATOR: Declares a delegator for a coroutine delegate task in the executor class body.
 *
 * A delegator:
 *   - Owns a context (and therefore independent state tracking)
 *   - May have Args and Params
 *   - Can NOT have Vars (since these would conflict with the delegate's Vars on the same context)
 *   - Has a body (must be defined out-of-class) that forwards to a delegate, adding/setting args if needed
 *
 * Usage (inside class body):
 *   CORO_DECLARE_DELEGATOR(delegatorToSomeCoro, ARGS(...), DEFINES_PARAMS(...))
 */
#define CORO_DECLARE_DELEGATOR(taskName_, ...)                                                                         \
  HBS__DELEGATOR_MEMBERS(taskName_, __VA_ARGS__);                                                                      \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__CORO_WRAPPER(taskName_)                                                                                         \
  HBS__TASK_BODYCODE_SIG(taskName_, Hbs::Status);

/**
 * CORO_DEFINE_DELEGATOR: Defines coroutine delegator task body (must be out of class)
 */

#define CORO_DEFINE_DELEGATOR(nameTuple_, ...)                                                                         \
  HBS__TASK_DEFINE_OUTOFCLASS(HBS__EVAL(HBS__TUPLE_CLASS nameTuple_), HBS__EVAL(HBS__TUPLE_TASK nameTuple_), Hbs::Status, , , __VA_ARGS__)

/**
 * CORO_DECLARE_DELEGATE: Declares coroutine delegate task in the executor class body.
 *
 * A delegate:
 *   - Has no context of its own - it must be called from a delegator which owns the context
 *   - May have Args, Vars, and Params
 *   - Has a body (must be defined out-of-class) that uses the delegators context
 *
 * Usage (inside class body):
 *   CORO_DECLARE_DELEGATE(someCoroDelegate, ARGS(...), VARS(...), DEFINES_PARAMS(...))
 */
#define CORO_DECLARE_DELEGATE(taskName_, ...)                                                                          \
  HBS__DELEGATE_MEMBERS(taskName_, __VA_ARGS__);                                                                       \
  HBS__TASK_BODYCODE_SIG(taskName_, Hbs::Status);

/**
 * CORO_DEFINE_DELEGATE: Defines coroutine delegate task body (must be out of class)
 */
#define CORO_DEFINE_DELEGATE(nameTuple_, ...) CORO_DEFINE(nameTuple_, __VA_ARGS__)

// ===========================================================================
// HSM Control Flow Macros
// ===========================================================================

/**
 * @brief Marker for the global transitions block.
 *
 * Place this block before any state declarations if used. Transitions
 * written here are evaluated on every tick regardless of the current state.
 *
 * @code
 * HSM_GLOBAL_TRANSITIONS { if (cond1) HSM_GOTO(someState); else ... }
 * @endcode
 */
#define HSM_GLOBAL_TRANSITIONS

/**
 * @brief Transition checks within a state body, skipped on the first entry
 *        tick.
 *
 * Prevents chains of transitions by not evaluating the block on the tick
 * that enters the state (except for the initial state). Write the transitions 
 * as the first block in any state.
 *
 * @code
 * HSM_TRANSITIONS { if (cond1) HSM_GOTO(someState); else ... }
 * @endcode
 */
#define HSM_TRANSITIONS if (ctx.transitionsEnabled_())

/**
 * @brief Entry code block — only executes on the first tick in a state.
 *
 * Write this as the second block in any state where one-time initialisation
 * is needed.
 *
 * @code
 * HSM_ON_ENTRY { oneTimeStateEntryCode... }
 * @endcode
 */
#define HSM_ON_ENTRY  if (ctx.isStateEntry())

/**
 * @brief Transition to another state.
 *
 * Use within @ref HSM_TRANSITIONS or @ref HSM_GLOBAL_TRANSITIONS only —
 * nowhere else. Usually written as:
 * @code
 * if (someCond) HSM_GOTO(someState);
 * @endcode
 * To obtain UML2 transition-action semantics, place action code before the
 * macro:
 * @code
 * if (someCond) { someActionCode...; HSM_GOTO(someState); }
 * @endcode
 * Action code must be run-to-completion and must not involve anything
 * time-consuming.
 */
#define HSM_GOTO(_target) goto _target

// Internal implementation
#define HBS__HSM_STATE_I(stateIdent_, stateAlias_, resumePoint_, status_, transitionsOnEntry_)                         \
  if (ctx.isAtStart_())                                                                                                \
    goto initial_state;                                                                                                \
  if ((ctx.isAtState_(resumePoint_)) && (ctx.isStateEntry()))                                                          \
  {                                                                                                                    \
    stateAlias_                                                                                                        \
    stateIdent_ : ctx.enterHsmState_(resumePoint_, #stateIdent_, status_, transitionsOnEntry_);                        \
  }                                                                                                                    \
  if (ctx.isAtState_(resumePoint_))

/**
 * @brief Define a normal (running) HSM state.
 *
 * @code
 * HSM_STATE(someIdentifier) { stateCode... }
 * @endcode
 */
#define HSM_STATE(stateIdent_) \
    HBS__HSM_STATE_I(stateIdent_, , __LINE__, Hbs::RUNNING, false)

/**
 * @brief Define the initial state of the HSM — there can only be one.
 *
 * The HSM enters this state on the very first tick.
 */
#define HSM_INITIAL_STATE(stateIdent_) \
    if (ctx.isAtState_(Hbs::Context::RESUME_RESET)) goto stateIdent_; \
    HBS__HSM_STATE_I(stateIdent_, initial_state:, __LINE__, Hbs::INITIAL, true)

/**
 * @brief Define a success terminal state.
 *
 * When the HSM reaches this state it returns @c Hbs::SUCCESS status to
 * its callers.
 */
#define HSM_SUCCESS_STATE(stateIdent_) \
    HBS__HSM_STATE_I(stateIdent_, , __LINE__, Hbs::SUCCESS, false)

/**
 * @brief Define a failure terminal state.
 *
 * When the HSM reaches this state it returns @c Hbs::FAILURE status to
 * its callers.
 */
#define HSM_FAILURE_STATE(stateIdent_) \
    HBS__HSM_STATE_I(stateIdent_, , __LINE__, Hbs::FAILURE, false)


// ===========================================================================
// HSM_DECLARE and HSM_DEFINE
// ===========================================================================

/**
 * Declares the HSM inside the executor/container class.
 *
 * Emits everything needed inside the executor class body except
 * the _bodyCode function body opening. Use this when the HSM body will be
 * defined separately out-of-class via HSM_DEFINE((ClassName,taskName),...).
 *
 * @note HSM_DECLARE accepts only ARGS — never VARS or DEFINES_PARAMS. VARS and
 * DEFINES_PARAMS belong exclusively in the out-of-class HSM_DEFINE.
 *
 * Usage (inside class body):
 * @code
 *   HSM_DECLARE( myHsm, ARGS( (float, speed, 1.0f) ) )
 * @endcode
 *
 * The ARGS section is optional.
 */

#define HSM_DECLARE(taskName_, ...)                                                                                    \
  HBS__TASK_MEMBERS_DECLARE(taskName_, __VA_ARGS__);                                                                   \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__HSM_WRAPPER(taskName_)                                                                                          \
  HBS__TASK_BODYCODE_SIG(taskName_, void);


/**
 * Defines the variables, configurabla parameters and body of a HSM.
 *
 * Two forms depending on the first (tuple) argument:
 *
 * @code
 *   HSM_DEFINE((taskName), ...)
 * @endcode
 * Inline definition inside the executor class body.
 *
 * @code
 *   HSM_DEFINE((ClassName, taskName), ...)
 * @endcode
 * Out-of-class definition (e.g. in a .cpp file).
 *
 * All section arguments are optional.
 */

#define HSM_DEFINE(nameTuple_, ...)  HBS__HSM_DEFINE_DISPATCH(nameTuple_, __VA_ARGS__)

// --- Inline HSM_DEFINE implementation ---
// Emits all members + wrappers + _bodyCode signature opening (no semicolon).
// Uses the full VARS-expanded inline signature — no forward-declaration mismatch possible.

#define HBS__HSM_DEFINE_INLINE(taskName_, ...)                                                                         \
  HBS__TASK_MEMBERS(taskName_, __VA_ARGS__);                                                                           \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__HSM_WRAPPER_INLINE(taskName_)                                                                                   \
  HBS__TASK_BODYCODE_SIG_INLINE(taskName_, void, __VA_ARGS__)

// --- Out-of-class HSM_DEFINE implementation ---
#define HBS__HSM_DEFINE_OUTOFCLASS(className_, taskName_, ...)                                                         \
  HBS__TASK_DEFINE_OUTOFCLASS(className_, taskName_, void, \
                              HBS__VARS_DECL(taskName_, __VA_ARGS__),  \
                              HBS__VARS_SETUP(className_, taskName_), __VA_ARGS__)

/**
 * HSM_DECLARE_DELEGATOR: Declares a delegator for a HSM delegate task in the executor class body.
 *
 * A delegator:
 *   - Owns a context (and therefore independent state tracking)
 *   - May have Args and Params
 *   - Can NOT have Vars (since these would conflict with the delegate's Vars on the same context)
 *   - Has a body (must be defined out-of-class) that forwards to a delegate, adding/setting args if needed
 *
 * Usage (inside class body):
 *   HSM_DECLARE_DELEGATOR(delegatorToSomeHsm, ARGS(...), DEFINES_PARAMS(...))
 */
#define HSM_DECLARE_DELEGATOR(taskName_, ...)                                                                          \
  HBS__DELEGATOR_MEMBERS(taskName_, __VA_ARGS__);                                                                      \
  HBS__TASK_ZERO_ARG_WRAPPER(taskName_)                                                                                \
  HBS__HSM_WRAPPER(taskName_)                                                                                          \
  HBS__TASK_BODYCODE_SIG(taskName_, void);

/**
 * HSM_DEFINE_DELEGATOR: Defines a HSM delegator task body (must be out of class definition).
 */
#define HSM_DEFINE_DELEGATOR(nameTuple_, ...)                                                                         \
  HBS__TASK_DEFINE_OUTOFCLASS(HBS__TUPLE_CLASS nameTuple_, HBS__TUPLE_TASK nameTuple_, void, , , __VA_ARGS__)

/**
 * HSM_DECLARE_DELEGATE: Declares a HSM delegate task in the executor class body.
 *
 * A delegate:
 *   - Has now context of its own - it must be called from a delegator which owns the context
 *   - May have Args, Vars, and Params
 *   - Has a body (must be defined out-of-class) that uses the delegators context
 *
 * Usage (inside class body):
 *   HSM_DECLARE_DELEGATE(someHsmDelegate, ARGS(...), VARS(...), DEFINES_PARAMS(...))
 */
#define HSM_DECLARE_DELEGATE(taskName_, ...)     \
  HBS__DELEGATE_MEMBERS(taskName_, __VA_ARGS__); \
  HBS__TASK_BODYCODE_SIG(taskName_, void);

/**
 * HSM_DEFINE_DELEGATE: Defines a HSM delegate task body (must be out of class definition).
 */
#define HSM_DEFINE_DELEGATE(nameTuple_, ...)  \
  HSM_DEFINE(nameTuple_, __VA_ARGS__)

// ===========================================================================
// BT Sequence / Fallback Combinators and support macros
// ===========================================================================

/// cache the terminal status to prevent BT child rerunning - acts like BT memory semantics on a per child basis
#define HBS_HOLD(expr_)   ctx.applyHold_([&]() -> Hbs::Status { return (expr_); }, /* defer: */ false)

/// defer the terminal status by one tick (useful to prevent double actuations in a sequence of actuations).
/// HBS_DEFER implies HBS_HOLD semantics also, i.e. it does not rerun until the entire BT combinator terminates
#define HBS_DEFER(expr_)  ctx.applyHold_([&]() -> Hbs::Status { return (expr_); }, /* defer: */ true)


// BT_SEQUENCE support
// -------------------

// Applied to each step expression.
// SUCCESS  : fall through to next step (REACTIVE default — same tick advancement).
// FAILURE  : sequence fails immediately.
// RUNNING  : sequence running.
// INITIAL  : mapped to FAILURE (child opted out / not applicable).
#define HBS__BT_SEQ_STEP(expr_)                                                                                        \
  {                                                                                                                    \
    Hbs::Status s = (expr_);                                                                                           \
    if (s == Hbs::INITIAL)                                                                                             \
      s = Hbs::FAILURE;                                                                                                \
    if (s != Hbs::SUCCESS)                                                                                             \
      return s;                                                                                                        \
  }

/**
 * Reactive behaviour tree sequence combinator.
 *
 * Evaluates children left-to-right every tick with reactive semantics: a
 * succeeding child immediately passes control to the next child within the same
 * tick. Returns SUCCESS when all children succeed, FAILURE as soon as any child
 * fails, and RUNNING while a child is still running. A child returning INITIAL
 * is treated as FAILURE.
 *
 * Children are tasks or ordinary functions that return a Hbs::Status. Wrap with
 * HBS_HOLD to cache a terminal result across ticks (avoiding re-evaluation), or
 * HBS_DEFER to insert a one-tick gap before advancing — useful for children that
 * set actuation on their terminal tick.
 *
 * @warning At most one BT combinator may be evaluated per tick within a given
 * task context. Evaluating two combinators in the same tick is not supported
 * and will corrupt the internal state.
 *
 * @code
 * // condition re-evaluated reactively; action held once complete; actuating
 * // step deferred one tick to separate its actuation from the next child
 * auto result = BT_SEQUENCE(ballVisible(), 
 *                           HBS_HOLD(approach()),
 *                           HBS_DEFER(kick()));
 * @endcode
 */
#define BT_SEQUENCE(...)                                                                                               \
  (                                                                                                                    \
      [&]() -> Hbs::Status                                                                                             \
      {                                                                                                                \
        constexpr uint16_t combId = static_cast<uint16_t>(__COUNTER__);                                                \
        ctx.beginBtCombinator_(combId);                                                                                \
        HBS__MAPC(HBS__BT_SEQ_STEP, , __VA_ARGS__)                                                                     \
        return Hbs::SUCCESS;                                                                                           \
      }())


// ===========================================================================
// BT_FALLBACK helpers
// ===========================================================================

// Mirror of BT__SEQ_STEP: advances on FAILURE, stops on SUCCESS or RUNNING.
// INITIAL  : mapped to FAILURE (child opted out, fallback tries next).
#define HBS__BT_FALLBACK_STEP(expr_)  \
  {                               \
    Hbs::Status s = (expr_);      \
    if (s == Hbs::INITIAL)        \
      s = Hbs::FAILURE;           \
    if (s != Hbs::FAILURE)        \
      return s;                   \
  }

// ===========================================================================
// BT_FALLBACK
// ===========================================================================
/**
 * Reactive behaviour tree fallback combinator.
 *
 * Evaluates children left-to-right every tick with reactive semantics: a failing
 * child immediately passes control to the next child within the same tick.
 * Returns FAILURE when all children fail, SUCCESS as soon as any child succeeds,
 * and RUNNING while a child is still running. A child returning INITIAL is
 * treated as FAILURE (child not applicable; try the next).
 *
 * Children are tasks or ordinary functions that return a Hbs::Status. Wrap with
 * HBS_HOLD to cache a terminal result across ticks, or HBS_DEFER to insert a
 * one-tick gap before advancing.
 *
 * @warning At most one BT combinator may be evaluated per tick within a given
 * task context. Evaluating two combinators in the same tick is not supported
 * and will corrupt the internal state.
 *
 * @code
 * // try primary behaviour; if it fails, hold the fallback result once reached
 * auto result = BT_FALLBACK(HBS_HOLD(primaryAction()),
 *                           HBS_HOLD(fallbackAction()));
 * @endcode
 */
#define BT_FALLBACK(...)                                                                                               \
  (                                                                                                                    \
      [&]() -> Hbs::Status                                                                                             \
      {                                                                                                                \
        constexpr uint16_t combId = static_cast<uint16_t>(__COUNTER__);                                                \
        ctx.beginBtCombinator_(combId);                                                                                \
        HBS__MAPC(HBS__BT_FALLBACK_STEP, , __VA_ARGS__)                                                                \
        return Hbs::FAILURE;                                                                                           \
      }())


/*
// ===========================================================================
// Example Expansion
// ===========================================================================

// ---------------------------------------------------------------------------
// Pattern A — fully inline (no separate declaration needed):
//
// CORO_DEFINE( (myTask),
//   ARGS( (float, speed, 1.0f) ),
//   VARS( (int, step, 0) ),
//   DEFINES_PARAMS( (float, threshold, 0.5f) )
// )
// { CORO_BEGIN() ... CORO_END() }
//
// Generated Code (simplified):
//
// // 1. Context Member
// Hbs::Context myTask_ctx{hbsEnv, "myTask"};
//
// // 2. Args Struct
// struct myTask_Args { float speed {1.0f}; };
//
// // 3. Vars Struct
// struct myTask_Vars : public Hbs::VarStorage { int step {0}; };
//
// // 4. Params Struct & Member
// struct myTask_Params { float threshold {0.5f}; };
// myTask_Params myTask_params;
//
// // 5. Zero-arg wrapper
// template <typename = void>
// Hbs::Status myTask() requires std::is_default_constructible_v<myTask_Args>
// { return myTask(myTask_Args{}); }
//
// // 6. Public wrapper (inline — calls prepareVars_ and passes params directly)
// Hbs::Status myTask(const myTask_Args& args) {
//   myTask_ctx.beginTick_();
//   if (myTask_ctx.finished()) return myTask_ctx.lastStatus();
//   myTask_ctx.prepareVars_<myTask_Vars>();
//   auto* varsPtr = myTask_ctx.getVars_<myTask_Vars>();
//   auto status = myTask_bodyCode(myTask_ctx, args, myTask_params, varsPtr->step);
//   myTask_ctx.setLastStatus_(status);
//   return status;
// }
//
// // 7. _bodyCode signature opening — { body } follows
// Hbs::Status myTask_bodyCode(Hbs::Context& ctx,
//     [[maybe_unused]] const myTask_Args& args,
//     [[maybe_unused]] const myTask_Params& params,
//     [[maybe_unused]] int& step)
// { CORO_BEGIN() ... CORO_END() }

// ---------------------------------------------------------------------------
// Pattern B — split declare/define, definition in .cpp:
//
// IMPORTANT: CORO_DECLARE accepts only ARGS — never VARS or DEFINES_PARAMS.
// VARS and DEFINES_PARAMS belong exclusively in the out-of-class CORO_DEFINE.
//
// // In executor .h (inside class body):
// CORO_DECLARE( myTask, ARGS( (float, speed, 1.0f) ) )
//
// // In myTask.cpp (VARS/PARAMS here; defaults omitted from ARGS tuples):
// CORO_DEFINE( (MyExecutor, myTask),
//   ARGS( (float, speed) ),
//   VARS( (int, step) ),
//   DEFINES_PARAMS( (float, threshold) )
// )
// { CORO_BEGIN() ... CORO_END() }
//
// DECLARE emits (inside the class):
//   - myTask_ctx, myTask_Args (from ARGS), empty myTask_Vars placeholder,
//     empty myTask_Params placeholder, myTask_paramsPtr (unique_ptr<VarStorage>)
//   - zero-arg wrapper
//   - public wrapper: calls myTask_bodyCode(ctx, args)  [2-param, no VARS/params]
//   - forward declaration: Hbs::Status myTask_bodyCode(ctx, args);
//
// Out-of-class DEFINE emits (wrapper dispatch, inspired by CABSL):
//   namespace hbs_detail_MyExecutor {
//     struct myTask_Vars : VarStorage { int step; };
//     struct myTask_Params : VarStorage { float threshold; };
//     struct myTask_Wrapper : public MyExecutor {
//       Hbs::Status myTask_impl(ctx, args, myTask_Params& params, int& step);
//     };
//   }
//   // Shim: defines the 2-param _bodyCode, lazily inits params, calls _impl
//   Hbs::Status MyExecutor::myTask_bodyCode(ctx, args) {
//     if (!myTask_paramsPtr) myTask_paramsPtr.reset(new hbs_detail_MyExecutor::myTask_Params());
//     auto& params = *static_cast<hbs_detail_MyExecutor::myTask_Params*>(myTask_paramsPtr.get());
//     ctx.prepareVars_<hbs_detail_MyExecutor::myTask_Vars>();
//     auto* varsPtr = ctx.getVars_<hbs_detail_MyExecutor::myTask_Vars>();
//     return reinterpret_cast<hbs_detail_MyExecutor::myTask_Wrapper*>(this)
//         ->myTask_impl(ctx, args, params, varsPtr->step);
//   }
//   // User body attaches here:
//   Hbs::Status hbs_detail_MyExecutor::myTask_Wrapper::myTask_impl(
//       ctx, args, params, int& step)
//   { CORO_BEGIN() ... CORO_END() }
*/

