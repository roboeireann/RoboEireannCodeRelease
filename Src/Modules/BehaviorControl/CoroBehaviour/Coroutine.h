/*
 * @file: coroutine.h
 *
 * This file defines macros and classes for a basic stackless asymmetric
 * coroutines.
 *
 * Originally the implementation used Duff's device, somewhat inspired
 * by ProtoThreads (see http://dunkels.com/adam/pt/). Now the implementation
 * uses computed goto and local labels. For an example of this in action see
 * https://github.com/bxparks/AceRoutine/blob/develop/src/ace_routine/Coroutine.h)
 *
 *
 * In this implementation, a coroutine is a callable function that:
 *  - can save state and yield such that it may be resumed from where it
 *    left off later. Just like a normal function it may be declared to
 *    yield nothing (void) or yield a "return" value
 *  - can take parameters that may change each time it is called/resumed.
 *    It is up to the author of each coroutine function to decide whether
 *    it only pays attention to the parameters when it is started (just
 *    after reset), or on each resume
 *
 * A context object is used to keep track of the state of the coroutine
 * and where to resume it. For basic coroutines, this object is just a
 * parameter passed into the function and passed into each CR_XXX macro.
 * For coroutine tasks, the class itself is the context and the CRT_XXX macros
 * do not take a context parameter.
 *
 * There are two main strategies for implementing coroutine variables whose
 * values must persist between resumes of the coroutine:
 *  - use static local variables - this is simple but doesn't allow multiple
 *    independent instances of the coroutine, so it is not so good for reuse
 *  - make the persistent variables part of the context class. This is the
 *    strategy for coroutine tasks (and probably the best strategy generally)
 *
 *
 * A basic coroutine example with no persistent variables, no return value:
 *
 *   void myCoroFunction(BasicCoro ctx, other params...) {
 *     CR_BEGIN(ctx)
 *     while (someConditionNotSatisfed) {
 *       if (ctx.coroTime > OVERDUE_TIME)
 *         CR_FAILED(ctx);
 *       doSomething();
 *       CR_YIELD(ctx);
 *     }
 *     CR_END(ctx)
 *   }
 *
 * An example of a CoroTask with persistent variables and a return value
 *
 *   TODO
 *
 * @author: Rudi Villing
 */

#pragma once

// #include "MacroUtils.h"

#include "Platform/BHAssert.h"
#include "Tools/TextLogging.h"

#include "fmt/format.h"

#include <string>

typedef unsigned int CoroTime; // give flexibility to change how time is represented

/**
 * class used to implement central knowledge required by a group of coroutines
 */
class CoroEnv
{
public:
  inline CoroTime getCurrentFrameTime() { return currentFrameTime; }
  inline CoroTime getPrevFrameTime() { return prevFrameTime; }

  void updateTimes(CoroTime timeNow)
  {
    prevFrameTime = currentFrameTime;
    currentFrameTime = timeNow;
  }

private:
  CoroTime currentFrameTime = 0;
  CoroTime prevFrameTime = 0;
};

/**
 * class used to implement minimal context required for coroutine
 *
 * The BasicCoro implements a context that can be used by any function
 * to make it resumable.
 *
 * For variables to persist between resumes, either subclass BasicCoro
 * to add member variables to the context, or else add static local
 * variables to the function (but beware that makes the function non-re-entrant).
 */
class BasicCoro
{
public:
  /**
   * possible states that a Coroutine can be in.
   * A coroutine starts in the reset state.
   * YIELDED means continuing work in progress / incomplete
   * SUCCESS means task ended successfully
   * FAILED means task ended with failure
   */
  enum CoroState
  {
    RESET,
    YIELDED,
    SUCCESS,
    FAILED,
    RUNNING // only active while the coro is running
  };

  /**
   * construct a new coroutine context
   */
  BasicCoro(CoroEnv &env, const std::string &name = "") : mEnv(env), mCoroName(name) { reset(); }
  BasicCoro() = delete;

  virtual ~BasicCoro() = default;

  inline bool isYielded() const { return (mCoroState == YIELDED) && checkPrevRun(); }
  inline bool isSuccess() const { return (mCoroState == SUCCESS) && checkPrevRun(); }
  inline bool isFailure() const { return (mCoroState == FAILED) && checkPrevRun(); }
  inline bool isEnded() const { return ((mCoroState == SUCCESS) || (mCoroState == FAILED)) && checkPrevRun(); }
  inline bool isRunning() const { return mCoroState == RUNNING; } //only true inside the coro (i.e. current tick) so no time check needed
  inline bool isReset() const { return (mCoroState == RESET) || !checkPrevRun(); }

  /** return duration since last reset
   */
  inline CoroTime getCoroDuration() const { return mEnv.getCurrentFrameTime() - mCoroBeginTime; }

  /** get the time since the last checkpoint started
   */
  inline CoroTime getCheckpointDuration() const { return mEnv.getCurrentFrameTime() - mCheckpointTime; }

  inline const std::string &getName() const { return mCoroName; }
  inline const std::string &getCheckpointName() const { return mCheckpointName; }

  /*
   * get the number of times the coroutine has been called/continued
   * since it was last reset
   */
  inline unsigned int getRunCount() const { return mRunCount; }

  /**
   * explicitly reset the context so that the next time the coroutine
   * is called it will start at the beginning again.
   */
  virtual void reset()
  {
    // coro_line_ = 0;
    mResumePoint = nullptr;
    mRunCount = 0;
    mCoroState = RESET;

    mCoroPrevRunTime = 1;
    mCoroBeginTime = 0;
    mCheckpointTime = 0;
  }

public: // this section should ideally be PRIVATE/PROTECTED but must be public for macros
  // Everything in this section is really part of the private implementation
  // and uses a different name convention for that reason
  inline void internalSetRunning() { checkNotRunning(); mCoroState = RUNNING; }
  inline void internalSetSuccess() { checkRunning(); mCoroState = SUCCESS; }
  inline void internalSetFailed() { checkRunning(); mCoroState = FAILED; }
  inline void internalSetYielded(void *resumePoint)
  {
    checkRunning();
    mCoroState = YIELDED;
    internalSetResumePoint(resumePoint);
  }

  inline void *internalGetResumePoint() { return mResumePoint; }
  inline void internalSetResumePoint(void *resumePoint) { mResumePoint = resumePoint; }

  void *internalBeginRun()
  {
    // auto-reset if our current (prev) state is done or failed
    // or if this coro did not run in the previous cycle
    if ((mCoroState == FAILED) || (mCoroState == SUCCESS))
      reset();
    else if (!checkPrevRun())
    {
      // TLOGW(tlogger(), "missed a cycle: prev_run={}, prevFrame={}, currFrame={}", mCoroPrevRunTime,
      //         mEnv.getPrevFrameTime(), mEnv.getCurrentFrameTime());
      reset();
    }

    // if coroutine was reset, we now switch to running
    if (mCoroState == RESET)
    {
      mCoroBeginTime = mEnv.getCurrentFrameTime();
      internalSetCheckpoint("default");
    }

    mCoroPrevRunTime = mEnv.getCurrentFrameTime(); // only used above on next cycle
    ++mRunCount;

    mCoroState = RUNNING;

    return mResumePoint; // point to resume or nullptr if starting at the top
  }

  void internalSetCheckpoint(const std::string &name, int line = 0)
  {
    // fast check for repeat of the current checkpoint
    if ((line > 0) && (line == mCheckpointLine))
      return;

    mCheckpointTime = mEnv.getCurrentFrameTime();
    mCheckpointName = name;
    mCheckpointLine = line;
  }

private:
  CoroEnv &mEnv;
  void *mResumePoint = nullptr; // ptr to a locally scoped label
  CoroState mCoroState = RESET;
  unsigned int mRunCount = 0;
  CoroTime mCoroPrevRunTime = 1;   // the timestamp when this coro was last run
  CoroTime mCoroBeginTime = 0;      // the timestamp when this coro last started (after reset)
  CoroTime mCheckpointTime = 0; // the timestamp when this coro last checkpointed
  int mCheckpointLine = 0; // the line number from which the last checkpoint was set
  std::string mCoroName;
  std::string mCheckpointName; // set by run after reset

  DECL_TLOGGER_FN(tlogger, "BasicCoro", TextLogging::INFO)

  /// did the coro run in either the prev tick or the current tick
  bool checkPrevRun() const
  {
    return (mCoroPrevRunTime == mEnv.getPrevFrameTime()) || (mCoroPrevRunTime == mEnv.getCurrentFrameTime());
  }

  void checkRunning() const
  {
    if (mCoroState != RUNNING)
      FAIL("Coro Yield or Exit called from non-running coroutine - did you forget CR_BEGIN or CR_LOOP?");
  }

  void checkNotRunning() const
  {
    if (mCoroState == RUNNING)
      FAIL("CR_BEGIN called twice for the same coro instance - did you try to implement 2 coroutines in one class?");
  }
};

// ==========================================================================

/**
 * check that coroutines calls YIELD or EXIT and doesn't use return statement
 * or reach closing brace
 */
class BasicCoroScope
{
  BasicCoro& coro;
public:
  BasicCoroScope(BasicCoro& coro) : coro(coro) { }
  BasicCoroScope() = delete;

  ~BasicCoroScope()
  {
    // coro state is set to running in BasicCoro::internalBeginRun
    // if it wasn't changed, then the coroutine exited improperly
    if (coro.isRunning())
      FAIL(fmt::format("Coro {} exited without calling CR_YIELD, CR_EXIT_SUCESS or CR_EXIT_FAILURE", coro.getName()));
  }
};

// ==========================================================================

/**
 * this class implements a base class for coroutine tasks. Tasks maintain
 * their own coroutine context.
 *
 * Having a task maintain its own context (and making it accessible to
 * the member function that implements the coroutine) is cleaner
 * than passing in the context object from outside the coroutine
 * each time.
 */
// class CoroTask : public BasicCoro
// {
//   public:
//   CoroTask(CoroEnv &env, const std::string &name = "") : BasicCoro(env, name) {}
//   CoroTask() = delete;
// };

// ===========================================================================

/*
 * the following macros are used to make the code in an ordinary function
 * resumable. The main restrictions are as follows:
 * - you cannot use a switch statement inside a coroutine body
 * - any coroutine params that you wish to freeze at the first invocation
 *   after reset should be cached somewhere (see local variables)
 * - any "local" variables whose state you wish to retain between invocations
 *   needs to use one of the following 2 approaches:
 *   1. static local variables -- simple but means coroutine is not re-entrant
 *      (i.e. it cannot keep different state for multiple different callers)
 *   2. store the variables in a context struct/class which is passed into
 *      the function on each invocation (as a reference)
 *
 * The main advantage of the following macros is that they can be applied to
 * essentially any function (except for the switch restriction) to add
 * resumable properties. It works very well if you need to implement a
 * long operation which spans multiple superloop/threadloop iterations.
 */

// implementation details (i.e. private/internal macro helpers)

// make code into a valid statment. the parameter *Code* can be a single
// statement or a braced statement block
#define _MAKE_STATEMENT(code)                                                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    code;                                                                                                              \
  } while (0)

#define _CR_BEGIN_RUN(coro)                                                                                            \
  BasicCoro &coro_(coro);                                                                                              \
  BasicCoroScope coroScope_(coro_);                                                                                    \
  _MAKE_STATEMENT({                                                                                                    \
    void *pResume = coro_.internalBeginRun();                                                                          \
    if (pResume)                                                                                                       \
      goto *pResume;                                                                                                   \
    /* else execute from start of next line of code */                                                                 \
  })

// public macros

/** 
 * begin a coroutine with an external coroutine context.
 * This allows coroutine code to be used in any function or class.
 */
#define CR_BEGIN_WITH(coro) _CR_BEGIN_RUN(coro)
#define CR_LOOP_WITH(coro)  CR_BEGIN_WITH(coro); while (true)

/**
 * begin a coroutine within a class derived from BasicCoro
 */
#define CR_BEGIN()          _CR_BEGIN_RUN(*this)
#define CR_LOOP()           CR_BEGIN(); while (true)

// all following macros are only legal after CR_BEGIN/CR_LOOP

/**
 * mark a checkpoint in the behaviour (recorded as a state in the activation graph).
 * It is safe to call this within a loop - if the same checkpoint is set
 * repeatedly the checkpoint start time is not updated.
 * 
 * @param identifier a valid C/C++ identifier to be used as the checkpoint name
 * 
 * Example: CR_CHECKPOINT(startWalking);
 */
#define CR_CHECKPOINT(identifier) coro_.internalSetCheckpoint(#identifier,__LINE__)

// It is not legal to end a coroutine just by exiting the function (at a closing
// brace) or by using a return statement. Instead one of YIELD, EXIT_SUCCESS or
// EXIT_FAILURE *must* be used

// First the variants for coroutines which do *not* have a "return" value
#define CR_YIELD()                                                                                                     \
  _MAKE_STATEMENT({                                                                                                    \
    __label__ resumePoint;                                                                                             \
    coro_.internalSetYielded(&&resumePoint);                                                                           \
    return;                                                                                                            \
  resumePoint:;                                                                                                        \
  })

#define CR_EXIT_SUCCESS()                                                                                              \
  _MAKE_STATEMENT({                                                                                                    \
    coro_.internalSetSuccess();                                                                                        \
    return;                                                                                                            \
  })

#define CR_EXIT_FAILED()                                                                                               \
  _MAKE_STATEMENT({                                                                                                    \
    coro_.internalSetFailed();                                                                                         \
    return;                                                                                                            \
  })

// Here are the variants for coroutines which do have a "return" value

#define CR_YIELD_VAL(val)                                                                                              \
  _MAKE_STATEMENT({                                                                                                    \
    __label__ resumePoint;                                                                                             \
    coro_.internalSetYielded(&&resumePoint);                                                                           \
    return (val);                                                                                                      \
  resumePoint:;                                                                                                        \
  })

#define CR_EXIT_SUCCESS_VAL(val)                                                                                       \
  _MAKE_STATEMENT({                                                                                                    \
    coro_.internalSetSuccess();                                                                                        \
    return (val);                                                                                                      \
  })

#define CR_EXIT_FAILED_VAL(val)                                                                                        \
  _MAKE_STATEMENT({                                                                                                    \
    coro_.internalSetFailed();                                                                                         \
    return (val);                                                                                                      \
  })

// TODO: failed attempt at using one macro whether there is a yield value or not.
// Not sure if it can be made to work, but perhaps adding dummy var at end and
// counting params - 1 might work

// #define CR_YIELD(...) CHOOSE_FUNC(CR_YIELD, __VA_ARGS__)
// #define CR_EXIT_SUCCESS(...) CHOOSE_FUNC(CR_EXIT_SUCCESS, __VA_ARGS__)
// #define CR_EXIT_FAILED(...) CHOOSE_FUNC(CR_EXIT_FAILED, __VA_ARGS__)



// ===========================================================================

/**
 * Note: it is generally not possible to declare variables within Coroutines because
 * of the jumps that take place. 
 * 
 * Workarounds:
 * 
 * 1. Declare a member variable in a subclass of BasicCoro, or at least
 *    in some context which persists beyond the coroutine body code
 * 
 * 2. Declare a variable in a nested code block in the coro body BUT
 *    not inside any loop or if-statement that contains a YIELD or EXIT
 */

/**
 * It is normal to write standard loops in coroutines but
 * the point at which the condition should be checked can be a bit non-intuitive
 * (usually the condition is checked in the loop body)
 * 
 * These are the normal cases...
 *
 * EXAMPLE 1: Coro has just one step, repeated forever
 * 
 *   CR_LOOP()
 *   {
 *     actionCode;
 *     CR_YIELD();
 *   }
 * 
 * EXAMPLE 2: Coro has just one step, but can exit (with success or maybe failure)
 * 
 *   CR_BEGIN();
 * 
 *   // optional: could include code to be executed just after start/reset here
 *
 *   while (true)
 *   {
 *     actionCode;
 *     if (terminatingCondition)
 *       CR_EXIT_SUCCESS();
 *     else
 *       CR_YIELD();
 *   }
 *
 * EXAMPLE 3: coro has multiple steps, repeated forever. Each time one of the
 * inner while loops terminates, the conditions on any other while loops
 * determine which step/phase/checkpoint is executed next
 * 
 *   CR_LOOP()
 *   {
 *     CR_CHECKPOINT(step1);
 *     while (shouldDoAction1)
 *     {
 *       actionCode1;
 *       CR_YIELD();
 *     }
 *
 *     CR_CHECKPOINT(step2);
 *     while (shouldDoAction2)
 *     {
 *       actionCode2;
 *       CR_YIELD();
 *     }
 *   }
 * 
 * 
 * EXAMPLE 4: coro has multiple steps, but can exit.
 * 
 *   CR_BEGIN();
 * 
 *   // optional: could include code to be executed just after start/reset here
 *
 *   CR_CHECKPOINT(step1);
 *   while (shouldDoAction1) // or alternatively: while (!action1Finished)
 *   {
 *     actionCode1;
 *     CR_YIELD();
 *   }
 *
 *   CR_CHECKPOINT(step2);
 *   while (true)
 *   {
 *     actionCode2;
 *     if (terminatingCondition2)
 *       CR_EXIT_SUCCESS();
 *     else
 *       CR_YIELD();
 *   }
 * 
 *
 * Some special conditions...
 *
 * EXAMPLE 5: execute until some milliseconds since the coro was started/reset
 * 
 *   while (getCoroDuration() < SOME_DURATION_MS)
 *     ...
 * 
 * 
 * EXAMPLE 6: execute until some milliseconds since switching to the current 
 * checkpoint was started/reset
 * 
 *   while (getCheckpointDuration() < SOME_DURATION_MS)
 *     ...
 * 
 */


// The following macros shorten some common loop expressions to one-liners
// (They're not used that often)

/**
 * No OPeration (i.e. do nothing) - sometimes useful in CR_AWAIT or CR_WHILE
 */
#define CR_NOP()  ((void)0)

/**
 * await some stopCondition, executing some code on each tick until the condition is true
 * 
 * The condition is checked at the start of each tick (if resuming from the YIELD)
 * and If the condition is true immediately the actionCode will *not* be executed at all
 * 
 * This is usually the right pattern to use, particularly if mapping from
 * an FSM type algorithm
 * 
 * If the next step is SUCCESS or FAILURE it is better not to use this construct
 * (see below)
 * 
 * @param stopCondition  condition on which to stop waiting. It is evaluated before running actionCode
 * @param actionCode can be a single statement (no final semicolon) or block of code in braces
 *                   but the code cannot contain commas in either case
 */
#define CR_AWAIT(stopCondition, actionCode)                                                                            \
  _MAKE_STATEMENT(while (!(stopCondition)) {                                                                           \
    actionCode;                                                                                                        \
    CR_YIELD();                                                                                                        \
  })

/**
 * await some booleanAction that returns true/false to indicate completion
 * and that requires a yield regardless of whether it returned true or false
 */
#define CR_AWAIT_BOOL(booleanAction)                                                                                   \
  _MAKE_STATEMENT({                                                                                                    \
    while (!(booleanAction))                                                                                           \
      CR_YIELD();                                                                                                      \
    CR_YIELD();                                                                                                        \
  })

/**
 * repeat some code on each tick that the specified continueCondition is true
 * (continueCondition is the opposite of the stoppingCondition used in CR_AWAIT)
 * 
 * If the next step is SUCCESS or FAILURE it is better not to use this construct
 * (see below)
 * 
 * @param continueCondition  condition on which to stop waiting. It is evaluated before running actionCode
 * @param actionCode can be a single statement (no final semicolon) or block of code in braces
 *                   but the code cannot contain commas in either case
 */
#define CR_WHILE(continueCondition, actionCode)                                                                        \
  _MAKE_STATEMENT(while (continueCondition) {                                                                          \
    actionCode;                                                                                                        \
    CR_YIELD();                                                                                                        \
  })

/**
 * call some action code once and yield.
 * This is mainly useful inside a larger loop with several if-else-statements
 * when it doesn't make sense to have a single CR_YIELD at the end shared
 * by all the clauses.
 * 
 * Example:
 * 
 * while (true)
 * {
 *   if (cond)
 *     CR_AWAIT(someTask.isSuccess(), someTask);
 *   else
 *     CR_TICK(otherTask);
 * }
 */
#define CR_TICK(actionCode)                                                                                            \
  _MAKE_STATEMENT({                                                                                                    \
    actionCode;                                                                                                        \
    CR_YIELD();                                                                                                        \
  })
