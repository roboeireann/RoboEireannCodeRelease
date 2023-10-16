/*
 * @file: CoroBehaviour.h
 *
 * General include file for all Coroutine based behaviour tasks
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/BehaviourEnv.h"
#include "Modules/BehaviorControl/CoroBehaviour/Coroutine.h"

#include "Representations/BehaviorControl/ActivationGraph.h"

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Debugging/Annotation.h"

#include "Tools/TextLogging.h"

#include <memory>

namespace CoroBehaviour
{

  /**
   * This is the base class for all coroutine behaviour tasks.
   *
   * The main difference from BasicCoro is that we also provide the behaviour
   * enviroment (mainly access to the required/provided representations)
   */
  class CoroBehaviour : public BasicCoro
  {
  public:
    CoroBehaviour(BehaviourEnv &env, const std::string &name)
        : BasicCoro(env, name), env(env)
    {
      loadParameters();
    }

    virtual void reset() override
    {
      prevAnnotation.clear();
      BasicCoro::reset();
    }

  protected:
    // This is a bit hacky but is needed so that we can enforce that parameters
    // will be loaded from config files before the behaviour tries to use the 
    // parameter values. (One particular case of this is when config file parameters
    // hold the names of subordinate behaviours to be called)
    friend struct ParamLoader;
    struct ParamLoader
    {
      ParamLoader(CoroBehaviour *ptr)
      {
        // ptr->registerPossibleSubtasks();
        ptr->loadParameters();
      }
    };

  protected:
    BehaviourEnv &env;

    // subclasses can override (via macros) but don't necessarily need to, so do-nothing implementation included here
    virtual void modifyParameters() {}
    virtual void loadParameters() {}
    // virtual void registerPossibleSubtasks() {}

    void annotation(const std::string &text)
    {
      if (text != prevAnnotation)
      {
        ANNOTATION("Behaviour", text);
        prevAnnotation = text;
      }
    }

    void addActivationGraphOutput(const std::string& parameters)
    {
      theActivationGraph.graph[activationGraphIndex].parameters.push_back(parameters);
    }

  private:
    std::string prevAnnotation;
    size_t activationGraphIndex;

    ActivationGraph& theActivationGraph { env.theActivationGraph };

    friend class CoroBehaviourScope;

    void beginScope()
    {
      activationGraphIndex = theActivationGraph.graph.size();
      ++theActivationGraph.currentDepth; // increase depth counter for activation graph
      theActivationGraph.graph.emplace_back(getName(), theActivationGraph.currentDepth, "", getCoroDuration(), 0,
                                            std::vector<std::string>());

      modifyParameters();
    }

    void endScope()
    {
      theActivationGraph.graph[activationGraphIndex].state = getCheckpointName();
      theActivationGraph.graph[activationGraphIndex].stateTime = getCheckpointDuration();

      --theActivationGraph.currentDepth; // decrease depth counter for activation graph
    }
  };

  /**
   * Internal implementation class used by macros to ensure that the
   * activation graph is updated as we enter and leave behaviours
   */
  class CoroBehaviourScope
  {
  public:
    CoroBehaviourScope(CoroBehaviour &coro) : coro(coro) { coro.beginScope(); }
    CoroBehaviourScope() = delete;

    ~CoroBehaviourScope() { coro.endScope(); }

  private:
    CoroBehaviour &coro;
  };

  // this class can be useful while developing to use as a placeholder
  // for a subtask to be added later

  // class PlaceholderBehaviour : public CoroBehaviour
  // {
  // public:
  //   PlaceholderBehaviour(BehaviourEnv &env, const std::string &name) : CoroBehaviour(env, name) {}

  //   void operator()(void) {} // do nothing at all
  // };


  /**
   * Allow behaviours to be registered by name. Normally only a small
   * set of behaviours (typically the possible root/start behaviours)
   * will be registered by name
   * 
   * We need to specialize this class twice - once for normal behaviours
   * and once for team behaviours since these behaviours run in different
   * modules with different BehaviourEnv objects and we need singleton
   * access to them
   */
  class CoroBehaviourRegistryBase
  {
  public:
    CoroBehaviourRegistryBase(BehaviourEnv& env) : env(env) {}
    ~CoroBehaviourRegistryBase() {}

    // Basic is wrapper base class that provides access to a previously
    // registered basic behaviours (that are callable with no parameters).
    //
    // It provides a minimal interface that is compatible with some of the 
    // behaviour macros.
    //
    // CoroBehaviour and its base class BasicCoro don't have any default
    // execution method but all behaviours provide some version of operator()
    // and top level behaviours provide a version with no parameters so that
    // is what the Basic class provides.
    struct Basic
    {
      virtual ~Basic() = default;

      virtual void operator()(void) = 0;

      // delegate to the CoroBehaviour interface
      virtual bool isYielded() const = 0;
      virtual bool isSuccess() const = 0;
      virtual bool isFailure() const = 0;
      virtual bool isEnded() const = 0;
      virtual bool isReset() const = 0;

      virtual void reset() = 0;

    protected:
      virtual Basic* createIfNeeded(BehaviourEnv &env) = 0;

      friend class CoroBehaviourRegistryBase;
    };

    /**
     * get the basic wrapper for a previously registered behaviour
     * (you don't need to know the exact type for this)
     */
    Basic& getBasic(const std::string& name)
    {
#ifndef NDEBUG
      if (basicBehaviours.find(name) == basicBehaviours.end())
        FAIL("Could not find basic behaviour with name: " << name);
#endif
      
      return *(basicBehaviours[name]->createIfNeeded(env));
    }

    /**
     * get the previously registered behaviour
     * (you need to know the exact type for this)
     */
    template <class T>
    T& getCoro(const std::string& name)
    {
#ifndef NDEBUG
      if (basicBehaviours.find(name) == basicBehaviours.end())
        FAIL("Could not find basic behaviour with name: " << name);
#endif
      
      return *(dynamic_cast< Wrapper<T>* >(basicBehaviours[name])->createDelegateIfNeeded(env));
    }

    /**
     * register a basic behaviour, i.e. one that is callable with no parameters
     */
    template <class T>
    void registerCoro(const std::string& name)
    {
      basicBehaviours[name] = std::unique_ptr<Basic>(new Wrapper<T>());
    }


  private:
    template <class T> class Wrapper : public Basic
    {
    public:
      Wrapper() = default;

      void operator()(void) override { delegate->operator()(); }

      // delegate to the CoroBehaviour interface
      bool isYielded() const override { return delegate->isYielded(); }
      bool isSuccess() const override { return delegate->isSuccess(); }
      bool isFailure() const override { return delegate->isFailure(); }
      bool isEnded() const override { return delegate->isEnded(); }
      bool isReset() const override { return delegate->isReset(); }

      void reset() override { delegate->reset(); }

    private:
      std::unique_ptr<T> delegate;

      Basic* createIfNeeded(BehaviourEnv &env) override
      {
        if (!delegate)
          delegate.reset(new T(env));

        return this;
      }

      T* createDelegatesIfNeeded(BehaviourEnv &env)
      {
        if (!delegate)
          delegate.reset(new T(env));

        return delegate;
      }
    };

    std::unordered_map<std::string, std::unique_ptr<Basic> > basicBehaviours;
    BehaviourEnv &env;
  };

  // we need different versions for normal behaviours and team behaviours
  // because each of them has a different behaviour environment
  // (At least differing in the representations which are modifiable)

  
} // CoroBehaviour

// ==========================================================================

#define CRBEHAVIOUR(cls_)  struct cls_ : public CoroBehaviour::CoroBehaviour
#define CRBEHAVIOUR_INIT(cls_) cls_(BehaviourEnv& env) : CoroBehaviour::CoroBehaviour(env, #cls_)


/**
 * begin a new behaviour coroutine body where you have control over the
 * behaviour flow, use this macro. 
 * 
 * If you will write a simple behaviour with one infinite loop, use
 * CRBEHAVIOUR_LOOP instead.
 * On the other hand if your behaviour might consist of several loops,
 * one after the next, then you should use CRBEHAVIOUR_BEGIN instead.
 * For the latter, the general code template is as follows:
 * 
 *    CRBEHAVIOUR(SomeTask)
 *    {
 *      CRBEHAVIOUR_INIT(SomeTask) {}
 * 
 *      void operator() (void)
 *      {
 *        CRBEHAVIOUR_BEGIN();
 * 
 *        // TODO: body of coroutine goes here
 *      }
 * 
 *    private:
 *      // declare variables and subtasks here
 *    };
 */
#define CRBEHAVIOUR_BEGIN()                                                                                            \
  CoroBehaviourScope coroBehaviourScope_(*this);                                                                       \
  CR_BEGIN()

/** begin a new coroutine body that implements an infinite loop
 * (This is probably the most common usage).
 *
 * E.g.
 *
 *    CRBEHAVIOUR(SomeTask)
 *    {
 *      CRBEHAVIOUR_INIT(SomeTask) {}
 * 
 *      void operator()(void) 
 *      {
 *        CRBEHAVIOUR_LOOP() 
 *        {
 *          ...body of coroutine to repeat...
 *        }
 *      }
 *    };
 */
#define CRBEHAVIOUR_LOOP()                                                                                             \
  CRBEHAVIOUR_BEGIN();                                                                                                 \
  while (true)


/**
 * Define some parameters at compile time
 * 
 * These can be modified via SimRobot at runtime, though it depends on the
 * behaviour whether it would handle runtime changes gracefully
 */ 
#define DEFINES_PARAMS(theName_,header_,...) \
    _STREAM_STREAMABLE(Params, Streamable, , , (header_), __VA_ARGS__); \
    Params params;\
    void modifyParameters() override \
    { \
        Global::getDebugDataTable().updateObject("parameters:" #theName_, params, false); \
        DEBUG_RESPONSE_ONCE("debug data:parameters:" #theName_) \
          OUTPUT(idDebugDataResponse, bin, "parameters:" #theName_ << TypeRegistry::demangle(typeid(Params).name()) << params); \
    } \
    using defines_params_semicolon_helper = Params

/**
 * Load parameters from a file into a coro behaviour
 * 
 * The file will be called <theName>.cfg and will be located in the
 * `CoroBehaviour` subdirectory of the scenario.
 */ 
#define LOADS_PARAMS(theName_,header_,...) \
    DEFINES_PARAMS(theName_,header_,__VA_ARGS__); \
    void loadParameters() override \
    { \
      /* fmt::print(stderr, #theName "::loadParameters\n"); */ \
      loadModuleParameters(params, #theName_, nullptr, "CoroBehaviour/"); \
    } \
    ParamLoader loader {this}; \
    using loads_params_semicolon_helper = Params


#define ANNOTATION_FMT(...) annotation(fmt::format(__VA_ARGS__))



