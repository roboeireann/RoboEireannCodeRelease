/**
 * @file Tools/Framework/FrameExecutionUnit.h
 *
 * This file declares a frame execution unit, created from a list of execution unit creators.
 *
 * @author Thomas Röfer
 * @author Jan Fiedler
 */

#pragma once

#include "Tools/Streams/TypeRegistry.h"

#include <list>

struct ModulePacket;
template<typename T>
class Receiver;
class Thread;
struct LogfileLogger;

/**
 * @class FrameExecutionUnit
 *
 * A list of frame execution units that executes thread specific code.
 */
class FrameExecutionUnit
{
public:
  virtual ~FrameExecutionUnit() = default;


  /**
   * called during startup
   * 
   * If this execution unit is associated with the thread that has the GameInfo, 
   * it will control the states of the logger. This function gives the execution
   * unit a chance to store the Logger pointer also.
   * 
   * @param logger pointer to the logger (can be nullptr)
   * @param threadName the name of the thread that will be passed to Logger::execute
   * @return true if this thread has GameInfo and false otherwise
   */
  virtual bool initLogging([[maybe_unused]] LogfileLogger* logger, [[maybe_unused]] const std::string& threadName) { return false; }

  /**
   * The function is executed in every frame.
   * @return Must the modules of this frame be executed? They still can be
   *         run if false is returned here if the framework requires it.
   */
  virtual bool beforeFrame() {return true;}

  /**
   * The function is executed before the modules are executed.
   */
  virtual void beforeModules() {}

  /**
   * The function is executed immediately after the modules were executed.
   */
  virtual void afterModules() {}

  /**
   * The function is executed in every frame.
   * It should contain code that ends a frame. It can also implement waiting for
   * the next frame, in which case it should return false.
   * The default implementation returns false on the robot, but true anywhere else.
   * @return Should the thread wait for a packet to arrive before continuing?
   */
  virtual bool afterFrame();
};

/**
 * The class is a base class for execution unit creators.
 */
class ExecutionUnitCreatorBase
{
private:
  static ExecutionUnitCreatorBase* first; /**< The first entry in the list of all execution unit creators. */
  ExecutionUnitCreatorBase* next; /**< The next entry in the list of all execution unit creators. */

public:
  /**
   * The constructor.
   * Inserts the execution unit creator into the static list.
   */
  ExecutionUnitCreatorBase() : next(first) { first = this; }
  virtual ~ExecutionUnitCreatorBase() = default;

protected:
  /**
   * The function creates a execution unit.
   * @return A pointer to the new execution unit.
   */
  virtual FrameExecutionUnit* create() const = 0;

  /**
   * The function returns the name of the execution unit.
   * @return The name of the execution unit.
   */
  virtual const std::string getName() const = 0;

  friend class ModuleContainer; /**< Only ModuleContainer can create execution units. */
};

/**
 * The template class instantiates creators for execution units of a certain type.
 */
template<typename T> class ExecutionUnitCreator : public ExecutionUnitCreatorBase
{
protected:
  /**
   * The function returns the name of the execution unit.
   * @return The name of the execution unit.
   */
  const std::string getName() const override { return TypeRegistry::demangle(typeid(T).name()); }

  /**
   * The function creates a execution unit.
   * @return A pointer to the new execution unit.
   */
  FrameExecutionUnit* create() const override { return new T(); }
};

/**
 * The macro REGISTER_EXECUTION_UNIT instantiates a execution unit creator.
 * As a convention, it should be used in the first line of the source file.
 * For each execution unit, REGISTER_EXECUTION_UNIT must exactly be used once.
 *
 * @param className The type of the class that will later be instantiated
 *                  as a frame execution unit.
 */
#define REGISTER_EXECUTION_UNIT(className) \
  ExecutionUnitCreator<className> _create##className;
