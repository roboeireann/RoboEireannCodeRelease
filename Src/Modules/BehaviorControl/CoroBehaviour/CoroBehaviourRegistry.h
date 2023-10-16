/*
 * @file: CoroBehaviourRegistry.h
 *
 * Registry for entry point behaviours that are callable without any
 * parameters.
 * 
 * The only reason we need this class to extend CoroBehaviourRegistryBase
 * is because there needs to be different registries for Team and SingleRobot
 * behaviours and both of them require a static singleton pointer
 *
 * @author: Rudi Villing
 */

#pragma once

#include "CoroBehaviourDetail.h"


namespace CoroBehaviour
{
  class CoroBehaviourRegistry : public CoroBehaviourRegistryBase
  {
  public:
    static thread_local CoroBehaviourRegistry* theInstance; ///< The singleton instance in this thread.

    CoroBehaviourRegistry(BehaviourEnv& env);
    ~CoroBehaviourRegistry();
  };


  class TeamBehaviourRegistry : public CoroBehaviourRegistryBase
  {
  public:
    static thread_local TeamBehaviourRegistry* theInstance; ///< The singleton instance in this thread.

    TeamBehaviourRegistry(BehaviourEnv& env);
    ~TeamBehaviourRegistry();
  };

} // CoroBehaviour

