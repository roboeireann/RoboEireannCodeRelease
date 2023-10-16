/*
 * @file: CoroBehaviourRegistry.cpp
 *
 * basic implementation for CoroBehaviourRegistry
 *
 * @author: Rudi Villing
 */

#include "CoroBehaviourRegistry.h"



using namespace CoroBehaviour;


thread_local CoroBehaviourRegistry* CoroBehaviourRegistry::theInstance = nullptr;

CoroBehaviourRegistry::CoroBehaviourRegistry(BehaviourEnv& env) : CoroBehaviourRegistryBase(env)
{
  ASSERT(!theInstance);
  theInstance = this;
}

CoroBehaviourRegistry::~CoroBehaviourRegistry()
{
  ASSERT(theInstance);
  theInstance = nullptr;
}


// ============================================================================

thread_local TeamBehaviourRegistry* TeamBehaviourRegistry::theInstance = nullptr;

TeamBehaviourRegistry::TeamBehaviourRegistry(BehaviourEnv& env) : CoroBehaviourRegistryBase(env)
{
  ASSERT(!theInstance);
  theInstance = this;
}

TeamBehaviourRegistry::~TeamBehaviourRegistry()
{
  ASSERT(theInstance);
  theInstance = nullptr;
}

