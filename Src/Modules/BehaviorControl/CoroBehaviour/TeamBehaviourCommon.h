#pragma once

#include "CoroBehaviourRegistry.h"

#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"
#include "Tools/BehaviorControl/Framework/Skill/TeamSkill.h"



// -----------------------------------------------------------
// macros
// -----------------------------------------------------------

// NOTE: remember to register any team coro behaviours you will be calling
// like this in TeamCoroBehaviourRegistry::registerBehaviours
#define CALL_TEAM_CORO(name) CoroBehaviour::TeamBehaviourRegistry::theInstance->getBasic(name)()

#define DECLARE_CALLS_TEAM_CORO(varName, name)                                                                         \
  CoroBehaviour::TeamBehaviourRegistry::Basic &varName                                                                 \
  {                                                                                                                    \
    CoroBehaviour::TeamBehaviourRegistry::theInstance->getBasic(name)                                                  \
  }

// cards and skills compatibility

/// call a skill without declaring it first
/// example: CALL_TEAM_SKILL(SomeSkill)(someArg);
#define CALL_TEAM_SKILL(name) (*TeamSkillRegistry::theInstance->getSkill<TeamSkills::name##Skill>(#name))

/// declare a card or skill variable that can be accessed without looking it up
/// on each tick. (Tiny performance gain)
#define DECLARE_CALLS_TEAM_SKILL(name)                                                                                 \
  TeamSkills::name##Skill &the##name##Skill =                                                                          \
      *(TeamSkillRegistry::theInstance->getSkill<TeamSkills::name##Skill>(#name))