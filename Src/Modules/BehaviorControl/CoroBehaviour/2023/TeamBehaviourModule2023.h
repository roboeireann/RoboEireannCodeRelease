/**
 * @file TeamBehaviourModule2022.h
 *
 * This is the module that runs the team role selection and coordination
 * behaviour.
 * Because of some library dependencies, this has to run as a separate module
 * before the single robot behaviour.
 *
 * From this file, the team behaviour uses the coro behaviour architecture
 * as the top level behaviour. (Cards and skills can be used if really needed
 * as sub-behaviours, but using coro/tasks is much preferred).
 *
 * The implementation is based in part on the BH2021 code release but
 * modified for the coro behaviour architecture
 *
 * @author Rudi Villing
 */

#pragma once


#include "Tools/Module/Module.h"
#include <string>


#define TEAM_REPRESENTATION_INCLUDES
#include "TeamBehaviourRepresentations2023.inc"
#undef TEAM_REPRESENTATION_INCLUDES

#define TEAM_REPRESENTATIONS_MODULE
#include "TeamBehaviourRepresentations2023.inc"
#undef TEAM_REPRESENTATIONS_MODULE

// Include here so macros do not dismantle themselves
#include "TeamBehaviour2023.h"



class TeamBehaviourModule2023 : public TeamBehaviourModule2023Base
{
public:
  /** Constructor. */
  TeamBehaviourModule2023();

  /**
   * Creates extended module info (union of this module's info and requirements of all behavior parts (cards and skills)).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();

private:
  // local copy of provided representations
  TeamBehaviorStatus theTeamBehaviorStatus; /**< The team behavior status that is modified by the behavior. */

  // coro behaviour support
  CoroBehaviour::BehaviourEnv env;
  CoroBehaviour::TeamBehaviourRegistry theTeamCoroRegistry; // there can only be one instance in this thread

  // allow some compatibility with cards and skills based behaviours
  TeamSkillRegistry theTeamSkillRegistry; /**< The manager of all skills. */
  TeamCardRegistry theTeamCardRegistry; /**< The manager of all cards. */

  /**
   * helper function to ensure all necessary representations used by team behaviours
   * are registered with BehaviourEnv
   * prior to any skills or behaviours constructing and trying to access the
   * representations
   */
  CoroBehaviour::BehaviourEnv::RepresentationRegistry initRegisteredRepresentations();

  /**
   * helper to register all entry point behaviours that may be specified using config files
   */
  void registerEntryPointBehaviours();

  /**
   * Updates the team activation graph and executes the main behaviour.
   * (Other update functions merely copy values)
   * @param teamActivationGraph The provided team activation graph.
   */
  void update(TeamActivationGraph& teamActivationGraph) override;

  // simply copy the values that were filled in when update(ActivationGraph...) was executed
  void update(TeamBehaviorStatus& teamBehaviorStatus) override { teamBehaviorStatus = theTeamBehaviorStatus; }
};
