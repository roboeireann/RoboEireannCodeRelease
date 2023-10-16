/**
 * @file CoroBehaviourModule2023.cpp
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

#include "TeamBehaviourModule2023.h"

// include any entry point behaviours that will be registered and specified in config files
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/NoTeamCoordination.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/MinimalTeamCoordination.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/DynamicBallHandlingTeamCoordination.h"



DECL_TLOGGER(tlogger, "TeamBehaviourModule2023", TextLogging::INFO);

MAKE_MODULE(TeamBehaviourModule2023, behaviorControl, TeamBehaviourModule2023::getExtModuleInfo);

TeamBehaviourModule2023::TeamBehaviourModule2023()
    : env(initRegisteredRepresentations(), const_cast<TeamActivationGraph &>(theTeamActivationGraph)),
      theTeamCoroRegistry(env),
      theTeamSkillRegistry("teamSkills.cfg", const_cast<TeamActivationGraph &>(theTeamActivationGraph),
                           theTeamBehaviorStatus),
      theTeamCardRegistry(const_cast<TeamActivationGraph &>(theTeamActivationGraph))
{
  registerEntryPointBehaviours();

  // for cards and skills compatibility
  theTeamSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<TeamCard>::first));
}

std::vector<ModuleBase::Info> TeamBehaviourModule2023::getExtModuleInfo()
{
  auto result = TeamBehaviourModule2023::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<TeamSkill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<TeamCard>::first, result);

  return result;
}

void TeamBehaviourModule2023::update(TeamActivationGraph& teamActivationGraph)
{
  // using namespace CoroBehaviour2022;

  teamActivationGraph.graph.clear();
  teamActivationGraph.currentDepth = 0;

  env.updateTimes(theFrameInfo.time); // update coro behaviour env for this cycle

  // cards and skills compatibility
  theTeamSkillRegistry.modifyAllParameters();
  theTeamCardRegistry.modifyAllParameters();

  theTeamSkillRegistry.preProcess(theFrameInfo.time);
  theTeamCardRegistry.preProcess(theFrameInfo.time);

  // run the behaviour for this tick

  ASSERT(teamActivationGraph.graph.empty());
  teamActivationGraph.graph.emplace_back("TeamBehaviourModule2023", 0, "", theFrameInfo.time, 0, std::vector<std::string>());

  // execute the top level (team) behaviour
  CALL_TEAM_CORO(topLevelBehaviour);


  // cards and skills compatibility
  theTeamCardRegistry.postProcess();
  theTeamSkillRegistry.postProcess();

  theLibCheck.performTeamCheck();
}


CoroBehaviour::BehaviourEnv::RepresentationRegistry TeamBehaviourModule2023::initRegisteredRepresentations()
{
  CoroBehaviour::BehaviourEnv::RepresentationRegistry representationRegistry;

#define TEAM_REPRESENTATIONS_FOR_BEHAVIOUR
  #include "TeamBehaviourRepresentations2023.inc"
#undef TEAM_REPRESENTATIONS_FOR_BEHAVIOUR

  return representationRegistry;
}


#define REGISTER_TEAM_CORO(name) theTeamCoroRegistry.registerCoro<name>(#name)

void TeamBehaviourModule2023::registerEntryPointBehaviours()
{
  using namespace CoroBehaviour;

  REGISTER_TEAM_CORO(NoTeamCoordinationTask);
  REGISTER_TEAM_CORO(MinimalTeamCoordinationTask);
  REGISTER_TEAM_CORO(RC2023::DynamicBallHandlingTeamCoordinationTask);

  TLOGD(tlogger, "Team behaviours registered with cororegistry");
}

