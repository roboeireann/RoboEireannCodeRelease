/**
 * @file TeamBehaviourModule2025.cpp
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

#include "TeamBehaviourModule2025.h"

// include any entry point behaviours that will be registered and specified in config files
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/NoTeamCoordination.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/MinimalTeamCoordination.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2025Game/TeamCoordination.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2025Leaderboards/MostPassesTeamCoordination.h"

#include "Tools/Debugging/DebugDrawings3D.h"



DECL_TLOGGER(tlogger, "TeamBehaviourModule2025", TextLogging::INFO);

MAKE_MODULE(TeamBehaviourModule2025, behaviorControl, TeamBehaviourModule2025::getExtModuleInfo);

TeamBehaviourModule2025::TeamBehaviourModule2025()
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

std::vector<ModuleBase::Info> TeamBehaviourModule2025::getExtModuleInfo()
{
  auto result = TeamBehaviourModule2025::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<TeamSkill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<TeamCard>::first, result);

  return result;
}

void TeamBehaviourModule2025::update(TeamActivationGraph& teamActivationGraph)
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
  teamActivationGraph.graph.emplace_back("TeamBehaviourModule2025", 0, "", theFrameInfo.time, 0, std::vector<std::string>());

  // execute the top level (team) behaviour
  CALL_TEAM_CORO(topLevelBehaviour);


  // cards and skills compatibility
  theTeamCardRegistry.postProcess();
  theTeamSkillRegistry.postProcess();

  theLibCheck.performTeamCheck();

  draw();
}


void TeamBehaviourModule2025::draw()
{
  DEBUG_DRAWING3D("behaviour:TeamBehaviourStatus:ballPlayer", "robot")
  {
    if (theTeamBehaviorStatus.role.isBallPlayer())
      SPHERE3D("behaviour:TeamBehaviourStatus:ballPlayer", 0, 0, 700, 30, ColorRGBA::red);
  }


  // FIXME: hack until we get a good mechanism to declare drawings for behaviours
  // that only run in some cycles - declare the drawings here
  DECLARE_DEBUG_DRAWING("behaviour:TeamCoordination:formation", "drawingOnField");
}

CoroBehaviour::BehaviourEnv::RepresentationRegistry TeamBehaviourModule2025::initRegisteredRepresentations()
{
  CoroBehaviour::BehaviourEnv::RepresentationRegistry representationRegistry;

#define TEAM_REPRESENTATIONS_FOR_BEHAVIOUR
  #include "TeamBehaviourRepresentations2025.inc"
#undef TEAM_REPRESENTATIONS_FOR_BEHAVIOUR

  return representationRegistry;
}


#define REGISTER_TEAM_CORO(name) theTeamCoroRegistry.registerCoro<name>(#name)

void TeamBehaviourModule2025::registerEntryPointBehaviours()
{
  using namespace CoroBehaviour;

  REGISTER_TEAM_CORO(NoTeamCoordinationTask);
  REGISTER_TEAM_CORO(MinimalTeamCoordinationTask);
  
  REGISTER_TEAM_CORO(RE2025::TeamCoordinationTask);

  REGISTER_TEAM_CORO(RC2025::Leaderboards::MostPassesTeamCoordinationTask);

  TLOGD(tlogger, "Team behaviours registered with cororegistry");
}

