/**
 * @file CoroBehaviorModule2023.cpp
 *
 * This is the module that runs the behaviour for a single robot.
 * Because of its dependencies, it expects a team behaviour module
 * to run first.
 * 
 * From this file, the behaviour uses the coro behaviour architecture.
 * 
 * The implementation is based in part on the BH2021 code release but
 * modified for the coro behaviour architecture. Other parts of the 
 * implementation have been modified and tidied a bit also.
 *
 * @author Rudi Villing
 */

#include "Tools/TextLogging.h"

#include "CoroBehaviourModule2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

// entry point behaviours that we can specify via a config file
#include "Modules/BehaviorControl/CoroBehaviour/Control/SoccerGameControl.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/BasicSetState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/BasicReturnFromPenalized.h"

#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2023Game/ReadyState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2023Game/PlayingState.h"

#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/ReadyOrSetState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/AttackerPlayingState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RC2023DynamicBallHandling/DefenderPlayingState.h"

#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2022KickLengthCalibration/KickLengthCalibrationAssistant.h"


#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/Settings.h"

#include <regex>
#include <iostream>


DECL_TLOGGER(tlogger, "CoroBehaviourModule2023", TextLogging::INFO);

MAKE_MODULE(CoroBehaviourModule2023, behaviorControl, CoroBehaviourModule2023::getExtModuleInfo);

CoroBehaviourModule2023::CoroBehaviourModule2023()
    : env(initRegisteredRepresentations(), const_cast<ActivationGraph &>(theActivationGraph)), theCoroBehaviourRegistry(env),
      theSkillRegistry("skills.cfg", const_cast<ActivationGraph &>(theActivationGraph), theArmMotionRequest,
                       theBehaviorStatus, theCalibrationRequest, theHeadMotionRequest, theMotionRequest, theTeamTalk),
      theCardRegistry(const_cast<ActivationGraph &>(theActivationGraph))
{
  registerEntryPointBehaviours();

  // for cards and skills compatibility
  theSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<Card>::first));
}

std::vector<ModuleBase::Info> CoroBehaviourModule2023::getExtModuleInfo()
{
  auto result = CoroBehaviourModule2023::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<Card>::first, result);

  return result;
}


void CoroBehaviourModule2023::update(ActivationGraph &activationGraph)
{
  // FIXME: hack declare drawing used in behaviour here
  DECLARE_DEBUG_DRAWING("behaviour:ObstacleSkills:ballKickSectorWheel", "drawingOnField");

  activationGraph.graph.clear();
  activationGraph.currentDepth = 0;

  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo = Vector2f::Zero();

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;

  env.updateTimes(theFrameInfo.time); // update for this cycle

  // reset the time spent with this status if needed
  if (status != prevStatus)
  {
    prevStatus = status;
    statusStartTime = theFrameInfo.time;
  }

  // cards and skills compatibility
  theSkillRegistry.modifyAllParameters();
  theCardRegistry.modifyAllParameters();

  theSkillRegistry.preProcess(theFrameInfo.time);
  theCardRegistry.preProcess(theFrameInfo.time);

  // run the behaviour for this tick

  ASSERT(activationGraph.graph.empty());
  // add: option_name, depth, state_name, optionTime, stateTime, stringVecOfParams
  activationGraph.graph.emplace_back("CoroBehaviourModule2023", 0, TypeRegistry::getEnumName(status), theFrameInfo.time,
                                     theFrameInfo.time - statusStartTime, std::vector<std::string>());

#ifdef TARGET_SIM // TODO - not in 2021 BehaviorControl.cpp
  MODIFY("module:CoroBehaviourModule2023:status", status);
#endif

  executeBehaviourTick();

  activationGraph.graph[0].state = TypeRegistry::getEnumName(status);

  // cards and skills compatibility
  theCardRegistry.postProcess();
  theSkillRegistry.postProcess();


  theLibCheck.performCheck(theMotionRequest);
}


// some very simple FSM macros inspired by CABSL that help to tidy up the following
// function a little bit
#define FSM_STATE(s)   \
  if (false) { goto fsm_state_##s; /* <<< to avoid unused label warning */ fsm_state_##s: status = s; } \
  if (status == s)

// switch to s immediately in the current cycle
#define GOTO_STATE(s)  goto fsm_state_##s

// run s in the next cycle
#define NEXT_STATE(s)  status = s; return




void CoroBehaviourModule2023::executeBehaviourTick()
{
  using namespace CoroBehaviour;

  MODIFY("module:CoroBehaviourModule2023:status", status);

  if (firstFrame)
  {
    firstFrame = false;
    if (SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::playSound("wearoff.wav"); // traditional RoboEireann startup sound
  }

  // before entering the state machine part of the function respond to button based information requests
  checkUsbMountedButtonRequest();
  checkJointTemperatureButtonRequest();

  // unstiff mode
  if ((theRobotInfo.mode == RobotInfo::unstiff) && (status == gettingUp || status == penalized || status == playing))
    status = sittingDown;

  // camera status
  if (!theCameraStatus.ok && (status == penalized || status == playing))
    status = cameraStatusFailed;

#ifndef NDEBUG
  // low battery
  if ((theRobotHealth.batteryLevel <= 1) && (status == penalized || status == playing))
    status = lowBattery;
#endif

  // basic state machine based on status

  if (status == sittingDown || status == lowBattery || status == cameraStatusFailed)
  {
    TLOGD(tlogger, "{} [{}]: sittingDown (actually {})", theRobotInfo.number, theFrameInfo.time, status);
    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.lookForward();

    if (motionSkills.keyframeMotion(KeyframeMotionRequest::sitDown))
      NEXT_STATE(inactive);
  }

  FSM_STATE(inactive)
  {
    TLOGD(tlogger, "{} [{}]: inactive", theRobotInfo.number, theFrameInfo.time);

    if (theRobotInfo.mode != RobotInfo::unstiff)
      GOTO_STATE(gettingUp);

    commonSkills.activityStatus(BehaviorStatus::unknown);
    headSkills.lookAtAngles(JointAngles::off, JointAngles::off);
    motionSkills.unstiff();
  }

  FSM_STATE(gettingUp)
  {
    TLOGD(tlogger, "{} [{}]: gettingUp", theRobotInfo.number, theFrameInfo.time);

    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.lookForward();

    if (commonSkills.stand()) // stand normal, true if done
      NEXT_STATE(playing);
  }

  FSM_STATE(playing)
  {
    TLOGD(tlogger, "{} [{}]: playing", theRobotInfo.number, theFrameInfo.time);

    // update the status if the robot has just been penalized
    if (theRobotInfo.penalty != PENALTY_NONE)
    {
      ANNOTATION("Behavior", "Penalized " + theRobotInfo.getPenaltyString());
      SystemCall::say("Penalized");
      GOTO_STATE(penalized);
    }

    // execute the top level normal play behaviour
    CALL_BASIC_CORO(topLevelBehaviour);
  }
  
  FSM_STATE(penalized)
  {
    TLOGD(tlogger, "{} [{}]: penalized", theRobotInfo.number, theFrameInfo.time);

    // update the status if the robot has just been unpenalized?
    if (theRobotInfo.penalty == PENALTY_NONE)
    {
      ANNOTATION("Behavior", "Unpenalized");
      SystemCall::say("Not penalized");

      GOTO_STATE(playing);
    }

    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.standLookDown(true); // stand high
  }

}


void CoroBehaviourModule2023::checkUsbMountedButtonRequest()
{
  // speak the USB mounted state if the top of the head is triple-tapped
  if (theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 3)
  {
    if (SystemCall::usbIsMounted())
      SystemCall::say("USB mounted");
    else
      SystemCall::say("USB not mounted");
  }
}



void CoroBehaviourModule2023::checkJointTemperatureButtonRequest()
{
  if ((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0) ||
      (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
  {
    SystemCall::say(fmt::format("{} hottest joint is {}. It is {} degrees.", 
                                theRobotInfo.getRobotAndColourString(), 
                                Joints::jointLongNames[theRobotHealth.jointWithMaxTemperature],
                                theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]));
  }
}


CoroBehaviour::BehaviourEnv::RepresentationRegistry CoroBehaviourModule2023::initRegisteredRepresentations()
{
  CoroBehaviour::BehaviourEnv::RepresentationRegistry representationRegistry;

#define CORO_REPRESENTATIONS_FOR_BEHAVIOUR
#include "CoroBehaviourRepresentations2023.inc"
#undef CORO_REPRESENTATIONS_FOR_BEHAVIOUR

  return representationRegistry;
}



#define REGISTER_CORO(name) theCoroBehaviourRegistry.registerCoro<name>(#name)

void CoroBehaviourModule2023::registerEntryPointBehaviours()
{
  // Register any basic (single robot) ehaviours to be looked up from this registry here

  using namespace CoroBehaviour;

  // standard tasks (used by most)
  REGISTER_CORO(SoccerGameControlTask);
  REGISTER_CORO(BasicSetStateTask);
  REGISTER_CORO(BasicReturnFromPenalizedTask);
  REGISTER_CORO(StandLookForwardTask);
  REGISTER_CORO(PlaceholderTask);
  REGISTER_CORO(PlaceholderSuccessTask);

  // RE2023 specific tasks
  REGISTER_CORO(RE2023::ReadyStateTask);
  REGISTER_CORO(RE2023::PlayingStateTask);

  // RC2023: dynamic ball handling challenge
  REGISTER_CORO(RC2023::ReadyOrSetStateTask);
  REGISTER_CORO(RC2023::AttackerPlayingStateTask);
  REGISTER_CORO(RC2023::DefenderPlayingStateTask);

  // kick length calibration
  REGISTER_CORO(RE2022::KickLengthCalibrationAssistantTask);

  TLOGD(tlogger, "behaviours registered with cororegistry");
}
