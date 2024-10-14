/**
 * @file CoroBehaviorModule2024.cpp
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

#include "CoroBehaviourModule2024.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

// entry point behaviours that we can specify via a config file
#include "Modules/BehaviorControl/CoroBehaviour/Control/SoccerGameControl.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/BasicSetState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/BasicReturnFromPenalized.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/Standard/BasicTasks.h"

#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2024Game/StandbyState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2024Game/ReadyState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2024Game/PlayingState.h"
#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2024Game/PlayingStatePositioningTest.h"

#include "Modules/BehaviorControl/CoroBehaviour/Control/RE2022KickLengthCalibration/KickLengthCalibrationAssistant.h"


#include "Modules/BehaviorControl/CoroBehaviour/Control/RL2024/DefenderNeuralControl.h"


#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Annotation.h"
#include "Tools/Settings.h"

#include <regex>
#include <iostream>


DECL_TLOGGER(tlogger, "CoroBehaviourModule2024", TextLogging::INFO);

MAKE_MODULE(CoroBehaviourModule2024, behaviorControl, CoroBehaviourModule2024::getExtModuleInfo);

CoroBehaviourModule2024::CoroBehaviourModule2024()
    : env(initRegisteredRepresentations(), const_cast<ActivationGraph &>(theActivationGraph)), theCoroBehaviourRegistry(env),
      theSkillRegistry("skills.cfg", const_cast<ActivationGraph &>(theActivationGraph), theArmMotionRequest,
                       theBehaviorStatus, theCalibrationRequest, theHeadMotionRequest, theMotionRequest, theTeamTalk),
      theCardRegistry(const_cast<ActivationGraph &>(theActivationGraph))
{
  registerEntryPointBehaviours();

  // for cards and skills compatibility
  theSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<Card>::first));
}

std::vector<ModuleBase::Info> CoroBehaviourModule2024::getExtModuleInfo()
{
  auto result = CoroBehaviourModule2024::getModuleInfo();

  SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  CardCreatorBase::addToModuleInfo(CardCreatorList<Card>::first, result);

  return result;
}


void CoroBehaviourModule2024::update(ActivationGraph &activationGraph)
{
  // FIXME: hack declare drawing used in behaviour here
  DECLARE_DEBUG_DRAWING("behaviour:ObstacleSkills:ballKickSectorWheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behaviour:GotoBallSkills:passTargetOnField", "drawingOnField");

  // declare debug drawings from behaviours
  for (const CoroBehaviour::BehaviourEnv::DebugDrawingInfo& info : env.getDrawingDeclarations())
  {
    Global::getDrawingManager().addDrawingId(info.drawingId, info.drawingType);
    DECLARE_DEBUG_RESPONSE(info.debugResponseId);
  }


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
  activationGraph.graph.emplace_back("CoroBehaviourModule2024", 0, TypeRegistry::getEnumName(status), theFrameInfo.time,
                                     theFrameInfo.time - statusStartTime, std::vector<std::string>());

#ifdef TARGET_SIM // TODO - not in 2021 BehaviorControl.cpp
  MODIFY("module:CoroBehaviourModule2024:status", status);
#endif

  executeBehaviourTick();

  activationGraph.graph[0].state = TypeRegistry::getEnumName(status);

  // cards and skills compatibility
  theCardRegistry.postProcess();
  theSkillRegistry.postProcess();


  theLibCheck.performCheck(theMotionRequest);

  draw();
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



STREAMABLE(SayText,
{,
  (std::string) text,
  (float)(1.f) speed,
  (float)(1.f) pitchFactor,
  (float)(1.f) pitchSdFactor,
});

void CoroBehaviourModule2024::executeBehaviourTick()
{
  using namespace CoroBehaviour;

  SayText sayText;

  MODIFY_ONCE("module:CoroBehaviourModule2024:say", sayText);
  if (!sayText.text.empty())
    SystemCall::say(sayText.text, sayText.speed, sayText.pitchFactor, sayText.pitchSdFactor);



  MODIFY("module:CoroBehaviourModule2024:status", status);

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
  if ((theGameInfo.playerMode == GameInfo::unstiff) && (status == gettingUp || status == penalized || status == playing))
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
    TLOGD(tlogger, "{} [{}]: sittingDown (actually {})", theGameInfo.playerNumber, theFrameInfo.time, status);
    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.lookForward();

    if (motionSkills.keyframeMotion(KeyframeMotionRequest::sitDown))
      NEXT_STATE(inactive);
  }

  FSM_STATE(inactive)
  {
    TLOGD(tlogger, "{} [{}]: inactive", theGameInfo.playerNumber, theFrameInfo.time);

    if (theGameInfo.playerMode != GameInfo::unstiff)
      GOTO_STATE(gettingUp);

    commonSkills.activityStatus(BehaviorStatus::unknown);
    headSkills.lookAtAngles(JointAngles::off, JointAngles::off);
    motionSkills.unstiff();
  }

  FSM_STATE(gettingUp)
  {
    TLOGD(tlogger, "{} [{}]: gettingUp", theGameInfo.playerNumber, theFrameInfo.time);

    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.lookForward();

    if (commonSkills.stand()) // stand normal, true if done
      NEXT_STATE(playing);
  }

  FSM_STATE(playing)
  {
    TLOGD(tlogger, "{} [{}]: playing", theGameInfo.playerNumber, theFrameInfo.time);

    // update the status if the robot has just been penalized
    if (theGameInfo.isPenalized())
    {
      ANNOTATION("Behavior", "Penalized " + theGameInfo.getPenaltyString());
      SystemCall::say("Penalized");
      GOTO_STATE(penalized);
    }

    // execute the top level normal play behaviour
    CALL_BASIC_CORO(topLevelBehaviour);
  }
  
  FSM_STATE(penalized)
  {
    TLOGD(tlogger, "{} [{}]: penalized", theGameInfo.playerNumber, theFrameInfo.time);

    // update the status if the robot has just been unpenalized?
    if (!theGameInfo.isPenalized())
    {
      ANNOTATION("Behavior", "Unpenalized");
      SystemCall::say("Not penalized");

      GOTO_STATE(playing);
    }

    commonSkills.activityStatus(BehaviorStatus::unknown);
    commonSkills.standLookDown(true); // stand high
  }

}


void CoroBehaviourModule2024::checkUsbMountedButtonRequest()
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



void CoroBehaviourModule2024::checkJointTemperatureButtonRequest()
{
  if ((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0) ||
      (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
  {
    SystemCall::say(fmt::format("{} hottest joint is {}. It is {} degrees.", 
                                theGameInfo.getPlayerNumberAndColorString(), 
                                Joints::jointLongNames[theRobotHealth.jointWithMaxTemperature],
                                theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]));
  }
}


CoroBehaviour::BehaviourEnv::RepresentationRegistry CoroBehaviourModule2024::initRegisteredRepresentations()
{
  CoroBehaviour::BehaviourEnv::RepresentationRegistry representationRegistry;

#define CORO_REPRESENTATIONS_FOR_BEHAVIOUR
#include "CoroBehaviourRepresentations2024.inc"
#undef CORO_REPRESENTATIONS_FOR_BEHAVIOUR

  return representationRegistry;
}



#define REGISTER_CORO(name) theCoroBehaviourRegistry.registerCoro<name>(#name)

void CoroBehaviourModule2024::registerEntryPointBehaviours()
{
  // Register any basic (single robot) ehaviours to be looked up from this registry here

  using namespace CoroBehaviour;

  // standard tasks (used by most)
  REGISTER_CORO(SoccerGameControlTask);
  REGISTER_CORO(BasicSetStateTask);
  REGISTER_CORO(BasicReturnFromPenalizedTask);
  REGISTER_CORO(StandLookActiveTask);
  REGISTER_CORO(StandLookForwardTask);
  REGISTER_CORO(PlaceholderTask);
  REGISTER_CORO(PlaceholderSuccessTask);

  // RE2024 specific tasks
  REGISTER_CORO(RE2024::StandbyStateTask);
  REGISTER_CORO(RE2024::ReadyStateTask);
  REGISTER_CORO(RE2024::PlayingStateTask);
  REGISTER_CORO(RE2024::PlayingStatePositioningTestTask);

  // kick length calibration
  REGISTER_CORO(RE2022::KickLengthCalibrationAssistantTask);

  // Reinforcement Learning tasks
  REGISTER_CORO(RL2024::DefenderNeuralControlTask);

  TLOGD(tlogger, "behaviours registered with cororegistry");
}

#undef REGISTER_CORO



void CoroBehaviourModule2024::draw()
{
  DEBUG_DRAWING3D("module:CoroBehaviour2024:refereeStandby", "field")
  {
    // draw the stick figure referee
    float yBehindSideline = theFieldDimensions.yPosLeftSideline + 300;

    // legs
    LINE3D("module:CoroBehaviour2024:refereeStandby", -150, yBehindSideline, 0, -150, yBehindSideline, 750, 40, ColorRGBA::black);
    LINE3D("module:CoroBehaviour2024:refereeStandby", 150, yBehindSideline, 0, 150, yBehindSideline, 750, 40, ColorRGBA::black);

    LINE3D("module:CoroBehaviour2024:refereeStandby", -150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);
    LINE3D("module:CoroBehaviour2024:refereeStandby", 150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);

    // torso + head
    LINE3D("module:CoroBehaviour2024:refereeStandby", 0, yBehindSideline, 800, 0, yBehindSideline, 1700, 40, ColorRGBA::black);

    // shoulders
    LINE3D("module:CoroBehaviour2024:refereeStandby", -220, yBehindSideline, 1350, 220, yBehindSideline, 1350, 40, ColorRGBA::black);

    // approx arms (based on current gesture)
    // gestures default to left arm (+ve x side, red team side)

    // Upper arm 300mm, forearm 300mm, hand 150mm, so we treat total arm incl 
    // hand as 750mm and forearm incl hand as 450mm

    Vector3f shoulder(220, yBehindSideline, 1350);
    // Vector3f handDown(220, yBehindSideline, 1350 - 750);                   // straight down
    Vector3f handUp(220, yBehindSideline, 1350 + 750);                   // straight down
    // Vector3f handSide(220 + 750, yBehindSideline, 1350);                   // straight out to side
    // Vector3f handSideUp(220 + 530, yBehindSideline, 1350 + 530);           // 45 deg up
    // Vector3f handSideDown(220 + 530, yBehindSideline, 1350 - 530);         // 45 deg down
    // Vector3f handFrontPoint(220 - 100, yBehindSideline - 600, 1350 - 375); // point to center (very approx)

    // move requiring elbows...
    // Vector3f elbowDown(220, yBehindSideline, 1350 - 300);       // straight down
    Vector3f elbowUp(220, yBehindSideline, 1350 + 300);       // straight down
//     Vector3f forearmUp(150, yBehindSideline - 150, 1350 + 150); // up overlapping body
// 
//     Vector3f elbowSideDown(220 + 210, yBehindSideline, 1350 - 210); // down and to side
//     float forearmLen = 450.f;
//     Vector3f forearmForwardOrigin(0, -450, 0); // point forward from elbow


    float side = 1.f;

    // standby gesture
    {
      LINE3D("module:CoroBehaviour2024:refereeStandby", shoulder.x() * side, shoulder.y(), shoulder.z(),
              elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
      LINE3D("module:CoroBehaviour2024:refereeStandby", elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
              handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);

      LINE3D("module:CoroBehaviour2024:refereeStandby", -shoulder.x() * side, shoulder.y(), shoulder.z(),
              -elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
      LINE3D("module:CoroBehaviour2024:refereeStandby", -elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
              -handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);
    }
  }
}