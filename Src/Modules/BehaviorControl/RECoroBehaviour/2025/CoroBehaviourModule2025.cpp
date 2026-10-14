/**
 * @file CoroBehaviorModule2025.cpp
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

#include "TextLogging/TextLogging.h"

#include "CoroBehaviourModule2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"

// entry point behaviours that we can specify via a config file
// utility behaviours
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2022KickLengthCalibration/KickLengthCalibrationAssistant.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2024KickTests/KickTestsAssistant.h"

// main game related behaviours
#include "Modules/BehaviorControl/RECoroBehaviour/Control/SoccerGameControl.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/Standard/BasicSetState.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/Standard/BasicReturnFromPenalized.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/Standard/BasicTasks.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Game/StandbyState.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Game/ReadyState.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Game/PlayingState.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Game/PlayingStatePositioningTest.h"

// DBHC
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025DynamicBallHandling/AttackerPlayingState.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025DynamicBallHandling/DefenderPlayingState.h"

// Leaderboard challenges
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Leaderboards/FastestWalk.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Leaderboards/BallControl.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Control/RE2025Leaderboards/MostPasses.h"

// other behaviours

// //Enter in our LookForwardTask
// #include "Modules/BehaviorControl/RECoroBehaviour/Control/LookBackWalkTests/LookBackWalkTest.h"
// //Enter in StrafingDefenderTask
// #include "Modules/BehaviorControl/RECoroBehaviour/Control/LookBackWalkTests/StrafingDefender.h"
// 
// //Enter in ObstacleTimeoutTask
// #include "Modules/BehaviorControl/RECoroBehaviour/Control/LookBackWalkTests/TestObstacleTimeout.h"



#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Annotation.h"
#include "Framework/Settings.h"

#include <regex>
#include <iostream>


DECL_TLOGGER(tlogger, "CoroBehaviourModule2025", TextLogging::INFO);


// MAKE_MODULE(CoroBehaviourModule2025, behaviorControl, CoroBehaviourModule2025::getExtModuleInfo);
MAKE_MODULE(CoroBehaviourModule2025, CoroBehaviourModule2025::getExtModuleInfo);



CoroBehaviourModule2025::CoroBehaviourModule2025()
    : env(initRegisteredRepresentations(), const_cast<ActivationGraph &>(theActivationGraph)), theCoroBehaviourRegistry(env) //,
      // theSkillRegistry("skills.cfg", const_cast<ActivationGraph &>(theActivationGraph), theArmMotionRequest,
      //                  theBehaviorStatus, theCalibrationRequest, theHeadMotionRequest, theMotionRequest, theTeamTalk),
      // theCardRegistry(const_cast<ActivationGraph &>(theActivationGraph))
{
  registerEntryPointBehaviours();

  // for cards and skills compatibility
  // theSkillRegistry.checkSkills(CardCreatorBase::gatherSkillInfo(CardCreatorList<Card>::first));
}

std::vector<ModuleBase::Info> CoroBehaviourModule2025::getExtModuleInfo()
{
  auto result = CoroBehaviourModule2025::getModuleInfo();

  // SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
  // CardCreatorBase::addToModuleInfo(CardCreatorList<Card>::first, result);

  return result;
}


void CoroBehaviourModule2025::update(ActivationGraph &activationGraph)
{
  // FIXME: hack declare debug modify used in behaviour here
  DECLARE_DEBUG_RESPONSE("behaviour:StrikerPlayingTask:kickStraightAhead");


  // FIXME: hack declare drawing used in behaviour here
  DECLARE_DEBUG_DRAWING("behaviour:ObstacleSkills:ballKickSectorWheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behaviour:GotoBallSkills:passTargetOnField", "drawingOnField");

  // declare debug drawings from behaviours
  for (const RE::BehaviourEnv::DebugDrawingInfo& info : env.getDrawingDeclarations())
  {
    Global::getDrawingManager().addDrawingId(info.drawingId, info.drawingType);
    DECLARE_DEBUG_RESPONSE(info.debugResponseId);
  }


  activationGraph.graph.clear();
  // activationGraph.currentDepth = 0;
  env.activationGraphDepth = 0;

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

//   // cards and skills compatibility
//   theSkillRegistry.modifyAllParameters();
//   theCardRegistry.modifyAllParameters();
// 
//   theSkillRegistry.preProcess(theFrameInfo.time);
//   theCardRegistry.preProcess(theFrameInfo.time);

  // run the behaviour for this tick

  ASSERT(activationGraph.graph.empty());
  // add: option_name, depth, state_name, optionTime, stateTime, stringVecOfParams
  activationGraph.graph.emplace_back("CoroBehaviourModule2025", 0, TypeRegistry::getEnumName(status), theFrameInfo.time,
                                     theFrameInfo.time - statusStartTime, std::vector<std::string>());

#ifdef TARGET_SIM // TODO - not in 2021 BehaviorControl.cpp
  MODIFY("module:CoroBehaviourModule2025:status", status);
#endif

  executeBehaviourTick();

  activationGraph.graph[0].state = TypeRegistry::getEnumName(status);

  // // cards and skills compatibility
  // theCardRegistry.postProcess();
  // theSkillRegistry.postProcess();


  // theLibCheck.performCheck(theMotionRequest);
  theLibCheck.performCheck();

  draw();
}


// // some very simple FSM macros inspired by CABSL that help to tidy up the following
// // function a little bit
// #define FSM_STATE(s)   \
//   if (false) { goto fsm_state_##s; /* <<< to avoid unused label warning */ fsm_state_##s: status = s; } \
//   if (status == s)
// 
// // switch to s immediately in the current cycle
// #define GOTO_STATE(s)  goto fsm_state_##s
// 
// // run s in the next cycle
// #define NEXT_STATE(s)  status = s; return



STREAMABLE(SayText,
{,
  (std::string) text,
  (float)(1.f) speed,
  (float)(1.f) pitchFactor,
  (float)(1.f) pitchSdFactor,
});

void CoroBehaviourModule2025::executeBehaviourTick()
{
  using namespace RE;

  SayText sayText;

  MODIFY_ONCE("module:CoroBehaviourModule2025:say", sayText);
  if (!sayText.text.empty())
    SystemCall::say(sayText.text, /* force */ false, sayText.speed); //, sayText.pitchFactor, sayText.pitchSdFactor);



  MODIFY("module:CoroBehaviourModule2025:status", status);

  if (firstFrame)
  {
    firstFrame = false;
    if (SystemCall::getMode() == SystemCall::physicalRobot)
      SystemCall::playSound("wearoff.wav"); // traditional RoboEireann startup sound
  }

  // before entering the state machine part of the function respond to button based information requests
  checkUsbMountedButtonRequest();
  checkJointTemperatureButtonRequest();

  if (handlePhysicalRobot())
    return;
  else if (handlePlayerState())
    return;

  // handle basic player active/playing status
  {
    TLOGD(tlogger, "{} [{}]: playing", theGameState.playerNumber, theFrameInfo.time);

    status = playing;

    // execute the top level normal play behaviour
    CALL_BASIC_CORO(topLevelBehaviour);
  }
}


bool CoroBehaviourModule2025::handlePlayerState()
{
  switch (theGameState.playerState)
  {
  case GameState::unstiff:
    status = sittingDown;
    return sitDownUnstiff(), true;

  case GameState::calibration:
    // TODO: call calibrationControl behaviour - however we can't mix behaviour schemes currently
    status = sittingDown;
    sitDownUnstiff(); // FIXME - replace with calibration procedure
    return true;

  case GameState::prepare:
    // K1-specific: must be entered before handing over control to our motion code. This is
    // neither penalized nor substitute, so it must not fall through to the default branch below -
    // that branch announced "Penalized" and treated the robot as penalized while it was simply
    // waiting here, sometimes for a long time (real match: robots got stuck repeating "Penalized"
    // and never reached active/playing).
    // This is a stiff stand, not sitDownUnstiff()/compliant - the STAND button that triggers this
    // state means "stand up", and MotionEngine.cpp deliberately suppresses automatic get-up motions
    // while playerState == prepare so it doesn't fight this deliberate stand.
    status = inactive;
    headSkills.lookAtAngles(JointAngles::off, JointAngles::off);
    motionSkills.prepare();
    //
    return true;

  case GameState::active:
    if (theExtendedGameState.wasPenalized())
    {
      ANNOTATION("Behavior", "Unpenalized");
      SystemCall::say("Not penalized");
    }
    return false; // we didn't really handle the playing state yet

  default: // playerState is any penalized state or is a substitute
    if (!theExtendedGameState.wasPenalized())
    {
      ANNOTATION_FMT("Behavior", "Penalized {}", TypeRegistry::getEnumName(theGameState.playerState));
      SystemCall::say("Penalized");
    }
    status = penalized;
    return commonSkills.standLookForward(/* high */ true), true;
  }

  // if we get this far we didn't match the player state
  TLOG_ABORT(tlogger, "Unknown player state.");
}


bool CoroBehaviourModule2025::handlePhysicalRobot()
{
  // camera status
  if (!theCameraStatus.ok)
  {
    status = cameraStatusFailed;
    return sitDownUnstiff(), true;
  }
#ifndef NDEBUG
  // low battery
  else if (theRobotHealth.batteryLevel <= 1)
  {
    status = lowBattery;
    return sitDownUnstiff(), true;
  }
#endif

  return false;
}


void CoroBehaviourModule2025::sitDownUnstiff()
{
  headSkills.lookAtAngles(JointAngles::off, JointAngles::off);
  motionSkills.unstiff();
}


void CoroBehaviourModule2025::checkUsbMountedButtonRequest()
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



void CoroBehaviourModule2025::checkJointTemperatureButtonRequest()
{
  if ((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0) ||
      (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 &&
       theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
  {
    SystemCall::say(fmt::format("{} hottest joint is {}. It is {} degrees.", 
                                theGameState.getPlayerNumberAndColorString(), 
                                Joints::jointLongNames[theRobotHealth.jointWithMaxTemperature],
                                theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]));
  }
}


RE::BehaviourEnv::RepresentationRegistry CoroBehaviourModule2025::initRegisteredRepresentations()
{
  RE::BehaviourEnv::RepresentationRegistry representationRegistry;

#define CORO_REPRESENTATIONS_FOR_BEHAVIOUR
#include "CoroBehaviourRepresentations2025.inc"
#undef CORO_REPRESENTATIONS_FOR_BEHAVIOUR

  return representationRegistry;
}



#define REGISTER_CORO(name) theCoroBehaviourRegistry.registerCoro<name>(#name)

void CoroBehaviourModule2025::registerEntryPointBehaviours()
{
  // Register any basic (single robot) ehaviours to be looked up from this registry here

  using namespace RE;

  // standard tasks (used by most)
  REGISTER_CORO(SoccerGameControlTask);
  REGISTER_CORO(BasicSetStateTask);
  REGISTER_CORO(BasicReturnFromPenalizedTask);
  REGISTER_CORO(StandLookActiveTask);
  REGISTER_CORO(StandLookForwardTask);
  REGISTER_CORO(PlaceholderTask);
  REGISTER_CORO(PlaceholderSuccessTask);

  // RC2025 Leaderboard challenges
  REGISTER_CORO(v2025::Leaderboards::FastestWalkTask);
  REGISTER_CORO(v2025::Leaderboards::BallControlTask);
  REGISTER_CORO(v2025::Leaderboards::MostPassesPlayingStateTask);

  // RE2025 Game specific tasks
  REGISTER_CORO(v2025::StandbyStateTask);
  REGISTER_CORO(v2025::ReadyStateTask);
  REGISTER_CORO(v2025::PlayingStateTask);
  REGISTER_CORO(v2025::PlayingStatePositioningTestTask);
  
  // RE2025 dynamic ball handling challenge revisited
  REGISTER_CORO(v2025::DBHC::AttackerPlayingStateTask);
  REGISTER_CORO(v2025::DBHC::DefenderPlayingStateTask);
  
  // kick length calibration
  REGISTER_CORO(v2022::KickLengthCalibrationAssistantTask);
  // kick tests
  REGISTER_CORO(v2024::KickTestsAssistantTask);
  
  TLOGD(tlogger, "behaviours registered with cororegistry");
}

#undef REGISTER_CORO



void CoroBehaviourModule2025::draw()
{
  DEBUG_DRAWING3D("module:CoroBehaviour2025:refereeStandby", "field")
  {
    // draw the stick figure referee
    float yBehindSideline = theFieldDimensions.yPosLeftTouchline + 300;

    // legs
    LINE3D("module:CoroBehaviour2025:refereeStandby", -150, yBehindSideline, 0, -150, yBehindSideline, 750, 40, ColorRGBA::black);
    LINE3D("module:CoroBehaviour2025:refereeStandby", 150, yBehindSideline, 0, 150, yBehindSideline, 750, 40, ColorRGBA::black);

    LINE3D("module:CoroBehaviour2025:refereeStandby", -150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);
    LINE3D("module:CoroBehaviour2025:refereeStandby", 150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);

    // torso + head
    LINE3D("module:CoroBehaviour2025:refereeStandby", 0, yBehindSideline, 800, 0, yBehindSideline, 1700, 40, ColorRGBA::black);

    // shoulders
    LINE3D("module:CoroBehaviour2025:refereeStandby", -220, yBehindSideline, 1350, 220, yBehindSideline, 1350, 40, ColorRGBA::black);

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
      LINE3D("module:CoroBehaviour2025:refereeStandby", shoulder.x() * side, shoulder.y(), shoulder.z(),
              elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
      LINE3D("module:CoroBehaviour2025:refereeStandby", elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
              handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);

      LINE3D("module:CoroBehaviour2025:refereeStandby", -shoulder.x() * side, shoulder.y(), shoulder.z(),
              -elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
      LINE3D("module:CoroBehaviour2025:refereeStandby", -elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
              -handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);
    }
  }
}
