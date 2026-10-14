/**
 * @file REHybridBehaviourModule.cpp
 *
 * This is the module that first runs the team coordination behaviour and
 * then runs the behaviour for a single robot.
 *
 * @author Rudi Villing
 */

#include "REHybridBehaviourModule.h"

#include "TextLogging/TextLogging.h"

// other behaviours

#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Thread.h"

#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Annotation.h"
#include "Framework/Settings.h"

#include <regex>
#include <iostream>


DECL_TLOGGER(tlogger, "REHybridBehaviourModule", TextLogging::INFO);


MAKE_MODULE(REHybridBehaviourModule);


REHybridBehaviourModule::REHybridBehaviourModule()
{
  // check if all required entry point tasks have been registered and fail fast
  // so that we can fix it
  bool allRegistered = true;
  if (!HBS_TASK_IS_REGISTERED(teamTask))
  {
    TLOGE(tlogger, "teamTask '{}' has not been registered", teamTask);
    allRegistered = false;
  }

  for (auto &taskName : fallbackPlayerTasks)
  {
    if (!HBS_TASK_IS_REGISTERED(taskName))
    {
      TLOGE(tlogger, "fallbackPlayerTask '{}' has not been registered", taskName);
      allRegistered = false;
    }
  }

  if (!allRegistered)
    TLOG_ABORT(tlogger, "Some tasks not registered. Use HBS_REGISTER_TASK for each task specified in the reHybridBehaviour.cfg file");
}

// std::vector<ModuleBase::Info> REHybridBehaviourModule::getExtModuleInfo()
// {
//   auto result = REHybridBehaviourModule::getModuleInfo();
// 
//   // SkillImplementationCreatorBase::addToModuleInfo(SkillImplementationCreatorList<Skill>::first, result);
//   // CardCreatorBase::addToModuleInfo(CardCreatorList<Card>::first, result);
// 
//   return result;
// }


STREAMABLE(SayText,
{,
  (std::string) text,
  (float)(1.f) speed,
  (float)(1.f) pitchFactor,
  (float)(1.f) pitchSdFactor,
});

void REHybridBehaviourModule::update(ActivationGraph &activationGraph)
{
  // TODO: this is a bit hacky but seems to be the nominal way to declare
  // debug things in tasks, since different tasks run in different cycles
  // so we can't rely on a task consistently telling SimRobot what it offers

  DECLARE_DEBUG_RESPONSE("behaviour:StrikerPlayingTask:kickStraightAhead");

  DECLARE_DEBUG_DRAWING("behaviour:ObstacleSkills:ballKickSectorWheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behaviour:GotoBallSkills:passTargetOnField", "drawingOnField");

  // // declare debug drawings from behaviours
  // for (const RE::BehaviourEnv::DebugDrawingInfo& info : env.getDrawingDeclarations())
  // {
  //   Global::getDrawingManager().addDrawingId(info.drawingId, info.drawingType);
  //   DECLARE_DEBUG_RESPONSE(info.debugResponseId);
  // }


  SayText sayText;

  MODIFY_ONCE("behaviour:say", sayText);
  if (!sayText.text.empty())
    SystemCall::say(sayText.text, /* force */ false, sayText.speed); //, sayText.pitchFactor, sayText.pitchSdFactor);



  activationGraph.graph.clear();

  HBS_BEGIN_TICK(theFrameInfo.time);

  // execute the team (coordination) task before any player specific tasks
  HBS_EXECUTE(teamTask);

  // reset the state of representations provided by player behaviour
  theBehaviorStatus.calibrationFinished = false;
  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.passOrigin = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo.reset(); // option value not set

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  theHeadMotionRequest.pan = JointAngles::off;
  theHeadMotionRequest.tilt = JointAngles::off;
  theHeadMotionRequest.speed = 60_deg; // TODO: a safe value? 
  theHeadMotionRequest.stopAndGoMode = false;

  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.shouldInterceptBall =
      theGameState.isPlaying() && !theGameState.isPenalized() &&
      (theFieldInterceptBall.interceptBall || theFieldInterceptBall.predictedInterceptBall);
  theMotionRequest.shouldWalkOutOfBallLine = theFieldBall.isRollingTowardsOpponentGoal;
  theMotionRequest.targetOfInterest.reset();

  // let us know when cognition is ready and about to run the first behaviour cycle  
  if (startupAnnouncement.once())
    SystemCall::playSound("startup_chord3.wav");

  // run the player behaviour for this tick like a BT fallback of the available
  // tasks. Stop after the first one that returns RUNNING or SUCCESS. We cannot
  // use the BT_FALLBACK macro in this context.
  for (auto& taskName : fallbackPlayerTasks)
  {
    Hbs::Status s = HBS_EXECUTE(taskName);
    if ((s == Hbs::SUCCESS) || (s == Hbs::RUNNING))
      break;
  }

  // -----------------------------------------------------------------------
  // convert TickTrace to ActivationGraph
  // -----------------------------------------------------------------------

  // ensure nothing tried to add to the activation graph
  ASSERT(activationGraph.graph.empty());

  for (auto& frame : tickTrace.getFrames())
  {
    activationGraph.graph.emplace_back(frame.taskName, frame.depth, frame.stateName, frame.taskDuration,
                                       frame.stateDuration, frame.contextStrings, frame.status, frame.strings);
  }

  // check that we have only tried to set the motionRequest once etc.
  theLibCheck.performCheck();

  // TODO: consider adding back in temperature feedback, general configuration feedback, if we can figure
  // a unique button press sequence to allow it that doesn't mess with game state

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



// void REHybridBehaviourModule::checkUsbMountedButtonRequest()
// {
//   // speak the USB mounted state if the top of the head is triple-tapped
//   if (theEnhancedKeyStates.hitStreak[KeyStates::headMiddle] == 3)
//   {
//     if (SystemCall::usbIsMounted())
//       SystemCall::say("USB mounted");
//     else
//       SystemCall::say("USB not mounted");
//   }
// }



// void REHybridBehaviourModule::checkJointTemperatureButtonRequest()
// {
//   if ((theEnhancedKeyStates.hitStreak[KeyStates::headRear] == 3 &&
//        theEnhancedKeyStates.pressedDuration[KeyStates::headFront] > 0) ||
//       (theEnhancedKeyStates.hitStreak[KeyStates::headFront] == 3 &&
//        theEnhancedKeyStates.pressedDuration[KeyStates::headRear] > 0))
//   {
//     SystemCall::say(fmt::format("{} hottest joint is {}. It is {} degrees.", 
//                                 theGameState.getPlayerNumberAndColorString(), 
//                                 Joints::jointLongNames[theRobotHealth.jointWithMaxTemperature],
//                                 theJointSensorData.temperatures[theRobotHealth.jointWithMaxTemperature]));
//   }
// }



void REHybridBehaviourModule::draw()
{
//   DEBUG_DRAWING3D("module:CoroBehaviour2025:refereeStandby", "field")
//   {
//     // draw the stick figure referee
//     float yBehindSideline = theFieldDimensions.yPosLeftTouchline + 300;
// 
//     // legs
//     LINE3D("module:CoroBehaviour2025:refereeStandby", -150, yBehindSideline, 0, -150, yBehindSideline, 750, 40, ColorRGBA::black);
//     LINE3D("module:CoroBehaviour2025:refereeStandby", 150, yBehindSideline, 0, 150, yBehindSideline, 750, 40, ColorRGBA::black);
// 
//     LINE3D("module:CoroBehaviour2025:refereeStandby", -150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);
//     LINE3D("module:CoroBehaviour2025:refereeStandby", 150, yBehindSideline, 750, 0, yBehindSideline, 800, 40, ColorRGBA::black);
// 
//     // torso + head
//     LINE3D("module:CoroBehaviour2025:refereeStandby", 0, yBehindSideline, 800, 0, yBehindSideline, 1700, 40, ColorRGBA::black);
// 
//     // shoulders
//     LINE3D("module:CoroBehaviour2025:refereeStandby", -220, yBehindSideline, 1350, 220, yBehindSideline, 1350, 40, ColorRGBA::black);
// 
//     // approx arms (based on current gesture)
//     // gestures default to left arm (+ve x side, red team side)
// 
//     // Upper arm 300mm, forearm 300mm, hand 150mm, so we treat total arm incl 
//     // hand as 750mm and forearm incl hand as 450mm
// 
//     Vector3f shoulder(220, yBehindSideline, 1350);
//     // Vector3f handDown(220, yBehindSideline, 1350 - 750);                   // straight down
//     Vector3f handUp(220, yBehindSideline, 1350 + 750);                   // straight down
//     // Vector3f handSide(220 + 750, yBehindSideline, 1350);                   // straight out to side
//     // Vector3f handSideUp(220 + 530, yBehindSideline, 1350 + 530);           // 45 deg up
//     // Vector3f handSideDown(220 + 530, yBehindSideline, 1350 - 530);         // 45 deg down
//     // Vector3f handFrontPoint(220 - 100, yBehindSideline - 600, 1350 - 375); // point to center (very approx)
// 
//     // move requiring elbows...
//     // Vector3f elbowDown(220, yBehindSideline, 1350 - 300);       // straight down
//     Vector3f elbowUp(220, yBehindSideline, 1350 + 300);       // straight down
// //     Vector3f forearmUp(150, yBehindSideline - 150, 1350 + 150); // up overlapping body
// // 
// //     Vector3f elbowSideDown(220 + 210, yBehindSideline, 1350 - 210); // down and to side
// //     float forearmLen = 450.f;
// //     Vector3f forearmForwardOrigin(0, -450, 0); // point forward from elbow
// 
// 
//     float side = 1.f;
// 
//     // standby gesture
//     {
//       LINE3D("module:CoroBehaviour2025:refereeStandby", shoulder.x() * side, shoulder.y(), shoulder.z(),
//               elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
//       LINE3D("module:CoroBehaviour2025:refereeStandby", elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
//               handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);
// 
//       LINE3D("module:CoroBehaviour2025:refereeStandby", -shoulder.x() * side, shoulder.y(), shoulder.z(),
//               -elbowUp.x() * side, elbowUp.y(), elbowUp.z(), 40, ColorRGBA::black);
//       LINE3D("module:CoroBehaviour2025:refereeStandby", -elbowUp.x() * side, elbowUp.y(), elbowUp.z(),
//               -handUp.x() * side, handUp.y(), handUp.z(), 40, ColorRGBA::black);
//     }
//   }
}
