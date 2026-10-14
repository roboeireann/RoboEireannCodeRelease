/**
 * @file GamePadControlK1.cpp
 *
 * A module to control the K1 robot using its GamePad (by passing behaviour).
 * It provides the motion request and head motion request directly.
 *
 * Keymap:
 *
 * LB + back (-)      | disable control
 * LB + start (+)     | enable control
 * Y                  | stand (active) - required before any walking/kicking
 *
 * L stick            | walk fwd/back/sideways/diagonal
 * R stick            | walk rotation
 * Hat                | pan and tilt head
 * LB + Hat Up        | move head to 0,0
 *
 * RT + L/R stick     | dribble TODO
 * 
 * RT + A             | Kick (right foot)
 * RT + B             | Kick (left foot)
 *
 * @author Rudi Villing
 */

#include "GamePadControlK1.h"
#include "Platform/SystemCall.h"
#include "TextLogging/TextLogging.h"



MAKE_MODULE(GamePadControlK1);

DECL_TLOGGER(tlogger, "GamePadControlK1", TextLogging::INFO);


void GamePadControlK1::update(MotionRequest& theMotionRequest)
{
  // frameNumber++;

  updateInterval.stop();
  if (updateInterval.sum() > 5000) // avg over about 5 secs
  {
    estimatedIntervalSecs = (float) updateInterval.avg();
    updateInterval.reset();
  }
  updateInterval.start();

  theMotionRequest.odometryData = theOdometryData;
  bool gamePadMotionAllowed = true;

  if (theRobotSafetyState.state != RobotSafetyState::operational)
  {
    if (theMotionRequest.motion != MotionRequest::playDead)
      TLOGI(tlogger, "safety state not operational (current motion={}), so playDead",
            TypeRegistry::getEnumName(theMotionRequest.motion));

    setPlayDeadMotion(theMotionRequest);
    gamePadMotionAllowed = false;
  }
  else if (theGameState.playerState == GameState::unstiff)
  {
    setPlayDeadMotion(theMotionRequest);
    gamePadMotionAllowed = false;
  }
  else if (theGameState.playerState == GameState::prepare)
  {
    setPrepareMotion(theMotionRequest);
    gamePadMotionAllowed = false;
  }

  if (theGameState.isPenalized() && !theExtendedGameState.wasPenalized())
  {
    SystemCall::say("Penalized");
    gamePadMotionAllowed = false;
  }
  else if (!theGameState.isPenalized() && theExtendedGameState.wasPenalized())
  {
    SystemCall::say("Not penalized");
  }

  // game pad operations that don't modify the head/motion request
  // -------------------------------------------------------------------------

  if (theGamePadEx.connected)
  {
    if (controlEnabled && theGamePadEx.justPressed(GamePad::maskLB | GamePad::maskBack))
    {
      controlEnabled = false;
      TLOGI(tlogger, "GamePad safe (control disabled)");
      SystemCall::say("Game pad safe");
    }
    else if (!controlEnabled && theGamePadEx.justPressed(GamePad::maskLB | GamePad::maskStart))
    {
      controlEnabled = true;
      TLOGI(tlogger, "GamePad active (control enabled)");
      SystemCall::say("Game pad active");
    }
  }

  if (!gamePadMotionAllowed)
    return;

  // game pad operations that DO modify the head/motion request
  // only allowed when playerState == active and gamePadMotionAllowed
  // -------------------------------------------------------------------------

  if (theGamePadEx.connected && controlEnabled)
  {
    if (theGameState.playerState != GameState::active)
      return;

    updateHeadMotion(headMotionRequest);

    // else if (theGamePadEx.pressed(GamePad::maskLB | GamePad::maskX) && (theMotionRequest.motion != MotionRequest::playDead))
    // {
    //   theMotionRequest.motion = MotionRequest::playDead;
    //   theMotionRequest.energySavingWalk = false;
    //   theMotionRequest.standHigh = true;
    //   TLOGI(tlogger, "requesting MotionRequest::playDead");
    // }
    // else if (theGamePadEx.pressed(GamePad::maskLB | GamePad::maskB) && (theMotionRequest.motion != MotionRequest::prepare))
    // {
    //   theMotionRequest.motion = MotionRequest::prepare;
    //   theMotionRequest.energySavingWalk = false;
    //   theMotionRequest.standHigh = true;
    //   TLOGI(tlogger, "requesting MotionRequest::prepare");
    // }
    if (theGamePadEx.justPressed(GamePad::maskY))
    {
      theMotionRequest.motion = MotionRequest::stand;
      theMotionRequest.energySavingWalk = false;
      theMotionRequest.standHigh ^= true; // toggle stand high
      TLOGI(tlogger, "requesting MotionRequest::stand, high={}", theMotionRequest.standHigh);
    }
    else if (!updateKickMotion(theMotionRequest))
    {
      updateWalkMotion(theMotionRequest);
    }
  }
  else // GamePad not connected
  {
    // if (gamePadMotionAllowed)
    { 
      // stopGracefully(theMotionRequest);
      setStandMotion(theMotionRequest);
      setHeadZero(); // since GamePad not connected any longer
    }
  }
}


void GamePadControlK1::setHeadZero(bool keepJointsEnabled)
{
  headMotionRequest.mode = HeadMotionRequest::panTiltMode;
  headMotionRequest.cameraControlMode = HeadMotionRequest::autoCamera;
  headMotionRequest.pan = keepJointsEnabled ? 0_deg : JointAngles::off;
  headMotionRequest.tilt = keepJointsEnabled ? 0_deg : JointAngles::off;
  headMotionRequest.speed = panTiltSpeed;
  headMotionRequest.stopAndGoMode = false;
}


void GamePadControlK1::setPlayDeadMotion(MotionRequest &theMotionRequest)
{
  if (theMotionRequest.motion != MotionRequest::playDead)
    TLOGI(tlogger, "requesting MotionRequest::playDead (with head joints off)");

  theMotionRequest.motion = MotionRequest::playDead;
  theMotionRequest.energySavingWalk = false;
  theMotionRequest.standHigh = true;

  setHeadZero(false);
}


void GamePadControlK1::setPrepareMotion(MotionRequest &theMotionRequest)
{
  if (theMotionRequest.motion != MotionRequest::prepare)
    TLOGI(tlogger, "requesting MotionRequest::prepare (with head zero'd)");

  theMotionRequest.motion = MotionRequest::prepare;
  theMotionRequest.energySavingWalk = false;
  theMotionRequest.standHigh = true;

  setHeadZero();
}


void GamePadControlK1::setStandMotion(MotionRequest &theMotionRequest)
{
  if (theMotionRequest.motion != MotionRequest::stand)
    TLOGI(tlogger, "requesting MotionRequest::stand");

  theMotionRequest.motion = MotionRequest::stand;
  theMotionRequest.energySavingWalk = false;
  theMotionRequest.standHigh = true;

  // leave the head as it was
}


void GamePadControlK1::updateHeadMotion(HeadMotionRequest& theHeadMotionRequest)
{
  if (theGamePadEx.pressed(GamePad::maskLB))
  {
    if (theGamePadEx.hat != GamePad::centered) // any hat button pressed
      setHeadZero();
  }
  else 
  {
    const float dt = estimatedIntervalSecs;

    Angle deltaPan = 0_deg;
    Angle deltaTilt = 0_deg;

    switch (theGamePadEx.hat)
    {
    case GamePad::up: deltaTilt = panTiltSpeed * dt; break;
    case GamePad::down: deltaTilt = -panTiltSpeed * dt; break;
    case GamePad::left: deltaPan = panTiltSpeed * dt; break;
    case GamePad::right: deltaPan = -panTiltSpeed * dt; break;
    default: break; // centered + all hat diagonals ignored
    }


    theHeadMotionRequest.pan = std::clamp(Angle(theJointAngles.angles[Joints::headYaw] + deltaPan),
                                          theHeadLimits.minPan(), theHeadLimits.maxPan());

    Rangea tiltLimits = theHeadLimits.getTiltBound(theHeadMotionRequest.pan);
    theHeadMotionRequest.tilt =
        std::clamp(Angle(theJointAngles.angles[Joints::headPitch] + deltaTilt), tiltLimits.min, tiltLimits.max);

    theHeadMotionRequest.speed = panTiltSpeed;
  }
}


bool GamePadControlK1::updateKickMotion(MotionRequest& theMotionRequest)
{
  if (!theGamePadEx.pressed(GamePad::maskRT)) // are we doing any sort of kicking?
    return false;
  if (!theGamePadEx.anyPressed(GamePad::maskA | GamePad::maskB))
    return false;

  // A triggers the right-foot RL kick, B the left-foot one.
  const KickInfo::KickType kickType = theGamePadEx.pressed(GamePad::maskB) ? KickInfo::standRLK1KickLongLeft : KickInfo::standRLK1KickLongRight;


  auto applyDeadZone = [this](float value)
  { 
    return (std::abs(value) <= deadZone ? 0.f : (value - deadZone * sgn(value)) / (1.f - deadZone)); 
  };

  // for kicks, the motion engine decides where to walk based on the ball position
  // so the GamePad x,y input only say how fast to move, not whether to move in x
  // or in y. Hence we just extract the max magnitude of these inputs.

  const float speed = std::max(std::abs(applyDeadZone(theGamePadEx.lStickX)),
                               std::abs(applyDeadZone(theGamePadEx.lStickY)));

  theMotionRequest.motion = MotionRequest::walkToBallAndKick;
  theMotionRequest.kickType = kickType;
  theMotionRequest.walkSpeed = {speed, speed, speed};
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.directionPrecision = {-5_deg, 5_deg};
  theMotionRequest.standHigh = true;
  theMotionRequest.energySavingWalk = false;

  if (intervalLogKick.updateTriggered(5000u))
    TLOGI(tlogger, "requesting MotionRequest::walkToBallAndKick {}",
          TypeRegistry::getEnumName(theMotionRequest.kickType));

  // TODO in future add possibility of dribble either here or in a new method

  return true;
}



void GamePadControlK1::updateWalkMotion(MotionRequest& theMotionRequest)
{
  auto applyDeadZone = [this](float value)
  { 
    return (std::abs(value) <= deadZone ? 0.f : (value - deadZone * sgn(value)) / (1.f - deadZone)); 
  };

  theMotionRequest.energySavingWalk = false;
  theMotionRequest.standHigh = false;

  theMotionRequest.walkSpeed.translation.x() = applyDeadZone(theGamePadEx.lStickY);
  // Note: lStickX left=-1, right=+1 but robot Y is positive-left, so negate
  theMotionRequest.walkSpeed.translation.y() = -applyDeadZone(theGamePadEx.lStickX);
  // Note: rStickX left=-1, right=+1 but rotation is positive-left, so negate
  theMotionRequest.walkSpeed.rotation = -applyDeadZone(theGamePadEx.rStickX);

  if (theMotionRequest.walkSpeed != Pose2f())
    theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
  else
    theMotionRequest.motion = MotionRequest::stand;

  if (intervalLogWalk.updateTriggered(5000u))
    TLOGI(tlogger, "requesting MotionRequest::{} speed={{ rot={:.2f}, x={:.2f}, y={:.2f} }}",
          TypeRegistry::getEnumName(theMotionRequest.motion), (float)theMotionRequest.walkSpeed.rotation,
          theMotionRequest.walkSpeed.translation.x(), theMotionRequest.walkSpeed.translation.y());
}


// no longer need
// void GamePadControlK1::stopGracefully(MotionRequest& theMotionRequest)
// {
//   using State = StopGracefullyState;
//   using FsmType = Fsm<StopGracefullyState>;
//   FsmType& fsm = stopGracefullyFsm;
// 
//   auto transitionTo = [&](State newState)
//   {
//     fsm.state = newState;
//     fsm.firstFrame = true;
//   };
// 
// 
//   // handle auto-reset
//   fsm.autoResetIfNeeded(frameNumber);
//   
//   if (fsm.state == State::start)
//   {
//     switch (theMotionRequest.motion)
//     {
//     case MotionRequest::playDead: transitionTo(State::playDead);
//     case MotionRequest::prepare: transitionTo(State::prepare);
// 
//     case MotionRequest::walkAtAbsoluteSpeed:
//     case MotionRequest::walkAtRelativeSpeed:
//     case MotionRequest::walkToPose:
//     case MotionRequest::walkToBallAndKick:
//     case MotionRequest::dribble: transitionTo(State::stopWalking);
// 
//     default: transitionTo(State::stand);
//     }
//   }
// 
//   // read firstFrame after initial_state so any transition it made is seen correctly
//   bool firstFrame = fsm.firstFrame;
//   fsm.firstFrame = false;  
// 
//   // WARNING WARNING WARNING
//   // The following FSM works without some additional tricks like goto labels
//   // because all transitions go forward to states written after. If this ever
//   // changes you will either need to add a loop mechanism (care needed) or
//   // add something like the if (false) { label stateName: .... } trick of CABSL
// 
//   if (fsm.state == State::stopWalking)
//   {
//     if (!firstFrame)
//     {
//       if (theMotionInfo.speed.translation.norm() == 0.f && theMotionInfo.speed.rotation == 0.f)
//         transitionTo(State::prepare);
//     }
//     {
//       if (firstFrame)
//         TLOGI(tlogger, "stopGracefully: stopWalking");
// 
//       theMotionRequest.motion = MotionRequest::walkAtRelativeSpeed;
//       theMotionRequest.walkSpeed = {0.f, 0.f, 0.f};
//       theMotionRequest.energySavingWalk = false;
//       theMotionRequest.standHigh = true;
// 
//       setHeadZero();
//     }
//   }
// 
//   if (fsm.state == State::stand)
//   {
//     if (!firstFrame)
//     {
//       if (theMotionInfo.executedPhase == MotionPhase::stand)
//         transitionTo(State::prepare);
//     }
//     {
//       if (firstFrame)
//         TLOGI(tlogger, "stopGracefully: stand");
//         
//       theMotionRequest.motion = MotionRequest::stand;
//       theMotionRequest.energySavingWalk = false;
//       theMotionRequest.standHigh = true;
// 
//       setHeadZero();
//     }
//   }
// 
//   if (fsm.state == State::prepare)
//   {
//     fsm.done = true;
// 
//     {
//       if (firstFrame)
//         TLOGI(tlogger, "stopGracefully: prepare");
// 
//       setPrepareMotion(theMotionRequest);
//     }
//   }
// 
//   if (fsm.state == State::playDead)
//   {
//     fsm.done = true;
// 
//     {
//       if (firstFrame)
//         TLOGI(tlogger, "stopGracefully: playDead");
// 
//       setPlayDeadMotion(theMotionRequest);
//     }
//   }
// }
