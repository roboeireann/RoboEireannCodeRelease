/**
 * @file GamePadControlK1.h
 *
 * A module to control the K1 robot using its GamePad (by passing behaviour).
 * It provides the motion request and head motion request directly.
 *
 * @author Rudi Villing
 */

#pragma once

#include "Framework/Module.h"
#include "Math/BHMath.h"
// #include "Representations/BehaviorControl/JoystickState.h"
#include "Representations/Infrastructure/SensorData/GamePad.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/RobotSafetyState.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Tools/RECommon/IntervalTrigger.h"
#include "Tools/RECommon/TimingStats.h"

MODULE(GamePadControlK1,
{,
  REQUIRES(BallModel),
  REQUIRES(GamePadEx),
  REQUIRES(GameState),
  REQUIRES(ExtendedGameState),
  REQUIRES(OdometryData),
  REQUIRES(RobotSafetyState),
  REQUIRES(MotionInfo),
  REQUIRES(JointAngles),
  REQUIRES(HeadLimits),

  REQUIRES(MotionRequest), // force it to be updated first
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),

  DEFINES_PARAMETERS(
  {,
    (float)(0.15f) deadZone, ///< Only start walking if the speed is high enough.
    (Angle)(60_deg) panTiltSpeed, ///< degrees per second
  }),
});

class GamePadControlK1 : public GamePadControlK1Base
{
protected:
  bool controlEnabled = false;
  HeadMotionRequest headMotionRequest; /**< The head motion request provided. */
  IntervalTriggerMs intervalLogKick;
  IntervalTriggerMs intervalLogWalk;
  IntervalTimingStats updateInterval;
  float estimatedIntervalSecs = 0.033f; // initial estimate


  /**
   * This method is called when the representation provided needs to be updated.
   * @param theHeadMotionRequest The representation updated.
   */
  void update(HeadMotionRequest& theHeadMotionRequest) override {theHeadMotionRequest = headMotionRequest;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theMotionRequest The representation updated.
   */
  void update(MotionRequest& theMotionRequest) override;

  bool updateKickMotion(MotionRequest& theMotionRequest); // returns false if not kicking
  void updateWalkMotion(MotionRequest& theMotionRequest);
  void updateHeadMotion(HeadMotionRequest& theHeadMotionRequest);

  void setPlayDeadMotion(MotionRequest& theMotionRequest);
  void setPrepareMotion(MotionRequest& theMotionRequest);
  void setStandMotion(MotionRequest& theMotionRequest);
  void setHeadZero(bool keepJointsEnabled=true);
  void setHeadPanTilt(Angle pan, Angle tilt, Angle speed); 

  // have to use a ptr here as CABSL strips references to pass by value which is
  // no use when we want to modify the thing referred to.
//   enum class StopGracefullyState { start, stopWalking, stand, prepare, playDead };
//   template <typename StateType>
//   struct Fsm
//   {
//     unsigned frameNumber = 0;
//     StateType state = static_cast<StateType>(0);
//     bool firstFrame = true;
//     bool done = false;
// 
//     void reset(unsigned frameNumberIn)
//     {
//       frameNumber = frameNumberIn;
//       state = static_cast<StateType>(0);
//       firstFrame = true;
//       done = false;
//     }
// 
//     /// returns true if we did autoreset
//     bool autoResetIfNeeded(unsigned frameNumberIn)
//     {
//       if (++frameNumber != frameNumberIn)
//       {
//         reset(frameNumberIn);
//         return true;
//       }
// 
//       return false;
//     }
//   };
//   Fsm<StopGracefullyState> stopGracefullyFsm;
//   unsigned frameNumber = 0;
//   void stopGracefully(MotionRequest& theMotionRequest);
};
