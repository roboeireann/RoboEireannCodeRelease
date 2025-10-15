/**
 * @file ReadyGestureDetector.h
 *
 * Module that detects the referee READY gesture from the camera image
 *
 * @author James Petri
 * @author Shauna Recto
 * @author Eoin Keatley (Robot Detection in opponents half)
 */

#pragma once

#include "ReadyGestureDetectionModel.h"

#include "Representations/Perception/StandbyReadyGesture.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

#include "Platform/Thread.h"

MODULE(ReadyGestureDetector,
{,
  REQUIRES(CameraImage),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(ExtendedGameInfo),
  PROVIDES(StandbyReadyGesture),

  LOADS_PARAMETERS(
  {,
    (std::string) modelName,
    (std::string) labelsName,
    (bool) isFloatInput,
    (bool) isEnabled,
    (float) gestureConfidenceThreshold,
    (int) cropPosX,
    (int) cropPosY,
    (int) startDelay,
    (int) detectionStartTime,
  }),
});

class ReadyGestureDetector : public ReadyGestureDetectorBase
{
public:
  ReadyGestureDetector();

private:
  std::string result;
  std::unique_ptr<ReadyGestureDetectionModel> model;
  float gestureConfidence;
  bool detected;
  bool opponentsInTheirHalfDetected;
  bool opponentMotionInStandby;
  bool walkingIn;

  // unsigned int previousGameState;
  unsigned int timeStartOfStandby;
  unsigned int detectionStartTime;
  unsigned int penaltyTimeCheckStartTime;

  void update(StandbyReadyGesture& info) override;
  
  void check();
  bool canSeeReferee();

  bool canProcceedOntoPitch();
  bool detectRobotsInOpponentHalf();
};
