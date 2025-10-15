/**
 * @file SetPlayGestureDetector.h
 *
 * Module that detects the referee gestures from the camera image for set plays
 *
 * @author Rudi Villing
 * @author Conal Hughes
 */

#pragma once

#include "SetPlayGestureDetectionModel.h"

#include "Representations/Perception/SetPlayRefereeGesture.h"
#include "Representations/Perception/ImagePreprocessing/ECImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

#include "Platform/Thread.h"

MODULE(SetPlayGestureDetector,
{,
  REQUIRES(CameraImage),
  REQUIRES(ECImage),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(SetPlayRefereeGesture),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBehaviorStatus),

  PROVIDES(SetPlayRefereeGesture),

  LOADS_PARAMETERS(
  {,
    (bool) isEnabled,
    (std::string) modelName,
    (std::string) labelsName,
    (bool) isFloatInput,
    (float) gestureConfidenceThreshold,
    (int) cropOffsetX,
    (int) cropOffsetY,
    (int) startInferenceOffsetMs,
    (int) endInferenceOffsetMs,
    (float) ballSeenTimeout, ///< time in ms during which the ball must have been seen in order to rely on own ball model
  }),
});

class SetPlayGestureDetector : public SetPlayGestureDetectorBase
{
public:
  SetPlayGestureDetector();

private:
  std::string result;
  std::unique_ptr<SetPlayGestureDetectionModel> model;

  float gestureConfidence;
  bool detected;
  // SetPlayRefereeGesture::Gesture detectedGesture;

  unsigned int previousGameState;
  unsigned int timeStartOfStandby;

  void update(SetPlayRefereeGesture& info) override;
  
  /**
   * request a new inference if none was already in progress and return the
   * result of any previous inference that was in progress
   */
  void checkInference(SetPlayRefereeGesture& gestureInfo); 

  bool canSeeReferee(); // is this robot in a position to see the referee - no point running inference if not
};