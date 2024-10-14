/**
 * @file ReadyGestureDetector.h
 *
 * Module that detects the referee READY gesture from the camera image
 *
 * @author James Petri
 * @author Shauna Recto
 */

#pragma once

#include "ReadyGestureDetectionModel.h"

#include "Representations/Perception/StandbyReadyGesture.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"

#include "Platform/Thread.h"

MODULE(ReadyGestureDetector,
{,
  REQUIRES(CameraImage),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
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

  unsigned int previousGameState;
  unsigned int timeStartOfStandby;

  void update(StandbyReadyGesture& info) override;
  
  void check();
  bool canSeeReferee();
};