/**
 * @file RefereeGestureDetector.h
 *
 * Module that detects referee gestures from the camera image
 *
 * @author Rudi Villing
 */

#pragma once

#include "RefereeGestureDetectorModel.h"
#include "DetectedGesture.h"

#include "Representations/Perception/RefereeGesture.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Tools/Module/Module.h"
#include "Tools/RingBuffer.h"


MODULE(RefereeGestureDetector,
{,
  REQUIRES(CameraImage),
  REQUIRES(FrameInfo),

  PROVIDES(RefereeGesture),
});


class RefereeGestureDetector : public RefereeGestureDetectorBase
{
public:
  RefereeGestureDetector();

private:
  STREAMABLE(ModifiableGesture,
  {,
    (RefereeGesture::Gesture)(RefereeGesture::NONE) gesture,  ///< the gesture
    (RefereeGesture::Team)(RefereeGesture::NO_TEAM) team,     ///< which team the gesture referred to
  });

  float fullTimeForearmPhase; // ranges between 0 and 1, used for debug drawing only
  float fullTimePhaseIncrement;

  RefereeGestureDetectorModel gestureDetectorModel;

  RingBuffer< DetectedGesture::GestureOneHot, 8> detectedGestures;

  void update(RefereeGesture& refereeGesture) override;
};
