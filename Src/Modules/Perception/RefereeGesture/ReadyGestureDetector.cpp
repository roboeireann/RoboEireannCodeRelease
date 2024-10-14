/**
 * @file ReadyGestureDetector.h
 *
 * Module that detects the referee READY gesture from the camera image
 *
 * @author James Petri
 * @author Shauna Recto
 */

#include "ReadyGestureDetector.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Range.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/TextLogging.h"


DECL_TLOGGER(tlogger, "ReadyGestureDetector", TextLogging::INFO);

MAKE_MODULE(ReadyGestureDetector, perception);

ReadyGestureDetector::ReadyGestureDetector()
{
  model.reset(new ReadyGestureDetectionModel(modelName, labelsName, isEnabled));

  /* Set default values */
  gestureConfidence = 0.f;
  detected = false;

  previousGameState = STATE_INITIAL;
  timeStartOfStandby = 0;
}

void ReadyGestureDetector::update(StandbyReadyGesture& info)
{
  DECLARE_PLOT("module:ReadyGestureDetector:gestureConfidence");
  DECLARE_PLOT("module:ReadyGestureDetector:gestureConfidenceThreshold");
  DECLARE_DEBUG_DRAWING("module:ReadyGestureDetector:cropBox", "drawingOnImage");

  if ((previousGameState != theGameInfo.state) && (theGameInfo.state == STATE_STANDBY))
  {
    timeStartOfStandby = theFrameInfo.time;
    previousGameState = theGameInfo.state;
  }

  // TLOGI(tlogger, "time since standby {}, time start of standby {}, state {}\n", theFrameInfo.getTimeSince(timeStartOfStandby), timeStartOfStandby, theGameInfo.state);

  bool debugReadySignalled = false;
  DEBUG_RESPONSE_ONCE("module:ReadyGestureDetector:signalReady")
  {
    debugReadySignalled = true;
  }

  /* We only check the gesture during standby state */
  if ((isEnabled) && (theGameInfo.state == STATE_STANDBY) && ((theFrameInfo.getTimeSince(timeStartOfStandby)) >= startDelay) &&
      canSeeReferee())
  {
    if (debugReadySignalled)
    {
      info.isDetected = true;
      info.confidence = 0.9f;
    }
    else
    {
      check();
      info.isDetected = detected;
      info.confidence = gestureConfidence;
    }

    if (info.isDetected)
      info.frameTimeLastDetection = theFrameInfo.time;
  }
  else
  {
    info.isDetected = false;
    info.confidence = 0.f;
  }
}

void ReadyGestureDetector::check()
{
    model->setImage(theCameraImage, cropPosX, cropPosY);

    gestureConfidence = 1.f - model->getConfidence();
    detected = (gestureConfidence > gestureConfidenceThreshold);

    PLOT("module:ReadyGestureDetector:gestureConfidence", gestureConfidence);
    PLOT("module:ReadyGestureDetector:gestureConfidenceThreshold", gestureConfidenceThreshold);
}

bool ReadyGestureDetector::canSeeReferee()
{
  switch (theGameInfo.playerNumber)
  {
  case 1:
  case 3:
  case 5:
  case 7:
    return !theGameInfo.isPlayingLeftToRight();
  case 2:
  case 4:
  case 6:
    return theGameInfo.isPlayingLeftToRight();

  default:
    return false;
  }


}