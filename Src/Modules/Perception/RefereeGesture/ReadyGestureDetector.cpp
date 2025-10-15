/**
 * @file ReadyGestureDetector.cpp
 *
 * Module that detects the referee READY gesture from the camera image
 *
 * @author James Petri
 * @author Shauna Recto
 * @author Eoin Keatley (Robot Detection in opponents half)
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

  //previousGameState = STATE_INITIAL;

  //Parameter Reset for Robot Detection In Field
  timeStartOfStandby = 0;
  penaltyTimeCheckStartTime = 0;
}

void ReadyGestureDetector::update(StandbyReadyGesture& info)
{
  DECLARE_PLOT("module:ReadyGestureDetector:gestureConfidence");
  DECLARE_PLOT("module:ReadyGestureDetector:gestureConfidenceThreshold");
  DECLARE_DEBUG_DRAWING("module:ReadyGestureDetector:cropBox", "drawingOnImage");

  if (theExtendedGameInfo.gameStateLastFrame != theGameInfo.state)
  // if ((previousGameState != theGameInfo.state) && (theGameInfo.state == STATE_STANDBY))
  {
    if (theGameInfo.state == STATE_STANDBY) // have we just switched to standby
    {
      timeStartOfStandby = theFrameInfo.time;
      opponentsInTheirHalfDetected = false;
      opponentMotionInStandby = false;
      detectionStartTime = 0; // first time we detected opponent obstacles in their half
      walkingIn = false;

    }
    else // no we've just switched to some other state
    {
      info.isDetected = false;
      info.confidence = 0.0f;
      info.frameTimeLastDetection = 0;
    }
    // previousGameState = theGameInfo.state;
  }

  // TLOGI(tlogger, "time since standby {}, time start of standby {}, state {}\n", theFrameInfo.getTimeSince(timeStartOfStandby), timeStartOfStandby, theGameInfo.state);

  bool debugReadySignalled = false;
  DEBUG_RESPONSE_ONCE("module:ReadyGestureDetector:signalReady")
  {
    debugReadySignalled = true;
  }

  /* We only check the gesture during standby state */
  if ((isEnabled) && (theGameInfo.state == STATE_STANDBY) && ((theFrameInfo.getTimeSince(timeStartOfStandby)) >= startDelay))
  {
    bool wasDetectedLastFrame = info.isDetected;

    if (debugReadySignalled)
    {
      info.isDetected = true;
      info.confidence = 0.9f;

      if (!wasDetectedLastFrame)
        TLOGI(tlogger, "===> Visual ready signalled via debug request (t={})", theFrameInfo.time);
    }
    else if (canSeeReferee())
    {
      check();
      info.isDetected = detected;
      info.confidence = gestureConfidence;

      if (info.isDetected && !wasDetectedLastFrame)
        TLOGI(tlogger, "===> Visual ready signalled based on detection model outputs (conf={}, t={})",
              gestureConfidence, theFrameInfo.time);
    } 
    else // cannot see referee
    {
      // Checking for obstacles in the opponent half
      detectRobotsInOpponentHalf();
      if (opponentsInTheirHalfDetected && canProcceedOntoPitch())
      {
        TLOGI(tlogger, "===> Visual ready signalled based on opponent robots in their half (t={})", theFrameInfo.time);
        // If we have detected robots in the opponent half and no illegal motion in standby, signal ready
        info.isDetected = true;
        info.confidence = 0.99f;

        if (info.isDetected && !wasDetectedLastFrame)
          TLOGI(tlogger, "===> Visual ready signalled based on obstacles in opponent half (t={})", theFrameInfo.time);
      }
    }

    if (info.isDetected)
    {
      info.frameTimeLastDetection = theFrameInfo.time;
    }
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

bool ReadyGestureDetector::canProcceedOntoPitch()
{
  // Check if any opponent player (1–5) is penalized for illegal motion in standby or already detected
  if (opponentMotionInStandby || !opponentsInTheirHalfDetected)
  {
    return false;
  }
  else
  {
    for (int i = 1; i < 5; ++i)
    {
      if (theGameInfo.opponentTeam().getPlayer(i).isPenalized(PENALTY_SPL_ILLEGAL_MOTION_IN_STANDBY))
      {
        TLOGI(tlogger, "OPPOSITION PENALISED NOT WALKING OUT");
        opponentMotionInStandby = true;
        return false;
      }
    }
  }

  // If 15 seconds have passed since initialTime and no illegal motion detected, allow proceeding
  if ((theFrameInfo.getTimeSince(detectionStartTime) > 13000))
  {
    
    if (!walkingIn)
    {
      TLOGI(tlogger, "No penalised robots detected in standby for 10+3 seconds, proceeding onto pitch");
      walkingIn = true;
    }
    return true;
  }

  return false;
}

bool ReadyGestureDetector::detectRobotsInOpponentHalf()
{
  if (opponentsInTheirHalfDetected) 
    return true;

  int robotCount = 0;

  // Count obstacles that are in the opponent half (x > 0) and within lateral field bounds  
  for (const auto &obstacle : theObstacleModel.obstacles)
  {
    Vector2f globalPosition = theRobotPose * obstacle.center;
    if ((globalPosition.x() > 0 && globalPosition.x() < 3000) &&
        (globalPosition.y() < 2500 && globalPosition.y() > -2500))
    {
      ++robotCount;
    }
  }

  if (robotCount >= 1)
  {
    if (detectionStartTime == 0)
    {
      detectionStartTime = theFrameInfo.time;
      TLOGI(tlogger, "Starting 3 Second Timer for robot detection in opponent half at {}ms", detectionStartTime);
    }
      
    if (theFrameInfo.getTimeSince(detectionStartTime) >= 3000)
    {
      TLOGI(tlogger, "Consistently detected {} robots in opponent half for 3 seconds", robotCount);
      opponentsInTheirHalfDetected = true;
    }
  }
  else if (detectionStartTime != 0)
  {
    TLOGI(tlogger, "Zero robots detected, resetting timer (was at {}ms)",
          theFrameInfo.getTimeSince(detectionStartTime));
    detectionStartTime = 0;
  }

  return opponentsInTheirHalfDetected;
}