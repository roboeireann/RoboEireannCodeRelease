/**
 * @file SetPlayGestureDetector.h
 *
 * Module that detects the referee set play gesture from the camera image
 *
 * @author Rudi Villing
 * @author Conal Hughes
 */

#include "SetPlayGestureDetector.h"

#include "Tools/Debugging/Annotation.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Range.h"
#include "Tools/ImageProcessing/ColorModelConversions.h"
#include "Tools/TextLogging.h"


DECL_TLOGGER(tlogger, "SetPlayGestureDetector", TextLogging::INFO);

MAKE_MODULE(SetPlayGestureDetector, perception);

SetPlayGestureDetector::SetPlayGestureDetector()
{
  model.reset(new SetPlayGestureDetectionModel(modelName, labelsName, isEnabled));

  /* Set default values */
  gestureConfidence = 0.f;
  detected = false;

  previousGameState = STATE_INITIAL;
  timeStartOfStandby = 0;
}

void SetPlayGestureDetector::update(SetPlayRefereeGesture& gestureInfo)
{
  DECLARE_PLOT("module:SetPlayGestureDetector:gestureConfidence");
  DECLARE_PLOT("module:SetPlayGestureDetector:gestureConfidenceThreshold");
  DECLARE_PLOT("module:SetPlayGestureDetector:detectedGesture");
  DECLARE_DEBUG_DRAWING("module:SetPlayGestureDetector:cropBox", "drawingOnImage");


  // TODO: not sure if this will be too useful for SetPlayGestures
  // bool debugReadySignalled = false;
  // DEBUG_RESPONSE_ONCE("module:SetPlayGestureDetector:signalReady")
  // {
  //   debugReadySignalled = true;
  // }

  // SetPlay gestures are only signalled in the PLAYING state
  // The rules state that the gesture should be held for "at least 5 seconds after the GameController signal is sent"
  // isEnabled allows this detector to be turned on/off via configuration 

  // if ((isEnabled) && (theGameInfo.state == STATE_PLAYING) && (theGameInfo.setPlay != SET_PLAY_NONE) &&
  //     (startInferenceOffsetMs <= theExtendedGameInfo.timeSinceFreeKickStarted) &&
  //     (theExtendedGameInfo.timeSinceFreeKickStarted <= endInferenceOffsetMs) //&& canSeeReferee()
  //     && !theTeamBehaviorStatus.role.isGoalkeeper()
  //     // /* Comment out for SimRobot*/ && (theGameInfo.isNoTeamKick()) /*Comment out for SimRobot*/
  //   )

  if (!isEnabled || (theGameInfo.state != STATE_PLAYING) || (theGameInfo.setPlay == SET_PLAY_NONE))
    return; // nothing to do here

  SetPlayRefereeGesture::Gesture prevDetectedGesture = gestureInfo.detectedGesture;

  if (theGameInfo.isNoTeamKick())
  {
    // try to infer kicking team using ball position

    if ((theGameInfo.state == STATE_PLAYING) && (theGameInfo.setPlay == SET_PLAY_CORNER_KICK)) // Corner kick
    {
      bool ballTimeout = theFieldBall.timeSinceBallWasSeen > ballSeenTimeout;
      bool ballInOpponentHalf = theFieldBall.recentBallEndPositionOnField().x() > 1000.f;

      // Handle no team kick case
      if (theGameInfo.isPlayingLeftToRight())
      {
        gestureInfo.detectedGesture = (ballInOpponentHalf && ballTimeout) ? SetPlayRefereeGesture::TOWARDS_RIGHT
                                                                          : SetPlayRefereeGesture::TOWARDS_LEFT;

        TLOGI(tlogger, "Implied gesture (from ball pos): {}", gestureInfo.detectedGesture);
      }
      else // playing right to left
      {
        gestureInfo.detectedGesture = (ballInOpponentHalf && ballTimeout) ? SetPlayRefereeGesture::TOWARDS_LEFT
                                                                          : SetPlayRefereeGesture::TOWARDS_RIGHT;

        TLOGI(tlogger, "Implied gesture (from ball pos): {}", gestureInfo.detectedGesture);
      }

      gestureInfo.confidence = 0.9f; // reset confidence
    }
    else if (theGameInfo.state == STATE_PLAYING && theGameInfo.setPlay == SET_PLAY_GOAL_KICK) // goal kick
    {
      bool ballTimeout = theFieldBall.timeSinceBallWasSeen > ballSeenTimeout;
      bool ballInOwnHalf = theFieldBall.recentBallEndPositionOnField().x() < 1000.f;

      if (theGameInfo.isPlayingLeftToRight())
      {
        gestureInfo.detectedGesture =
            (ballInOwnHalf && ballTimeout) ? SetPlayRefereeGesture::TOWARDS_RIGHT : SetPlayRefereeGesture::TOWARDS_LEFT;
      }
      else // right to left
      {
        gestureInfo.detectedGesture =
            (ballInOwnHalf && ballTimeout) ? SetPlayRefereeGesture::TOWARDS_LEFT : SetPlayRefereeGesture::TOWARDS_RIGHT;
      }

      gestureInfo.confidence = 0.9f; // reset confidence
    }
    else if ((startInferenceOffsetMs <= theExtendedGameInfo.timeSinceFreeKickStarted) &&
             (theExtendedGameInfo.timeSinceFreeKickStarted <= endInferenceOffsetMs)
             // && !theTeamBehaviorStatus.role.isGoalkeeper() // goalkeepers do not see the referee - FIXME prevents block running?????
             && canSeeReferee())
    {
      // if (debugReadySignalled)
      // {
      //   info.isDetected = true;
      //   info.confidence = 0.9f;
      // }
      // else
      {
        checkInference(gestureInfo);
      }
    }
  }
  else
  {
    // gestureInfo.isNewResult = false;
    gestureInfo.detectedGesture = SetPlayRefereeGesture::NONE;
    gestureInfo.confidence = 0.f;
    // keep same last detected frame time
  }

  if (gestureInfo.detectedGesture != SetPlayRefereeGesture::NONE) // was something detected?
  {
    gestureInfo.lastDetection = gestureInfo.detectedGesture;
    gestureInfo.frameTimeLastDetection = theFrameInfo.time;

    if (gestureInfo.detectedGesture != prevDetectedGesture)
      TLOGI(tlogger, "New gesture detected {} (t={})", gestureInfo.detectedGesture,//TypeRegistry::getEnumName(gestureInfo.detectedGesture),
            theFrameInfo.time);
  }

}

void SetPlayGestureDetector::checkInference(SetPlayRefereeGesture& gestureInfo)
{
    TLOGV(tlogger, "checkInference");

    // Get the most recent model outputs (from a previous request).
    // We do this first, because this also advances the model state machine
    // if a new output was available.

    // TODO the following arguments may need to be adapted based on the type of output the model produces
    SetPlayRefereeGesture::Gesture detectedGesture;
    float detectionConfidence;

    if (model->getOutputs(detectedGesture, detectionConfidence))
    {
      gestureInfo.detectedGesture = detectedGesture;
      gestureInfo.confidence = detectionConfidence;

      TLOGV(tlogger, "detectedGesture: {}, confidence: {}", gestureInfo.detectedGesture, gestureInfo.confidence);
    }
    

    // try to set a new request
    model->trySetInputImage(theCameraImage, theRobotPose, theFieldDimensions, cropOffsetX, cropOffsetY, theECImage);
    // gestureInfo.detectedGesture = SetPlayRefereeGesture::TOWARDS_LEFT;
    // gestureInfo.confidence = 0.7f;
    // return;
    PLOT("module:SetPlayGestureDetector:gestureConfidence", gestureConfidence);
    PLOT("module:SetPlayGestureDetector:gestureConfidenceThreshold", gestureConfidenceThreshold);
    PLOT("module:SetPlayGestureDetector:detectedGesture", static_cast<int>(gestureInfo.detectedGesture));
}

bool SetPlayGestureDetector::canSeeReferee()
{
  // TODO - this should be based on position of robot on field relative
  // to referee position and whether robot head is looking in the right
  // place or not.
  Vector2f expectedRefPointOnField = Vector2f(0, theFieldDimensions.yPosLeftSideline + 300);
  float distanceToReferee = (theRobotPose.translation - expectedRefPointOnField).norm();

  if (distanceToReferee < 1500.f)
  {
    // Robot is too close to the referee to see them
    return false;
  }
  Pose2f refPointOnField;
  if(theGameInfo.isPlayingLeftToRight())
    refPointOnField = Pose2f(pi_2, Vector2f(0, theFieldDimensions.yPosLeftSideline + 300));
  else
    refPointOnField = Pose2f(-pi_2, Vector2f(0, theFieldDimensions.yPosRightSideline + 300));

  Pose2f refPoint = theRobotPose.toRobotCoordinates(Pose2f(pi_2, Vector2f(0, theFieldDimensions.yPosLeftSideline + 300)));
  float angleThreshold = 75_deg; // angle threshold to consider the referee visible
  return abs(refPoint.rotation) < angleThreshold;

  // Take a look at BallLocatorTools::ballShouldBeVisibleInCurrentImage to see
  // how to check if a particular object size should be visible based on where
  // the camera is currently pointing

  return true;
}