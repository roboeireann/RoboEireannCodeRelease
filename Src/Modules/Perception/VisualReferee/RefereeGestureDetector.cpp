/**
 * @file RefereeGestureDetector.cpp
 *
 * Implementation of the module that detects referee gestures from the camera image
 *
 * @author Rudi Villing
 */

#include "RefereeGestureDetector.h"

#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/Range.h"

MAKE_MODULE(RefereeGestureDetector, perception);

using namespace DetectedGesture;

RefereeGestureDetector::RefereeGestureDetector() {}

void RefereeGestureDetector::update(RefereeGesture &refereeGesture)
{
  GestureOneHot detectedGesture;  
  if (gestureDetectorModel.tryGetInferenceOutput(detectedGesture))
  {
    detectedGestures.push_front(detectedGesture);

    if (detectedGestures.size() == detectedGestures.capacity())
    {
      // what is the most frequent detection during the last temporal window?
      GestureOneHot sum = {0};
      for (unsigned i=0; i<detectedGestures.size(); i++)
      {
        const GestureOneHot &entry = detectedGestures[i];
        for (unsigned j=0; j<entry.size(); j++)
          sum[j] += detectedGestures[i][j];
      }

      // which gesture has the largest sum
      int best = 0;
      for (unsigned i=0; i<sum.size(); i++)
        if (sum[i] > sum[best])
          best = i;

      // convert from one-hot to gesture and team

      switch (best)
      {
      case DetectedGesture::CORNER_BLUE:
        refereeGesture.gesture = RefereeGesture::CORNER_KICK;
        refereeGesture.team = RefereeGesture::BLUE_TEAM;
        break;
      case DetectedGesture::CORNER_RED:
        refereeGesture.gesture = RefereeGesture::CORNER_KICK;
        refereeGesture.team = RefereeGesture::RED_TEAM;
        break;
      case DetectedGesture::FULL_TIME:
        refereeGesture.gesture = RefereeGesture::FULL_TIME;
        refereeGesture.team = RefereeGesture::NO_TEAM;
        break;
      case DetectedGesture::GOAL_KICK_BLUE:
        refereeGesture.gesture = RefereeGesture::GOAL_KICK;
        refereeGesture.team = RefereeGesture::BLUE_TEAM;
        break;
      case DetectedGesture::GOAL_KICK_RED:
        refereeGesture.gesture = RefereeGesture::GOAL_KICK;
        refereeGesture.team = RefereeGesture::RED_TEAM;
        break;
      case DetectedGesture::GOAL_BLUE:
        refereeGesture.gesture = RefereeGesture::GOAL;
        refereeGesture.team = RefereeGesture::BLUE_TEAM;
        break;
      case DetectedGesture::GOAL_RED:
        refereeGesture.gesture = RefereeGesture::GOAL;
        refereeGesture.team = RefereeGesture::RED_TEAM;
        break;
      case DetectedGesture::KICK_IN_BLUE:
        refereeGesture.gesture = RefereeGesture::KICK_IN;
        refereeGesture.team = RefereeGesture::BLUE_TEAM;
        break;
      case DetectedGesture::KICK_IN_RED:
        refereeGesture.gesture = RefereeGesture::KICK_IN;
        refereeGesture.team = RefereeGesture::RED_TEAM;
        break;
      case DetectedGesture::PUSHING_BLUE:
        refereeGesture.gesture = RefereeGesture::PUSHING_FREE_KICK;
        refereeGesture.team = RefereeGesture::BLUE_TEAM;
        break;
      case DetectedGesture::PUSHING_RED:
        refereeGesture.gesture = RefereeGesture::PUSHING_FREE_KICK;
        refereeGesture.team = RefereeGesture::RED_TEAM;
        break;
      default:
      case DetectedGesture::UNRECOGNISED:
        refereeGesture.gesture = RefereeGesture::NONE;
        refereeGesture.team = RefereeGesture::NO_TEAM;
        break;
      }

      refereeGesture.lastDetectionTime = theFrameInfo.time;
    }
  }

  gestureDetectorModel.tryStartInference(theCameraImage);


  ModifiableGesture modifiableGesture;
  modifiableGesture.gesture = RefereeGesture::GOAL;
  modifiableGesture.team = RefereeGesture::BLUE_TEAM;

  MODIFY("module:RefereeGestureDetector:gesture", modifiableGesture);

  DEBUG_RESPONSE_ONCE("module:RefereeGestureDetector:detectNow")
  {
    // restrict to legal gesture and team combinations
    if ((modifiableGesture.team != RefereeGesture::NO_TEAM) ||
        ((modifiableGesture.gesture == RefereeGesture::NONE) ||
         (modifiableGesture.gesture == RefereeGesture::FULL_TIME)))
    {
      refereeGesture.gesture = modifiableGesture.gesture;
      refereeGesture.team = modifiableGesture.team;
      refereeGesture.lastDetectionTime = theFrameInfo.time;
    }
  }

  DEBUG_DRAWING3D("module:RefereeGestureDetector:referee", "field")
  {
    // indicate which colours are on which halves of the field

    Vector3f p1(200, 3000, 500), p2(200, 3000, 1000), p3(-200, 3000, 750), p4(-200, 3000, 750);
    Vector3f blue(-1500, 0, 0), red(1500, 0, 0);
    QUAD3D("module:RefereeGestureDetector:referee", p1 + blue, p2 + blue, p3 + blue, p4 + blue, ColorRGBA::blue);
    QUAD3D("module:RefereeGestureDetector:referee", p4 + blue, p3 + blue, p2 + blue, p1 + blue, ColorRGBA::blue);

    p1.x() *= -1;
    p2.x() *= -1;
    p3.x() *= -1;
    p4.x() *= -1;

    QUAD3D("module:RefereeGestureDetector:referee", p1 + red, p2 + red, p3 + red, p4 + red, ColorRGBA::red);
    QUAD3D("module:RefereeGestureDetector:referee", p4 + red, p3 + red, p2 + red, p1 + red, ColorRGBA::red);

    // draw the stick figure referee

    // legs
    LINE3D("module:RefereeGestureDetector:referee", -150, 3000, 0, -150, 3000, 750, 40, ColorRGBA::black);
    LINE3D("module:RefereeGestureDetector:referee", 150, 3000, 0, 150, 3000, 750, 40, ColorRGBA::black);

    LINE3D("module:RefereeGestureDetector:referee", -150, 3000, 750, 0, 3000, 800, 40, ColorRGBA::black);
    LINE3D("module:RefereeGestureDetector:referee", 150, 3000, 750, 0, 3000, 800, 40, ColorRGBA::black);

    // torso + head
    LINE3D("module:RefereeGestureDetector:referee", 0, 3000, 800, 0, 3000, 1700, 40, ColorRGBA::black);

    // shoulders
    LINE3D("module:RefereeGestureDetector:referee", -220, 3000, 1350, 220, 3000, 1350, 40, ColorRGBA::black);

    // approx arms (based on current gesture)
    // gestures default to left arm (+ve x side, red team side)

    // Upper arm 300mm, forearm 300mm, hand 150mm, so we treat total arm incl 
    // hand as 750mm and forearm incl hand as 450mm

    Vector3f shoulder(220, 3000, 1350);
    Vector3f handDown(220, 3000, 1350 - 750);                   // straight down
    Vector3f handSide(220 + 750, 3000, 1350);                   // straight out to side
    Vector3f handSideUp(220 + 530, 3000, 1350 + 530);           // 45 deg up
    Vector3f handSideDown(220 + 530, 3000, 1350 - 530);         // 45 deg down
    Vector3f handFrontPoint(220 - 100, 3000 - 600, 1350 - 375); // point to center (very approx)

    // move requiring elbows...
    Vector3f elbowDown(220, 3000, 1350 - 300);       // straight down
    Vector3f forearmUp(150, 3000 - 150, 1350 + 150); // up overlapping body

    Vector3f elbowSideDown(220 + 210, 3000, 1350 - 210); // down and to side
    float forearmLen = 450.f;
    Vector3f forearmForwardOrigin(0, -450, 0); // point forward from elbow


    if (refereeGesture.team != RefereeGesture::NO_TEAM)
    {
      float side = (refereeGesture.team == RefereeGesture::RED_TEAM) ? 1.0f : -1.0f;

      if (refereeGesture.gesture == RefereeGesture::KICK_IN)
      {
        LINE3D("module:RefereeGestureDetector:referee", shoulder.x() * side, shoulder.y(), shoulder.z(),
               handSide.x() * side, handSide.y(), handSide.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -shoulder.x() * side, shoulder.y(), shoulder.z(),
               -handDown.x() * side, handDown.y(), handDown.z(), 40, ColorRGBA::black);
      }
      else if (refereeGesture.gesture == RefereeGesture::GOAL_KICK)
      {
        LINE3D("module:RefereeGestureDetector:referee", shoulder.x() * side, shoulder.y(), shoulder.z(),
               handSideUp.x() * side, handSideUp.y(), handSideUp.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -shoulder.x() * side, shoulder.y(), shoulder.z(),
               -handDown.x() * side, handDown.y(), handDown.z(), 40, ColorRGBA::black);
      }
      else if (refereeGesture.gesture == RefereeGesture::CORNER_KICK)
      {
        LINE3D("module:RefereeGestureDetector:referee", shoulder.x() * side, shoulder.y(), shoulder.z(),
               handSideDown.x() * side, handSideDown.y(), handSideDown.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -shoulder.x() * side, shoulder.y(), shoulder.z(),
               -handDown.x() * side, handDown.y(), handDown.z(), 40, ColorRGBA::black);
      }
      else if (refereeGesture.gesture == RefereeGesture::GOAL)
      {
        LINE3D("module:RefereeGestureDetector:referee", shoulder.x() * side, shoulder.y(), shoulder.z(),
               handSide.x() * side, handSide.y(), handSide.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -shoulder.x() * side, shoulder.y(), shoulder.z(),
               -handFrontPoint.x() * side, handFrontPoint.y(), handFrontPoint.z(), 40, ColorRGBA::black);
      }
      else if (refereeGesture.gesture == RefereeGesture::PUSHING_FREE_KICK)
      {
        LINE3D("module:RefereeGestureDetector:referee", shoulder.x() * side, shoulder.y(), shoulder.z(),
               handSide.x() * side, handSide.y(), handSide.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -shoulder.x() * side, shoulder.y(), shoulder.z(),
               -elbowDown.x() * side, elbowDown.y(), elbowDown.z(), 40, ColorRGBA::black);
        LINE3D("module:RefereeGestureDetector:referee", -elbowDown.x() * side, elbowDown.y(), elbowDown.z(),
               -forearmUp.x() * side, forearmUp.y(), forearmUp.z(), 40, ColorRGBA::black);
      }
    }
    else if (refereeGesture.gesture == RefereeGesture::NONE)
    {
      LINE3D("module:RefereeGestureDetector:referee", shoulder.x(), shoulder.y(), shoulder.z(),
              handDown.x(), handDown.y(), handDown.z(), 40, ColorRGBA::black);
      LINE3D("module:RefereeGestureDetector:referee", -shoulder.x(), shoulder.y(), shoulder.z(),
              -handDown.x(), handDown.y(), handDown.z(), 40, ColorRGBA::black);
    }
    else if (refereeGesture.gesture == RefereeGesture::FULL_TIME)
    {
      if (refereeGesture.lastDetectionTime == theFrameInfo.time) // just detected this frame?
      {
        fullTimeForearmPhase = 0.f;
        fullTimePhaseIncrement =
            1.f / 30.f; // assume we're running at 30 frames/sec, so phase should change by 1/30 each frame
      }

      fullTimeForearmPhase += fullTimePhaseIncrement;
      if (fullTimeForearmPhase > 1.f)
      {
        fullTimeForearmPhase = 1.f;
        fullTimePhaseIncrement *= -1;
      }
      else if (fullTimeForearmPhase < 0.f)
      {
        fullTimeForearmPhase = 0.f;
        fullTimePhaseIncrement *= -1;
      }

      float a = Rangef(-60_deg, 60_deg).scale(fullTimeForearmPhase, Rangef::ZeroOneRange());
      float c = std::cos(a);
      float s = std::sin(a);

      // left
      LINE3D("module:RefereeGestureDetector:referee", shoulder.x(), shoulder.y(), shoulder.z(), elbowSideDown.x(),
             elbowSideDown.y(), elbowSideDown.z(), 40, ColorRGBA::black);
      LINE3D("module:RefereeGestureDetector:referee", elbowSideDown.x(), elbowSideDown.y(), elbowSideDown.z(),
             (elbowSideDown.x() + forearmLen * s), (elbowSideDown.y() - forearmLen * c), elbowSideDown.z(), 40,
             ColorRGBA::black);

      // right
      LINE3D("module:RefereeGestureDetector:referee", -shoulder.x(), shoulder.y(), shoulder.z(), -elbowSideDown.x(),
             elbowSideDown.y(), elbowSideDown.z(), 40, ColorRGBA::black);
      LINE3D("module:RefereeGestureDetector:referee", -elbowSideDown.x(), elbowSideDown.y(), elbowSideDown.z(),
             -(elbowSideDown.x() + forearmLen * s), (elbowSideDown.y() - forearmLen * c), elbowSideDown.z(), 40,
             ColorRGBA::black);
    }
  }
}
