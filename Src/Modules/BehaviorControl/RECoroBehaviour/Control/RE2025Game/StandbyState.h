/**
 * @file StandbyStateTask.h
 *
 * This task implements the standby state behaviour for RoboCup 2024.
 * Ultimately this involves the robots looking to where the referee is expected
 * to be and then attempting a walkout if the ref signal is detected.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"

#include "Math/Pose2f.h"

#include "TextLogging/TextLogging.h"


namespace RE
{
namespace v2025
{
  // this is the main entry point for the standby state and it delegates to the appropriate
  // specialized behaviour to do the real work

  CRBEHAVIOUR(StandbyStateTask)
  {
    CRBEHAVIOUR_INIT(StandbyStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();


      // TODO change this to repeat only until one (multiple?) robots claim
      // to have seen the gesture and then try a probe with one robot moving.
      // If it is not penalised, the others can go
      while (true)
      {
        headSkills.lookAtPoint(getBestViewPosition(), /* speed */ 180_deg, HeadMotionRequest::upperCamera);
        motionSkills.stand(/* high: */ true);

        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(FieldDimensions);
    READS(GameState);

    HeadSkills headSkills {env};
    MotionSkills motionSkills {env};
 

    Vector3f getBestViewPosition() 
    {
      // when playing left to right, robots on the right side of the field
      // are at the GC side and can see the ref
      bool canSeeReferee = theGameState.leftHandTeam ? (theRobotPose.translation.y() < 0) : (theRobotPose.translation.y() > 0);

      if (canSeeReferee)
      {
        int oppositeSide = theRobotPose.translation.y() > 0 ? -1 : 1;
        Vector2f refPointOnField = Vector2f(0, (theFieldDimensions.yPosLeftTouchline + 300) * oppositeSide);
        Vector2f refPoint = theRobotPose.toRobotCoordinates(refPointOnField);

        // return Vector3f(refPoint.x(), refPoint.y(), 523); // looking straight at camera height.
        return Vector3f(refPoint.x(), refPoint.y(), 1350); // looking approx the center of the height for a male with arms extended up.
        // return Vector3f(4000, 1000, 1200);
      }
      // else cannot see ref
      else
      { 
        int sameSide = theRobotPose.translation.y() > 0 ? 1 : -1;
        Vector2f sameSidePoint = Vector2f(theFieldDimensions.xPosOpponentPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea * sameSide);
        Vector2f sameSidePointRobot = theRobotPose.toRobotCoordinates(sameSidePoint);
        return Vector3f(sameSidePointRobot.x(), sameSidePointRobot.y(), 523);
      }
    }
  };


} // namespace v2025
} // namespace RE
