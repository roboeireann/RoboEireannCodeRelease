/**
 * @file BehaviorControl/FieldBallProvider/FieldBallProvider.h
 *
 * Provides the FieldBall for the Behavior
 *
 * @author Tim Laue
 * @author Rudi Villing
 * @author Nathan Ewnetu
 */

#include "FieldBallProvider.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Modeling/BallPhysics.h"

#include "Tools/TextLogging.h"
#include "Tools/FmtCommonTypes.h"


DECL_TLOGGER(tlogger, "FieldBallProvider", TextLogging::INFO);

MAKE_MODULE(FieldBallProvider, behaviorControl);

void FieldBallProvider::update(FieldBall& fieldBall)
{
  fieldBall.positionRelative        = theBallModel.estimate.position;
  fieldBall.positionOnField         = theRobotPose * fieldBall.positionRelative;
  fieldBall.positionOnFieldClipped  = fieldBall.positionOnField;
  theFieldDimensions.clipToField(fieldBall.positionOnFieldClipped);
  fieldBall.positionRelativeClipped = theRobotPose.inversePose * fieldBall.positionOnFieldClipped;
  fieldBall.endPositionRelative     = BallPhysics::getEndPosition(theBallModel.estimate.position, theBallModel.estimate.velocity, theBallSpecification.friction);
  fieldBall.endPositionOnField      = theRobotPose * fieldBall.endPositionRelative;

  fieldBall.teamPositionOnField     = theTeamBallModel.position;
  fieldBall.teamPositionRelative    = theRobotPose.inversePose * fieldBall.teamPositionOnField;
  fieldBall.teamEndPositionOnField  = BallPhysics::getEndPosition(theTeamBallModel.position, theTeamBallModel.velocity, theBallSpecification.friction);
  fieldBall.teamEndPositionRelative = theRobotPose.inversePose * fieldBall.teamEndPositionOnField;

  fieldBall.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
  fieldBall.timeSinceBallDisappeared = theFrameInfo.getTimeSince(theBallModel.timeWhenDisappeared);
  fieldBall.timeSinceTeamBallWasValid = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastValid);
  fieldBall.isTeamBallValid = theTeamBallModel.isValid;

  checkIfBallIsRollingTowardsAGoal(fieldBall);
  checkIfBallIsPassingOwnYAxis(fieldBall);
  checkIfBallIsPassingOwnXAxis(fieldBall);
  checkIfBallIsInsideOwnPenaltyArea(fieldBall);
  checkBallTimeToTarget(fieldBall);

}

void FieldBallProvider::checkIfBallIsRollingTowardsAGoal(FieldBall& fieldBall)
{
  fieldBall.isRollingTowardsOpponentGoal = false;
  fieldBall.isRollingTowardsOwnGoal = false;
  if(theBallModel.estimate.velocity.norm() == 0.f)
    return;
  const Vector2f bPosField = fieldBall.positionOnField;
  const Vector2f bVelField = theBallModel.estimate.velocity.rotated(theRobotPose.rotation);
  const Vector2f leftOppPost(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal + opponentGoalpostYOffset);
  const Vector2f rightOppPost(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal - opponentGoalpostYOffset);
  const Vector2f leftOwnPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal + ownGoalpostYOffset);
  const Vector2f rightOwnPost(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal - ownGoalpostYOffset);

  // Check, if ball is rolling TOWARDS (this is why we multiply the velocity) the expanded opponent goal:
  fieldBall.isRollingTowardsOpponentGoal = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOppPost, rightOppPost);
  fieldBall.isRollingTowardsOwnGoal      = Geometry::checkIntersectionOfLines(bPosField, bPosField + bVelField * 1000.f, leftOwnPost, rightOwnPost);
}

void FieldBallProvider::checkIfBallIsPassingOwnYAxis(FieldBall& fieldBall)
{
  fieldBall.intersectionPositionWithOwnYAxis = Vector2f::Zero();
  fieldBall.secsUntilIntersectsOwnYAxis = std::numeric_limits<float>::max();

  fieldBall.intersectionPositionWithOwnYAxisAggressive = Vector2f::Zero();
  fieldBall.secsUntilIntersectsOwnYAxisAggressive = std::numeric_limits<float>::max();

  const Vector2f& ballPosRel = theBallModel.estimate.position;
  const Vector2f& ballVelRel = theBallModel.estimate.velocity;
  if(ballVelRel.x() >= 0 || ballPosRel.x() < 0.f) // Ball does not move or moves away
    return;

  const float FIXME_HACK_FACTOR = 0.95f; // FIXME - it appears that the update of the predicted end position
                                    // gets out of sync with ball velocity such that predicted time to
                                    // reach end pos becomes huge (which doesn't make sense).
                                    // So we artifically shorten the distance we estimate time for 
  Vector2f intersection;
  if(Geometry::getIntersectionOfLines(Geometry::Line(ballPosRel, ballVelRel),
                                      Geometry::Line(Vector2f(0.f, 0.f), Vector2f(0.f, 1.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosRel).norm();
    Vector2f ballEndPosition = BallPhysics::getEndPosition(ballPosRel, ballVelRel, theBallSpecification.friction);
    const float distanceToEndPosition = (ballEndPosition - ballPosRel).norm();
    // Is the ball fast enough to reach the intersection point?
    if(distanceToIntersection < distanceToEndPosition)
    {
      fieldBall.intersectionPositionWithOwnYAxis = intersection;
      fieldBall.secsUntilIntersectsOwnYAxis = BallPhysics::timeForDistance(
          ballVelRel, distanceToIntersection * FIXME_HACK_FACTOR, theBallSpecification.friction);
      TLOGD(tlogger, "checkIfBallIsPassingOwnYAxis: distanceToIntersection < endPos");
    }
    else if ((distanceToEndPosition + aggressiveEndPositionDistance) > distanceToIntersection)
    {
      fieldBall.intersectionPositionWithOwnYAxisAggressive = intersection;

      float secsToEndPos = BallPhysics::timeForDistance(ballVelRel, distanceToEndPosition * FIXME_HACK_FACTOR,
                                                        theBallSpecification.friction);
      float secsEndPosToIntersection =
          (distanceToIntersection - distanceToEndPosition) /
          ballVelRel.norm(); // TODO hack (const velocity for diff between end point and intersection)
      fieldBall.secsUntilIntersectsOwnYAxisAggressive = secsToEndPos + secsEndPosToIntersection;

      TLOGD(
          tlogger,
          "checkIfBallIsPassingOwnYAxis: distToIntersect < endPos + aggr, distToEnd {}, ballVel {}, secToEnd {}, secExtra {}",
          distanceToEndPosition, ballVelRel, secsToEndPos, secsEndPosToIntersection);
    }
    else if (-ballVelRel.x() > aggressiveMinVelocityX)
    {
      fieldBall.intersectionPositionWithOwnYAxisAggressive = intersection;
      fieldBall.secsUntilIntersectsOwnYAxisAggressive =
          distanceToEndPosition / ballVelRel.norm(); // TODO: HACK - ignore friction in order to get an estimate
      TLOGD(tlogger, "checkIfBallIsPassingOwnYAxis: ballVel towards");
    }
    else
      TLOGD(tlogger, "checkIfBallIsPassingOwnYAxis: ball will intersect y, but conditions not satisfied");
  }
  else
    TLOGD(tlogger, "checkIfBallIsPassingOwnYAxis: ball does not intersect y");
}


void FieldBallProvider::checkIfBallIsPassingOwnXAxis(FieldBall& fieldBall)
{
  fieldBall.intersectionPositionWithOwnXAxis = Vector2f(200000.f, 200000.f); // larger than any field size (even future)
  fieldBall.secsUntilIntersectsOwnXAxis = std::numeric_limits<float>::max();

  const Vector2f& ballPosRel = theBallModel.estimate.position;
  const Vector2f& ballVelRel = theBallModel.estimate.velocity;

  // ball off to one side or the other and rolling further to the same side?
  if ((ballPosRel.y() > 0.f && ballVelRel.y() > 0.f) || (ballPosRel.y() < 0.f && ballVelRel.y() < 0.f))
    return; // ball won't intersect x

  Vector2f intersection;
  if (Geometry::getIntersectionOfLines(Geometry::Line(ballPosRel, ballVelRel),
                                       Geometry::Line(Vector2f(0.f, 0.f), Vector2f(1.f, 0.f)), intersection))
  {
    const float distanceToIntersection = (intersection - ballPosRel).norm();
    Vector2f ballEndPosition = BallPhysics::getEndPosition(ballPosRel, ballVelRel, theBallSpecification.friction);
    const float distanceToEndPosition = (ballEndPosition - ballPosRel).norm();

    // Is the ball fast enough to reach the intersection point?
    if (distanceToIntersection < distanceToEndPosition)
    {
      fieldBall.intersectionPositionWithOwnXAxis = intersection;
      fieldBall.secsUntilIntersectsOwnXAxis =
          BallPhysics::timeForDistance(ballVelRel, distanceToIntersection, theBallSpecification.friction);
    }
  }
}



void FieldBallProvider::checkIfBallIsInsideOwnPenaltyArea(FieldBall& fieldBall)
{
  fieldBall.isInsideOwnPenaltyArea = false;
  fieldBall.distanceToOwnPenaltyArea = -1.f;
  if(!fieldBall.ballWasSeen() && !theTeamBallModel.isValid)
    return;
  const Vector2f& bp = fieldBall.recentBallPositionOnField();
  fieldBall.isInsideOwnPenaltyArea = bp.x() < theFieldDimensions.xPosOwnPenaltyArea &&
                                     std::abs(bp.y()) < theFieldDimensions.yPosLeftPenaltyArea;
  if(fieldBall.isInsideOwnPenaltyArea)
  {
    fieldBall.distanceToOwnPenaltyArea = 0.f;
  }
  else
  {
    float dx = std::max(0.f, bp.x() - theFieldDimensions.xPosOwnPenaltyArea);
    float dy = std::max(0.f, std::abs(bp.y()) - theFieldDimensions.yPosLeftPenaltyArea);
    if(dx > 0.f && dy == 0.f) // ball in front of penalty area
      fieldBall.distanceToOwnPenaltyArea = dx;
    else if(dx == 0.f && dy >= 0.f) // ball is next to penalty area
      fieldBall.distanceToOwnPenaltyArea = dy;
    else if(bp.y() > 0.f) // ball is left front of penalty area
      fieldBall.distanceToOwnPenaltyArea = (Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftPenaltyArea) - bp).norm();
    else // ball is right front of penalty area
      fieldBall.distanceToOwnPenaltyArea = (Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea) - bp).norm();
  }
}


/**
 * Check the time it will take for the ball to reach the target location
 */
void FieldBallProvider::checkBallTimeToTarget(FieldBall& fieldBall)
{
  const Vector2f& ballPosRel = theBallModel.estimate.position;
  const Vector2f& ballVelRel = theBallModel.estimate.velocity;
  const unsigned currentTime = theFrameInfo.time; // Get the current frame time

  // Angle rHipPitch;
  // Angle prevRHipPitch = 0.f;

  // For verifying when max extent was in ReplayRobot

//   rHipPitch = theJointAngles.angles[Joints::rHipPitch];
//   const Angle targetPitch = -50_deg;
//   const Angle threshold = 0.5_deg;
// 
//   if (theMotionInfo.isWalkPhaseInWalkKick)
//   {
//     // Detect the transition through the target value (more reliable)
//     if (((prevRHipPitch > (targetPitch - threshold)) && (rHipPitch < (targetPitch - threshold))) ||
//         (std::abs(rHipPitch - targetPitch) < threshold))
//       OUTPUT_TEXT("[" << currentTime << "] Strike Point Detected at hip angle: " << rHipPitch);
//   }
//   prevRHipPitch = rHipPitch;

  if (fieldBall.ballWasSeen())
  {
    OUTPUT_TEXT("[" << currentTime << "] Ball predicted position  x:" << ballPosRel.x());
    OUTPUT_TEXT("Sim Ball velocity y:" << ballVelRel.y());
  }


  //If robot is moving, check the robotPose.rotation
  if (theMotionInfo.executedPhase == MotionPhase::walk)
  {
    //OUTPUT_TEXT("[" << currentTime << "] Robot Pose Rotation: " << theRobotPose.rotation);
  }
} 
