/**
 * @file TeammatesLocator.cpp
 *
 * Locate teammates based on communicated information and expected positions
 * according to behaviour
 *
 * @author Rudi Villing
 */

#include "TeammatesLocator.h"
#include "Tools/Math/Covariance.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Math/BHMath.h"

#include <algorithm>
#include <limits>


MAKE_MODULE(TeammatesLocator, modeling);

TeammatesLocator::TeammatesLocator()
{
}

void TeammatesLocator::update(TeammatesLocationModel& teammatesLocationModel)
{
  auto& locations = teammatesLocationModel.locations;

  // remember the previous locations so we can see if we should continue
  // to assume communicated or formation based locations
  std::array<TeammatesLocationModel::Location, Settings::numPlayerNumbers>  prevLocations;

  for (auto location : locations)
    prevLocations[Settings::toPlayerIndex(location.playerNumber)] = location;

  locations.clear();

  // update based on recently communicated data

  for (const Teammate &teammate : theTeamData.teammates)
  {
    const int teammateIndex = Settings::toPlayerIndex(teammate.number);
    TeammatesLocationModel::Location &prevLocation = prevLocations[teammateIndex];

    locations.push_back(prevLocation); // locations in the same order as teamData.teammates
    TeammatesLocationModel::Location &location = locations.back();

    // was teammate updated this frame (either via a team message being received or other update)
    if ((teammate.timeWhenLastUpdated == theFrameInfo.time) ||
        (teammate.timeWhenLastUpdated > location.timeWhenLocationCommunicated))
    {
      if (location.playerNumber == -1) // no old data for player, so add it
      {
        location.playerNumber = teammate.number;
        location.pose = teammate.theRobotPose;
      }

      // TODO - need to add support for communicated locations even when
      // not the ball player - may need to enhance the BehaviorStatus with extra info
      if (teammate.theTeamBehaviorStatus.role.isBallPlayer())
      {
        location.locationType = TeammatesLocationModel::independent;

        // FIXME include estimates based on walkingTo
        // no update needed because we're using teammate location but it hasn't been updated
      }
      else
        location.locationType = TeammatesLocationModel::formation;

      location.timeWhenLocationCommunicated = teammate.timeWhenLastUpdated;
      // {
      //   if (theActiveTactic.formationPoses.empty())
      //   {
      //     const SetupPoses::SetupPose &p = theSetupPoses.getPoseOfRobot(teammate.number);
      //     location.pose.translation = p.position;
      //     location.pose.rotation = (p.turnedTowards - p.position).angle();
      //   }
      //   else
      //     location.pose = theActiveTactic.formationPoses[teammateIndex];
      // }
    }
    else // teammate data is old, do we need to update predicted locations
    {
      ASSERT(location.playerNumber == teammate.number); // there must have been a previous location

      if (teammate.isPenalized)
      {
        location.locationType = TeammatesLocationModel::penalized;
        location.pose.translation = Vector2f(20000.f,20000.f);
      }
      else if ((theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT) &&
               ((theGameInfo.state == STATE_READY) || (theGameInfo.state == STATE_PLAYING)))
      {
        if (location.locationType == TeammatesLocationModel::formation)
        {
          location.pose = getEstimatedPose(location.pose, theActiveTactic.formationPoses[teammateIndex]);
          // location.pose = theActiveTactic.formationPoses[teammateIndex];
        }
        else
        {
          Vector2f estimatedPos = teammate.getEstimatedPosition(theFrameInfo.time);

          Vector2f walkingToOnField = teammate.theRobotPose.toWorld(teammate.theBehaviorStatus.walkingTo);
          Angle estimatedRotation =
              (walkingToOnField - estimatedPos).squaredNorm() > sqr(rotateToTargetDistance)
                  ? (walkingToOnField - estimatedPos).angle()                             // facing direction of walk
                  : (theFieldBall.recentBallEndPositionOnField() - estimatedPos).angle(); // facing ball

          location.pose = Pose2f(estimatedRotation, estimatedPos);
        }
      }
    }
  }

  prevFrameTime = theFrameInfo.time; // used for estimating distance travelled between frames

  // for simpler drawings look at representation:TeammatesLocationModel instead for the 3D field or 2D worldState (field) view
  draw(locations);
}

/**
 * @param prevPose - field coordinates
 * @param targetPose - field coordinates
 */
Pose2f TeammatesLocator::getEstimatedPose(const Pose2f& prevPose, const Pose2f& targetPose)
{
  const float secsSinceLastFrame = std::max(0, theFrameInfo.getTimeSince(prevFrameTime)) / 1000.f;
  const float distanceToTarget = targetPose.translation.norm(); // Do not overshoot the target.
  const float correctionFactor = 0.5f;
  const float speed = theWalkingEngineOutput.maxSpeed.translation.x(); // we guess full speed
  const Vector2f walkDirection =
      (targetPose.translation - prevPose.translation).normalized(); // unit vector in direction of travel
  Vector2f estimatedPos = prevPose.translation +
         std::min(distanceToTarget, secsSinceLastFrame * correctionFactor * speed) * walkDirection;

  Angle estimatedRotation = (targetPose.translation - estimatedPos).squaredNorm() > sqr(rotateToTargetDistance)
                                ? Angle((targetPose.translation - estimatedPos).angle()) // facing direction of walk
                                : targetPose.rotation;                                   // matching target pose

  return Pose2f(estimatedRotation, estimatedPos);
}

void TeammatesLocator::draw(const std::vector<TeammatesLocationModel::Location>& locations)
{
  DEBUG_DRAWING("module:TeammatesLocator", "drawingOnField")
  {
    ColorRGBA posColor = ColorRGBA::violet.darker(0.4f).alpha(0.4f);
    ColorRGBA fromColor = ColorRGBA::violet.lighter(0.25f).alpha(0.4f);

    for(const auto& location : locations)
    {
      CIRCLE("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(), 
             100, 20, Drawings::solidPen, posColor, Drawings::solidBrush, posColor);
      VERT_CROSS("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(), 
            20, 20, Drawings::solidPen, ColorRGBA::white);

      if (location.locationType == TeammatesLocationModel::independent)
        CIRCLE("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(),
               140, 30, Drawings::dottedPen, posColor, Drawings::noBrush, ColorRGBA::black);

      Vector2f directionVec = location.pose * Vector2f(200, 0);

      ARROW("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(), 
            directionVec.x(), directionVec.y(), 20, Drawings::dashedPen, posColor);

      // where is this teammate coming from and heading to?

      if (location.locationType == TeammatesLocationModel::independent)
      {
        if (theTeamData.teammatesByNumber[location.playerNumber])
        {
          Teammate& teammate = *theTeamData.teammatesByNumber[location.playerNumber];

          // coming from (last communicated pose)
          LINE("module:TeammatesLocator", teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y(),
               location.pose.translation.x(), location.pose.translation.y(), 10, Drawings::dashedPen, fromColor);

          CIRCLE("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(), 30, 0,
                 Drawings::dashedPen, fromColor, Drawings::noBrush, fromColor);

          // heading to
          LINE("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(),
               teammate.theBehaviorStatus.walkingTo.x(), teammate.theBehaviorStatus.walkingTo.y(), 10,
               Drawings::dashedPen, posColor);

          CIRCLE("module:TeammatesLocator", teammate.theBehaviorStatus.walkingTo.x(),
                 teammate.theBehaviorStatus.walkingTo.y(), 50, 0, Drawings::noPen, posColor, Drawings::solidBrush,
                 posColor);
        }
      }
      else // formation based
      {
        // no fixed "coming from" point since it is updated on every cycle

        const Vector2f &targetPos =
            theActiveTactic.formationPoses[Settings::toPlayerIndex(location.playerNumber)].translation;

        // heading to
        LINE("module:TeammatesLocator", location.pose.translation.x(), location.pose.translation.y(), targetPos.x(),
             targetPos.y(), 10, Drawings::dashedPen, posColor);

        CIRCLE("module:TeammatesLocator", targetPos.x(), targetPos.y(), 50, 0, Drawings::noPen, posColor,
               Drawings::solidBrush, posColor);
      }
    }
  }
}