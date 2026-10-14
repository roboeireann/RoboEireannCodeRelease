/**
 * @file DirectKickOff.cpp
 *
 * This ...
 *
 * @author Arne Hasselbring (from former KickoffStrikerCard.cpp in 2019).
 */

#include "SkillBehaviorControl.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"

namespace DirectKickOff
{
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace DirectKickOff;

option((SkillBehaviorControl) DirectKickOff,
       load((float) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
            (Angle) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
            (Angle) halfGoalSectorAngle, /**< Half of the goal opening angle around the positive x axis. */
            (Angle) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
            (float) ballDistanceForSlowWalk, /**< If the ball is close, walk slow. */
            (std::vector<KickInfo::KickType>) allowedKicks), /**< The kicks that may be used for the kickoff. */
       vars((bool)(false) wasActive, /**< Whether an in walk kick out of the center circle was already tried. */
            (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType, /**< The kick type to try. */
            (Angle)(0_deg) targetAngle)) /**< The target angle to which the kick-off should go (from the ball in field coordinates). */
{
  // Picks the allowed kick for the given leg, falling back to whatever is allowed if there is no kick for that leg
  // (e.g. on platforms like K1 that currently only have a single, right-foot-only kick).
  const auto findKickForLeg = [&](Legs::Leg leg) -> KickInfo::KickType
  {
    for(KickInfo::KickType kick : allowedKicks)
      if(theKickInfo[kick].kickLeg == leg)
        return kick;
    return allowedKicks.front();
  };
  const KickInfo::KickType rightKick = findKickForLeg(Legs::right);
  const KickInfo::KickType leftKick = findKickForLeg(Legs::left);

  initial_state(execute)
  {
    action
    {
      if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) < 2000)
      {
        LookLeftAndRight({.tilt = theBehaviorParameters.defaultLookDownAngle});
        Stand({.energySavingWalk = false});
      }
      else
      {
        // Prepare obstacle sectors.
        std::vector<ObstacleSector> obstacleSectors;
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
          const Vector2f obstacleOnField = theRobotPose * obstacle.center;
          if(obstacleOnField.x() < 0.f || obstacleOnField.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - 300.f))
            continue;

          const float width = (obstacle.left - obstacle.right).norm() + 4.f * theBallSpecification.radius;
          const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
          if(distance < theBallSpecification.radius)
            continue;

          const float radius = std::atan(width / (2.f * distance));
          const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();
          // Cull obstacles that are not in the goal sector anyway.
          if(direction - radius > halfGoalSectorAngle || direction + radius < -halfGoalSectorAngle)
            continue;
          obstacleSectors.emplace_back();
          obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
          obstacleSectors.back().distance = distance;
          obstacleSectors.back().x = obstacleOnField.x();
          // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
          if(wasActive && obstacleSectors.back().sector.isInside(targetAngle))
            obstacleSectors.back().x += hysteresisNumber;
        }
        // Obstacle to avoid passing straight forward and running after the ball with the same Robot
        if(!theIndirectKick.allowDirectKick)
        {
          obstacleSectors.emplace_back();
          obstacleSectors.back().sector = Rangea(-35_deg, 35_deg);
          obstacleSectors.back().distance = theFieldDimensions.centerCircleRadius;
          obstacleSectors.back().x = theFieldDimensions.centerCircleRadius;
        }

        // Sort obstacles according to their absolute x coordinate (to ease culling later on).
        std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

        SectorWheel wheel;
        std::list<SectorWheel::Sector> sectors;
        bool isLargeEnough = false;
        do
        {
          wheel.begin(theFieldBall.positionOnField);
          wheel.addSector(Rangea(-halfGoalSectorAngle, halfGoalSectorAngle), 2.f * std::max(theKickInfo[leftKick].range.max, theKickInfo[rightKick].range.max), SectorWheel::Sector::goal);
          for(const ObstacleSector& obstacleSector : obstacleSectors)
            wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
          sectors = wheel.finish();

          for(const SectorWheel::Sector& sector : sectors)
            if(sector.type == SectorWheel::Sector::goal &&
               sector.angleRange.getSize() >= ((wasActive && sector.angleRange.isInside(targetAngle)) ? Angle(minOpeningAngle * 0.5f) : minOpeningAngle))
            {
              isLargeEnough = true;
              break;
            }
        }
        while(!isLargeEnough && !obstacleSectors.empty() && (obstacleSectors.pop_back(), true));

        DRAW_SECTOR_WHEEL("option:DirectKickOff:wheel", sectors, theFieldBall.endPositionOnField);

        Angle bestOpeningAngle = 0_deg;
        Angle bestTargetAngle = 0_deg;
        for(const SectorWheel::Sector& sector : sectors)
        {
          if(sector.type != SectorWheel::Sector::goal)
            continue;
          const Angle openingAngle = sector.angleRange.getSize() + ((wasActive && sector.angleRange.isInside(targetAngle)) ? hysteresisAngle : 0_deg);
          if(openingAngle > bestOpeningAngle)
          {
            bestOpeningAngle = openingAngle;
            bestTargetAngle = sector.angleRange.getCenter();
          }
        }

        ASSERT(bestOpeningAngle > 0_deg);

        targetAngle = bestTargetAngle;
        if(targetAngle > (wasActive ? (kickType == rightKick ? -hysteresisAngle : hysteresisAngle) : 0_deg))
          kickType = rightKick;
        else
          kickType = leftKick;

        GoToBallAndKick({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                         .kickType = kickType,
                         .lookActiveWithBall = true,
                         .preStepType = PreStepType::notAllowed,
                         .reduceWalkSpeedType = theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange });
        wasActive = true;
      }
    }
  }
}
