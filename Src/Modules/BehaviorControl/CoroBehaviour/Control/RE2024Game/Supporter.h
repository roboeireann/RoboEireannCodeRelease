/**
 * @file Supporter.h
 *
 * This task implements the behaviour for supporter robots (defender,
 * supporter, forward).
 * 
 * It has been adapted from the RoboEireann 2019 behaviour 
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 * @author James Petri
 * @author Andy Lee Mitchell
 * @author Aidan Colgan
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2024/CoroBehaviour2024.h"


#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/ObstacleSkills.h"

#include "Tools/Hysteresis.h"



namespace CoroBehaviour
{
namespace RE2024
{
  CRBEHAVIOUR(GotoTacticPoseTask)
  {
    CRBEHAVIOUR_INIT(GotoTacticPoseTask) {}

    void operator()(void)
    {
      updateTacticPose();

      CRBEHAVIOUR_LOOP()
      {
        CR_CHECKPOINT(walk_to_tactic_pose);
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          lookLeftAndRightTask();
          walkToPoseAutoAvoidanceTask(tacticPose, Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      true, params.distanceThreshold, params.angleThreshold);
          CR_YIELD();
        }

        CR_CHECKPOINT(stand_at_tactic_pose);
        do
        {
          doHeadSkill();
          commonSkills.stand();
          CR_YIELD();
        }
        while (!outOfPosition());
      }
    }

  private:
    TextLogger tlogger = TextLogging::get("GoToTacticPose", TextLogging::VERBOSE);

    DEFINES_PARAMS(GotoTacticPoseTask,
    {,
      (float)(1.f) walkSpeed,
      (float)(1200.f) lookAtBallDistance, // mm - anything less than this and we'll look more directly at the ball rather than scanning
      (float)(100.f) distanceThreshold, // mm
      (float)(5_deg) angleThreshold,
      (float)(150.f) outOfPositionDistance, // mm
      (float)(10_deg) outOfPositionAngle,
      (int)(1000) ballLastSeenTimeoutLookAround, // [ms] if looking directly at a close ball model, look around a bit after this time
    });


    READS(RobotPose);
    READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(ActiveTactic);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    LookLeftAndRightTask lookLeftAndRightTask {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Pose2f tacticPose; // robot relative


    void updateTacticPose()
    {
      tacticPose = theRobotPose.toRobotCoordinates(theActiveTactic.formationPose);
    }

    bool outOfPosition()
    {
      return !commonSkills.isPoseClose(tacticPose, params.outOfPositionDistance, params.outOfPositionAngle);
    }

    void doHeadSkill()
    {
      bool needsLookAround = theFieldBall.timeSinceBallWasSeen < params.ballLastSeenTimeoutLookAround;

      if (!needsLookAround && (theFieldBall.positionRelative.squaredNorm() < sqr(params.lookAtBallDistance)))
        headSkills.lookAtBall(/* mirrored: */ false, /* forceOwnEstimate: */ true);
      else
        headSkills.lookActive(/* withBall: */ true);
    }
  };

  CRBEHAVIOUR(SupporterKickInTacticsTask) 
  {
    CRBEHAVIOUR_INIT(SupporterKickInTacticsTask) {}

    void operator()(void) 
    {
      CRBEHAVIOUR_LOOP() 
      { 
        CR_CHECKPOINT(move_out_of_exclusion_zone);
        // Ensure the robot is not in the exlcusion zone
        if (theFieldBall.ballWasSeen()) 
        {
          while ((theFieldBall.positionRelative.norm() <= params.exclusionRadius + params.exclusionThreshold))
          {
            doHeadSkill();
            walkToPoseAutoAvoidanceTask(getOutExclZone(), Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      /* keepTargetRotation */ true, params.distanceThreshold, params.angleThreshold);
            CR_YIELD();
          }
        }

        CR_CHECKPOINT(walk_to_tactic_pose);
        // Move to the tactic position
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          doHeadSkill();
          walkToPoseAutoAvoidanceTask(getTacticPose(), Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed),
                                      /* keepTargetRotation */ true, params.distanceThreshold, params.angleThreshold);
          CR_YIELD();
        }

        // Wait for the kick in to happen while ensuring the position is right
        CR_CHECKPOINT(stand_at_tactic_pose);
        do
        {
          doHeadSkill();
          commonSkills.stand();
          CR_YIELD();
        }
        while (commonSkills.isPoseClose(getTacticPose(), 400.f, 10_deg));
      }
    }

  private:
    DEFINES_PARAMS(SupporterKickInTacticsTask,
    {,
      (float)(0.3f)   defenderPositionRatio,   ///< Used to determine the position between the ball and the goal for the back blocker
      (float)(0.5f)   sideBlockerPositionRatio,///< Used to determine the position between the ball and the goal y-positions for the side blocker
      (float)(0.9f)   walkSpeed,               ///< The robot's walkspeed
      (float)(100.f)  distanceThreshold,       ///< The distance threshold for the positioning of the robot
      (float)(5_deg)  angleThreshold,          ///< The angle threshold for the positioning of the robot
      (float)(750.f)  exclusionRadius,         ///< Radius around the ball for a legal kick in
      (float)(100.f)  exclusionThreshold,      ///< A 10cm tolerance for the exclusion radius
      (unsigned)(2000)   ballSeenTimeout,
      (float)(3000.f) lookAtBallDistance,
    });

    READS(RobotPose);
    READS(GameInfo);
    READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(FieldDimensions);
    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    ObstacleSkills obstacleSkills {env};
    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};
    MotionSkills motionSkills {env};

    /// @brief Represents the supporter roles for the set play tactics. It is used to rerepresent the supporterIndex (defender, winger, winger)
    ENUM(Positioning,
    {,
      BACK_BLOCKER,
      SIDE_BLOCKER,
      FORWARD_BLOCKER,
      INTERCEPTOR, // only used when the goalie is striker??
    });
    
    // Determines manages the tactic based on the supporter role and kickin team
    Pose2f getTacticPose() 
    {
      int supporterIndex = theTeamBehaviorStatus.role.supporterIndex();
      supporterIndex = (supporterIndex == -1) ? INTERCEPTOR : supporterIndex;
      Pose2f tacticPose(0_deg, 0.0f, 0.0f);

      Vector2f tacticWeight = getTacticWeight(theGameInfo.setPlay, theFieldBall.ballWasSeen(params.ballSeenTimeout));

      if (supporterIndex == BACK_BLOCKER)
        tacticPose = getBackBlockerPosition(tacticWeight);
      else if (supporterIndex == SIDE_BLOCKER)
        tacticPose = getSideBlockerPosition(tacticWeight);
      else if (supporterIndex == FORWARD_BLOCKER)
        tacticPose = getForwardBlockerPosition(tacticWeight);
      else if (supporterIndex == INTERCEPTOR)
        tacticPose = getInterceptorPosition(tacticWeight);

      ACTGRAPH_FMT("tacticWeight1 = {}", tacticWeight);
      tacticWeight = (Vector2f(-4500.0f, 0.0f) + tacticWeight) * params.defenderPositionRatio;

      ACTGRAPH_FMT("tacticWeight = {}", tacticWeight);
      ACTGRAPH_FMT("kickInTacticRole = {}", getKickInTacticRole(supporterIndex));
      ACTGRAPH_FMT("kickInTacticPose = {}", tacticPose);
      
      return tacticPose;
    }
    
    Vector2f getTacticWeight(int setPlay, bool ballSeen) {
      if (setPlay == SET_PLAY_GOAL_KICK && commonSkills.isOurTeamKick())
        return Vector2f(0.0f, -1000.0f);
      else if (ballSeen)
        return theFieldBall.recentBallEndPositionOnField();
      else
        return Vector2f(0.0f, 0.0f);
    }

    const char* getKickInTacticRole(int index) 
    {  
      switch(index) 
      {
        case BACK_BLOCKER: return "Back Blocker";
        case SIDE_BLOCKER: return "Side Blocker";
        case FORWARD_BLOCKER: return "Front Blocker";
        case INTERCEPTOR: return "Interceptor";
        default: return "";
      }
    }

    /// @brief Forms the position for the side blocking supporter. Needed in order to ensure that there is no opening in the defences
    /// @param tacticWeightPoint << generally the ball location, unless the ball is lost, or a different tactic weightpoint is needed
    /// @return Robot relative position
    Pose2f getSideBlockerPosition(Vector2f tacticWeightPoint)
    {
      // Getting the ally goal
      Vector2f goalCenter(theFieldDimensions.xPosOwnGroundLine, 0.0f);
      
      Pose2f sideBlockerPos (
            theRobotPose.rotation,
            goalCenter.x() + (tacticWeightPoint.x() - goalCenter.x()) * params.sideBlockerPositionRatio,
            (-1.0f) * (goalCenter.y() + (tacticWeightPoint.y() - goalCenter.y()) * params.sideBlockerPositionRatio)
      );
      // Record the direction of the ball
      Vector2f ballRelativeTactic = sideBlockerPos.inverse() * theFieldBall.endPositionOnField;
      
      // Return the position of the robot and rotate it to be facing the ball.
      return theRobotPose.toRobotCoordinates(sideBlockerPos.rotate(ballRelativeTactic.angle()));
    }

    /// @brief Forms the position for the backwards blocking supporter (Defender)
    /// @param tacticWeightPoint << generally the ball location, unless the ball is lost, or a different tactic weightpoint is needed
    /// @return Robot relative position
    Pose2f getBackBlockerPosition(Vector2f tacticWeightPoint) 
    {
      // Getting the ally goal
      Vector2f goalCenter(theFieldDimensions.xPosOwnGroundLine, 0.0f);

      // Get the point between the goal and ball based on the position ratio parameter.    
      Vector2f backBlockerPos = goalCenter + (tacticWeightPoint - goalCenter) * params.defenderPositionRatio;
 
      // Set the positions on the field
      Pose2f tacticPoseOnField(theRobotPose.rotation, backBlockerPos);

      // Get the field ball field position relative to the robot's tactic position.
      Vector2f ballRelativeTactic = tacticPoseOnField.inverse() * theFieldBall.endPositionOnField;

      // Return the robot's position ensuring that the robot is facing the ball.
      return theRobotPose.toRobotCoordinates(tacticPoseOnField.rotate(ballRelativeTactic.angle()));
    }

    /// @brief Forms the position for the forward blocking supporter
    /// @param tacticWeightPoint << generally the ball location, unless the ball is lost, or a different tactic weightpoint is needed
    /// @return Robot relative position
    Pose2f getForwardBlockerPosition(Vector2f tacticWeightPoint) 
    {
      // Getting the ally goal
        Vector2f goalCenter(theFieldDimensions.xPosOwnGroundLine, 0.0f);

        // Get the field relative angle for the line between the ball and goal
        Angle slopeAngle = atan2f(tacticWeightPoint.y() - goalCenter.y(), tacticWeightPoint.x() - goalCenter.x()) + 180_deg;
 
        // Make the position to be 75+10 cm away from the ball in order to block that direction
        Pose2f forwardBlockerPos(
                theRobotPose.rotation,
                tacticWeightPoint.x() + (params.exclusionRadius + params.exclusionThreshold) * cosf(slopeAngle),
                tacticWeightPoint.y() + (params.exclusionRadius + params.exclusionThreshold) * sinf(slopeAngle)
        );
        // Record the direction of the ball
        Vector2f ballRelativeTactic = forwardBlockerPos.inverse() * theFieldBall.endPositionOnField;
        
        // Return the position of the robot and rotate it to be facing the ball.
        return theRobotPose.toRobotCoordinates(forwardBlockerPos.rotate(ballRelativeTactic.angle()));
    }

    /// @brief Forms the position for the interceptor supporter (only relevant when it is the opponent's kick)
    /// @param tacticWeightPoint << generally the ball location, unless the ball is lost, or a different tactic weightpoint is needed
    /// @return Robot relative position
    Pose2f getInterceptorPosition(Vector2f tacticWeightPoint) 
    {
      Vector2f referencePos(
            ((tacticWeightPoint.x() > 0.0f) ? 1.0f : -1.0f) * (params.exclusionRadius + params.exclusionThreshold),
            ((tacticWeightPoint.y() > 0.0f) ? 1.0f : -1.0f) * (params.exclusionRadius + params.exclusionThreshold)
      );
      if (theGameInfo.setPlay != SET_PLAY_CORNER_KICK)
        referencePos.x() = 0; 

      Pose2f intercetorPos(theRobotPose.rotation, tacticWeightPoint - referencePos);
      
      // Record the direction of the ball
      Vector2f ballRelativeTactic = intercetorPos.inverse() * theFieldBall.endPositionOnField;
      
      // Return the position of the robot and rotate it to be facing the ball.
      return theRobotPose.toRobotCoordinates(intercetorPos.rotate(ballRelativeTactic.angle()));
    }

    // Get the exclusion zone enforcing path
    Pose2f getOutExclZone() {
      if((theFieldBall.recentBallPositionOnField() - Vector2f(0.0f, 0.0f)).squaredNorm() >= params.exclusionRadius + params.exclusionThreshold)
      {
        Pose2f walkingPose(theRobotPose.rotation, 0.0f, 0.0f); // field relative
        Vector2f ballRelativeTactic = walkingPose.inverse() * theFieldBall.endPositionOnField;
        
        return theRobotPose.toRobotCoordinates(walkingPose.rotate(ballRelativeTactic.angle()));
      } 
      else 
        return Pose2f(0_deg, -50.0f, 0.0f);
    }

    void doHeadSkill()
    {
      if (theFieldBall.positionRelative.squaredNorm() < sqr(params.lookAtBallDistance))
        headSkills.lookAtBall(/* mirrored */ false, /* forceOwnEstimate */ !theFieldBall.ballWasSeen(10000.f));
      else
        headSkills.lookActive(/* withBall */ true);
    }
  };

  /**
   * Task for supporter behaviour at kickoff. We should look active and stand in
   * one place for a defensive kickoff until the ball is free, and yield to the 
   * normal supporter behaviour for a attacking kickoff.
  */
  CRBEHAVIOUR(SupporterKickOffTask)
  {
    CRBEHAVIOUR_INIT(SupporterKickOffTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();
      
      if (theGameInfo.isOurKick())  // attacking kickoff
      {
        CR_CHECKPOINT(our_kickoff);
        while (getCheckpointDuration() < static_cast<unsigned>(theGameConfig.kickoffDuration + params.kickoffDurationMargin))
        {
          // commonSkills.activityStatus(BehaviorStatus::reKickoff);
          headSkills.lookActive(true);
          commonSkills.stand();

          if (theGameInfo.isBallFree()) // by definition ballFree means kickoff over
            CR_EXIT_SUCCESS();
          else 
            CR_YIELD();
        }
      }
      else  // defensive kickoff
      {
        CR_CHECKPOINT(opponent_kickoff);

        while (getCheckpointDuration() < static_cast<unsigned>(theGameConfig.kickoffDuration + params.kickoffDurationMargin))
        {
          // commonSkills.activityStatus(BehaviorStatus::reKickoff);
          headSkills.lookActive(true);
          commonSkills.stand();

          if (theGameInfo.isBallFree()) // by definition ballFree means kickoff over
            CR_EXIT_SUCCESS();
          else 
            CR_YIELD();
        }
      }

    }

  private:
    DEFINES_PARAMS(SupporterKickOffTask,
    {,
      (int)(1000) kickoffDurationMargin, // a safety margin for ball free calls
    });

    READS(GameConfig);
    READS(GameInfo);
    // READS(ExtendedGameInfo);
      
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
  };



  /**
   * This is the entry point task for the supporter playing state.
   */
  CRBEHAVIOUR(SupporterPlayingTask)
  {
    CRBEHAVIOUR_INIT(SupporterPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {

        if (theGameInfo.isKickoff())
        {
          CR_CHECKPOINT(kickoff);
          annotation("kickoff");

          while (!supporterKickOffTask.isEnded())
          {
            supporterKickOffTask();
            CR_YIELD();
          }
        }


        while (needsClearanceKick())
        {
          CR_CHECKPOINT(needs_clearance_kick);
          gotoBallAndKickBestOptionTask(/* clearance*/ true);
          CR_YIELD();
        }

        CR_CHECKPOINT(normal);
        gotoTacticPoseTask();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(SupporterPlayingTask,
    {,
      (unsigned)(3000) kickAwayBallSeenTimeoutMs, ///< we'll allow the ball to be obstructed briefly on the way to kicking it away
      (float)(1000.f) kickAwayXLimit, ///< x-position on field must be less than this to consider it (i.e. closer than slightly into opponent half)
      (float)(1500.f) kickAwayDistance, ///< distance to ball must be less than this to consider it
      (float)(100.f) veryClose,
      (int)(10000) kickoffTimeMs, // after this time the ball is free and anyone can take it
      (float)(600) teammateThresholdToBall, // if there is a teammate inside this distance we will let them kick the ball
      (float)(600) closerThanTeammateToBallThreshold, // must be closer to the ball by this much over any teammate to proceed
    });

    READS(GameInfo);
    READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(ExtendedGameInfo);
    READS(ObstacleModel);
    READS(RobotPose);
      
    CommonSkills commonSkills {env};
    GotoTacticPoseTask gotoTacticPoseTask {env};
    GotoBallAndKickBestOptionTask gotoBallAndKickBestOptionTask {env};
    // SupporterClearanceTask supporterClearanceTask {env};
    HeadSkills headSkills {env};
    SupporterKickOffTask supporterKickOffTask {env};
    ObstacleSkills obstacleSkills {env};

    /**
     * is the ball situation such that our best bet is to walk out to the ball and
     * kick it away?
     */
    bool needsClearanceKick()
    {
      // TODO: the current implementation is pretty naive and just looks at the
      // ball position. It would be better to consider things like, where
      // are the opponent players, where are our defensive players, before
      // committing to go 1v1

      // NOTE: that we also assume that if the goalie is the closest robot
      // to the ball it will role switch to become the ball player, in which
      // case that behaviour will run (and kick the ball) rather than this one

      bool ballSeenRecently = theFieldBall.ballWasSeen(params.kickAwayBallSeenTimeoutMs);
      if (!ballSeenRecently)
        return false;

      float myDistanceToBall = theFieldBall.endPositionRelative.norm();

      if ((theFieldBall.endPositionOnField.x() > params.kickAwayXLimit) || (myDistanceToBall > params.kickAwayDistance))
        return false;

      // if the ball is very close to us (i.e. at our feet) kick it away
      if (myDistanceToBall < params.veryClose)
        return true;


      // bool ballInRange = (theFieldBall.endPositionOnField.norm() < params.kickAwayDistance); // FIXME: dirty hack

      bool teammateCloseToBall = false;
      bool closerThanTeammatesToBall = true;
      // bool ballClose = false;
      // TODO: check for striker
      for (auto obstacle : theObstacleModel.obstacles)
      {
        if (obstacle.type != Obstacle::teammate)
          continue;

        if (!teammateCloseToBall && (commonSkills.isPointClose(obstacle.center - theFieldBall.endPositionRelative,
                                                               params.teammateThresholdToBall)))
          teammateCloseToBall = true;

        if (closerThanTeammatesToBall)
        {
          float teammateDistanceToBall = (obstacle.center - theFieldBall.endPositionRelative).norm();

          if (myDistanceToBall + params.closerThanTeammateToBallThreshold > teammateDistanceToBall)
            closerThanTeammatesToBall = false;
        }
      }

      return !teammateCloseToBall && closerThanTeammatesToBall;
    }


    bool isKickoff()
    {
      // TODO - this is a naive implementation
      return (theExtendedGameInfo.gameStateBeforeCurrent == STATE_SET) &&
             (theExtendedGameInfo.timeSincePlayingStarted < params.kickoffTimeMs);
    }
  };

} // RE2024
} // CoroBehaviour
