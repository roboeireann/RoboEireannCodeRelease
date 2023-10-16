/**
 * @file PenaltyShotOther.h
 *
 * This file contains the behaviours for robots other than the striker and goalie
 * during the penalty shot.
 * 
 * This is new code (not from 2019).
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "GotoBallSkills.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/BallSkills.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Representations/BehaviorControl/KickoffState.h"

#include "Tools/Math/Pose2f.h"
#include "Tools/TextLogging.h"


namespace CoroBehaviour
{
namespace RE2023
{
  /**
   * During ready for a penalty shot everyone but the attacking stiker and
   * defensive goalie must leave the penalty area.
   * 
   * For now we keep things really simple by moving parallel to the ground line
   * to escape the penalty area.
   * 
   * We assume this is not applied to the attacking striker or defensive
   * goalie, so we'll apply it to everyone else
   */
  CRBEHAVIOUR(PenaltyShotOtherReadyTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotOtherReadyTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      if (calculateLegalPosition())
      {
        CR_CHECKPOINT(penaltykick_walk_to_tactic_pose);
        while (!walkToPoseAutoAvoidanceTask.isSuccess())
        {
          // commonSkills.activityStatus(BehaviorStatus::reWalkToTacticPose);
          headSkills.lookActive();

          walkToPoseAutoAvoidanceTask(theRobotPose.toRobotCoordinates(targetOnField), walkSpeed);
          CR_YIELD();
        }
      }

      // we've reached our legal pose so just stand there
      CR_CHECKPOINT(penaltykick_stand);
      while (true)
      {
        // commonSkills.activityStatus(BehaviorStatus::reStandAtTacticPose);
        headSkills.lookActive();
        commonSkills.stand();
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(PenaltyShotOtherReadyTask, 
    {, 
      (float)(1.f) walkSpeed,
      (float)(200.f) imprecision, ///< allowance for some imprecision in our current location
      (float)(200.f) safeExtra, ///< some extra distance to ensure we're really clear of the penalty area
      (float)(1000.f) excludeWidth, ///< don't allow other robot to occupy this zone which might obstruct the penalty taker
    });

    READS(RobotPose);
    READS(FieldDimensions);

    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};

    WalkToPoseAutoAvoidanceTask walkToPoseAutoAvoidanceTask {env};

    Pose2f targetOnField;
    Pose2f walkSpeed;

    
    DECL_TLOGGER_FN(tlogger, "PenaltyShotOtherReady", TextLogging::INFO)

    bool calculateLegalPosition()
    {
      walkSpeed = Pose2f(params.walkSpeed, params.walkSpeed, params.walkSpeed);
      
      bool changeRequired = false;
      float side = (theRobotPose.translation.y() > 0) ? 1 : -1;

      // if we are the attacking team, then we worry about the opponent penalty area.
      // Otherwise we care about our own penalty area
      if (commonSkills.isOurTeamKick())
      {
        // inside the opponent penalty area?
        if ((theRobotPose.translation.x() > (theFieldDimensions.xPosOpponentPenaltyArea - params.imprecision)) &&
            (std::fabs(theRobotPose.translation.y()) < (theFieldDimensions.yPosLeftPenaltyArea + params.imprecision)))
        {
          changeRequired = true;

          float distToSide = theFieldDimensions.yPosLeftPenaltyArea - std::fabs(theRobotPose.translation.y());
          float distToFront = theFieldDimensions.xPosOpponentPenaltyArea - theRobotPose.translation.x();

          if (distToSide < distToFront) // should we go to the side?
            targetOnField = Pose2f(-90_deg * side, theRobotPose.translation.x(),
                                   (theFieldDimensions.yPosLeftPenaltyArea + params.safeExtra) * side);
          else // no, go to the front
          {
            targetOnField = Pose2f(0, (theFieldDimensions.xPosOpponentPenaltyArea - params.safeExtra),
                                   theRobotPose.translation.y());

            if (std::fabs(theRobotPose.translation.y()) < params.excludeWidth)
              targetOnField.translation.y() = sgn(theRobotPose.translation.y()) * params.excludeWidth;
          }

          TLOGI(tlogger(), "ourKick, targetOnField = {}, {}, {}", targetOnField.rotation,
                     targetOnField.translation.x(), targetOnField.translation.y());
        }
      }
      else // not our kick - check our own penalty area
      {
        if ((theRobotPose.translation.x() < (theFieldDimensions.xPosOwnPenaltyArea + params.imprecision)) &&
            (std::fabs(theRobotPose.translation.y()) < (theFieldDimensions.yPosLeftPenaltyArea + params.imprecision)))
        {
          changeRequired = true;

          float distToSide = theFieldDimensions.yPosLeftPenaltyArea - std::fabs(theRobotPose.translation.y());
          float distToFront = theFieldDimensions.xPosOwnPenaltyArea - theRobotPose.translation.x();

          if (distToSide < distToFront) // should we go to the side?
          {
            targetOnField = Pose2f(-90_deg * side, theRobotPose.translation.x(),
                                   (theFieldDimensions.yPosLeftPenaltyArea + params.safeExtra) * side);
          }
          else // no, go to the front
          {
            targetOnField = Pose2f(180_deg, (theFieldDimensions.xPosOwnPenaltyArea + params.safeExtra),
                                   theRobotPose.translation.y());
                                   
            if (std::fabs(theRobotPose.translation.y()) < params.excludeWidth)
              targetOnField.translation.y() = sgn(theRobotPose.translation.y()) * params.excludeWidth;
          }

          TLOGI(tlogger(), "theirKick, targetOnField = {}, {}, {}", targetOnField.rotation,
                     targetOnField.translation.x(), targetOnField.translation.y());
        }
      }

      return changeRequired;
    }
  };




  CRBEHAVIOUR(PenaltyShotOtherPlayingTask)
  {
    CRBEHAVIOUR_INIT(PenaltyShotOtherPlayingTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // commonSkills.activityStatus(BehaviorStatus::reStandAndLookActive);
        headSkills.lookActive();
        commonSkills.stand();
        CR_YIELD();
      }
    }

  private:
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
  };


} // RE2023
} // CoroBehaviour2023