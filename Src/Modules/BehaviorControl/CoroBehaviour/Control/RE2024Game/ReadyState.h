/**
 * @file ReadyStateTask.h
 *
 * This task implements the ready state behaviour (proceeding to kickoff positions).
 * It has been adapted to the 2021 rules regarding default start positions
 * for the robots.
 * 
 * Some of the code for getting to kickoff pose is based on the BH2021
 * WalkToKickoffPose skill.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2024/CoroBehaviour2024.h"

#include "PenaltyShotGoalie.h"
#include "PenaltyShotStriker.h"
#include "PenaltyShotOther.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

#include "Representations/BehaviorControl/Skills.h"

#include "Tools/Math/Pose2f.h"

#include "Tools/TextLogging.h"


namespace CoroBehaviour
{
namespace RE2024
{
  CRBEHAVIOUR(GotoKickoffYTask)
  {
    CRBEHAVIOUR_INIT(GotoKickoffYTask) {}

    void operator()(const Pose2f& targetOnField)
    {
      updateTargetRelative(targetOnField);
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(targetRel, /* rough: */ true, /* disableAvoidance */ true);

      // addActivationGraphOutput(fmt::format("targetRel = {:.1f}_deg, x={:.0f}, y={:.0f}",
      //                                      Angle(targetRel.rotation).toDegrees(), targetRel.translation.x(),
      //                                      targetRel.translation.y()));

      CRBEHAVIOUR_LOOP()
      {
        headSkills.lookActive(/*withBall*/ false, /*ignoreBall*/ true);
        // motionSkills.walkToPose(targetRel, Pose2f(params.speed, params.speed, params.speed), obstacleAvoidance, /* keepTargetRotation */ false);
        walkToPoseTask(targetRel, params.speed, AVOIDANCE_NONE, /* keepTargetRotation */ true);

        if (closeEnoughToY(targetOnField))
          CR_EXIT_SUCCESS();
        else
          CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(GotoKickoffYTask,
    {,
      (float)(1.f) speed, 
      (float)(1000.f) yMidlineThreshold, ///< distance threshold to mid-line of field (i.e the line from goal to goal)
      (float)(500.f) yTargetThreshold, ///< if the y-distance to target is less than this, we're done
      (float)(1200.f) yGoalieThreshold, ///< if the y-distance to target is less than this, we're done
    });

    READS(RobotPose);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    READS(LibWalk);

    HeadSkills headSkills {env};
    WalkToPoseTask walkToPoseTask {env};
    MotionSkills motionSkills  {env};

    Pose2f targetRel;
    int side = 0;

    void updateTargetRelative(const Pose2f& targetOnField)
    {
      if (side == 0)
        side = sgn(theRobotPose.translation.y());

      Pose2f modTargetOnField = Pose2f(-90_deg * side, theRobotPose.translation.x(), targetOnField.translation.y());

      targetRel = theRobotPose.toRobotCoordinates(modTargetOnField);
    }

    bool closeEnoughToY(const Pose2f& targetOnField)
    {
      const float distMidline = std::fabs(theRobotPose.translation.y());
      const float distTarget = std::fabs(targetOnField.translation.y() - theRobotPose.translation.y());

      if (theGameInfo.playerNumber == 1)
        return (distTarget < params.yGoalieThreshold);
      else
        return ((distTarget < params.yTargetThreshold) || (distMidline < params.yMidlineThreshold));
    }
  };



  // common walk to ready pose behaviour (whether from sideline or after a goal)
  // This code is loosely based on WalkToKickoffPose skill from BH2021 code release
  CRBEHAVIOUR(GotoKickoffTask)
  {
    CRBEHAVIOUR_INIT(GotoKickoffTask) {}

    void operator()(const Pose2f& targetOnField)
    {
      const Pose2f targetRelative = theRobotPose.toRobotCoordinates(targetOnField);
      const float targetDist = targetRelative.translation.norm();
      const float targetAngle = std::abs(targetRelative.rotation);
      const int timeUntilSetState = getTimeUntilSetState();

      auto inOwnHalf = [this](float margin)
      {
        return theRobotPose.translation.x() < -margin &&
               (theGameInfo.isOurKick() ||
                theRobotPose.translation.squaredNorm() > sqr(theFieldDimensions.centerCircleRadius + margin));
      };

      const bool urgentAdjustmentNeeded = timeUntilSetState < params.timeToFinalTurn &&
                                  targetDist > params.minDistanceFinalTurn &&
                                  targetAngle > params.minAngleFinalTurn && 
                                  inOwnHalf(params.ownHalfMargin.max);
      const bool insideCenterCircle = theRobotPose.translation.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius);

      auto needsAdjustment = [this,targetDist,targetAngle,insideCenterCircle]()
      {
        if (insideCenterCircle)
          return (targetDist > params.fineAdjustEndDistance || targetAngle > params.fineAdjustEndAngle);
        else
          return (targetDist > params.adjustEndDistance || targetAngle > params.adjustEndAngle);
      };


      CRBEHAVIOUR_LOOP()
      {
        // goto approx kickoff pose
        while (!urgentAdjustmentNeeded && (targetDist >= params.approxDistance.min))
        {
          CR_CHECKPOINT(goto_kickoff_pose);
          headSkills.lookActive(/*withBall*/ false, /*ignoreBall*/ true);
          walkToPoseTask(targetRelative,
                         theExtendedGameInfo.walkingInFromSidelines ? params.walkInSpeed : params.normalSpeed,
                         (targetDist < params.distRough) && (timeUntilSetState < params.timeToRough) ? AVOIDANCE_ROUGH
                                                                                                     : AVOIDANCE_AUTO,
                         /* keepTargetRotation */ false, /* standAtTarget */ true);
          CR_YIELD();
        }

        // adjust to final pose (more precisely if inside circle)
        CR_CHECKPOINT(adjusting_pose);
        while (!urgentAdjustmentNeeded && (needsAdjustment() || (getCheckpointDuration() < params.adjustmentMs)))
        {
          headSkills.lookActive(/*withBall*/ false, /*ignoreBall*/ true);
          walkToPoseTask(targetRelative, insideCenterCircle ? params.fineAdjustingSpeed : params.adjustingSpeed,
                          theTeamBehaviorStatus.role.isGoalkeeper() ? AVOIDANCE_NONE : AVOIDANCE_ROUGH,
                          /* keepTargetRotation */ false, /* standAtTarget */ true);
          CR_YIELD();
        }

        // do we need a final turn on the spot because we're running out of time before SET?
        while (urgentAdjustmentNeeded)
        {
          CR_CHECKPOINT(urgent_final_turn);
          headSkills.lookActive(/*withBall*/ false, /*ignoreBall*/ true);
          walkToPoseTask(Pose2f(targetRelative.rotation), params.finalTurnSpeed, AVOIDANCE_NONE,
                          /* keepTargetRotation */ false, /* standAtTarget */ true);
          CR_YIELD();
        }

        // stand normal/high (as long as we don't need to reposition)
        CR_CHECKPOINT(stand_at_kickoff_pose);
        while (targetDist < params.approxDistance.max && targetAngle < params.approxAngleMax && inOwnHalf(params.ownHalfMargin.min))
        {
          headSkills.lookActive(/*withBall*/ false, /*ignoreBall*/ true);
          motionSkills.stand(/* high */ motionSkills.isStanding() && getCheckpointDuration() > params.standMs);          
          CR_YIELD();
        }

        // TLOGD(tlogger, "urgentAdjustmentNeeded={}, targetDist={:0.1f}, targetAngle={:.1f}, ownHalfMin={}",
        //       urgentAdjustmentNeeded, targetDist, Angle(targetAngle).toDegrees(),
        //       inOwnHalf(params.ownHalfMargin.min));

        CR_END_OF_LOOP_CHECK(); // ensure at least one of the steps above has executed and yielded
      }
    }


  private:
    DEFINES_PARAMS(GotoKickoffTask,
    {,
      (int)(3000) timeToFinalTurn, /**< when only this time remaining, the robot will just turn to its final rotation 
                                       (if its target pose is on own side of the field) */
      (int)(10000) timeToRough, ///< remaining time at which rough avoidance will be used
      (unsigned)(1000) adjustmentMs,
      (unsigned)(1000) standMs,
      (Rangef)({170.f, 200.f}) ownHalfMargin, // must be this distance inside own half / outside center circle where applicable
      (Rangef)({200.f, 300.f}) approxDistance,
      (Angle)(15_deg) approxAngleMax,
      (float)(40.f) adjustEndDistance,
      (float)(25.f) fineAdjustEndDistance,
      (Angle)(5_deg) adjustEndAngle,
      (Angle)(3_deg) fineAdjustEndAngle,
      (float)(500.f) distRough,
      (float)(500.f) minDistanceFinalTurn,
      (Angle)(20_deg) minAngleFinalTurn,
      (float)(1.f) normalSpeed, ///< speed to use for transition to ready state during game play
      (float)(1.f) walkInSpeed, ///< default speed to use during a walk in from the side
      (float)(0.9f) adjustingSpeed, ///< default speed to use while (coarsely) adjusting position
      (float)(0.7f) fineAdjustingSpeed, ///< default speed to use while fine adjusting the position
      (float)(1.f) finalTurnSpeed,
    });

    READS(RobotPose);
    READS(GameInfo);
    READS(ReceivedGameControlData);
    READS(ExtendedGameInfo);
    READS(GameConfig);
    READS(FieldDimensions);
    READS(TeamBehaviorStatus);
    READS(LibWalk);

    HeadSkills headSkills {env};
    MotionSkills motionSkills  {env};
    WalkToPoseTask walkToPoseTask {env};

    TextLogger& tlogger = TextLogging::get("GotoKickoffTask", TextLogging::INFO);


    int getTimeUntilSetState()
    {
      // secondaryTime is used because the robot might receive its first GC packet when the ready state has
      // been active for some time. On the other hand, the secondaryTime must not be used when the raw game state is
      // not yet ready (during the first 15s after a goal).
      int rawReadyRemaining = theReceivedGameControlData.state == STATE_READY
                                  ? theReceivedGameControlData.secondaryTime * 1000
                                  : theGameConfig.kickoffReadyDuration;
      int extendedReadyRemaining = theGameConfig.kickoffReadyDuration - theExtendedGameInfo.timeSinceReadyStarted;

      return std::min<int>(rawReadyRemaining, extendedReadyRemaining);
    }
  };


  // The main entry point for the normal ready state (walk in from side or after scoring a goal)

  CRBEHAVIOUR(ReadyStateKickoffTask)
  {
    CRBEHAVIOUR_INIT(ReadyStateKickoffTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_BEGIN();

      // initReadyPosesOnField();

      if (theExtendedGameInfo.walkingInFromSidelines)
      {
        // player 2 is our probe in standby state, so it is allowed to walk out first
        if ((theExtendedGameInfo.gameStateBeforeCurrent == STATE_STANDBY) && (theGameInfo.stateIsGuessed) &&
            (theGameInfo.playerNumber != 2))
        {
          CR_CHECKPOINT(wait_for_visual_ready_probe);
          while ((theExtendedGameInfo.timeSinceReadyStarted < params.probeForVisualReadyMs) &&
                 (theGameInfo.state == STATE_READY) && (theGameInfo.stateIsGuessed))
          {
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
            commonSkills.stand();
            CR_YIELD();
          }
        }
        else
        {
          CR_CHECKPOINT(lookAroundFromSideline);
          // contine looking around until time is up OR we have great localisation
          while ((getCoroDuration() < getLookAroundMs()) && (theRobotPose.quality != RobotPose::superb))
          {
            headSkills.lookActive(/* withBall: */ false, /* ignoreBall: */ true);
            commonSkills.stand();
            CR_YIELD();
          }
        }

        CR_CHECKPOINT(walkStraightOut);
        while (!gotoKickoffYTask.isSuccess())
        {
          // can adapt if the status of our team kick changes for some reason after we enter the ready state
          gotoKickoffYTask(getReadyPoseOnField());
          CR_YIELD();
        }
      }

      // from this point on behaviour is the same whether walking from the sidelines or back to positions 
      // after scoring a goal
      CR_CHECKPOINT(walkToKickoff);
      while (true)
      {
        // can adapt if the status of our team kick changes for some reason after we enter the ready state
        // CALL_SKILL(WalkToKickoffPose)(getReadyPoseOnField());
        gotoKickoffTask(getReadyPoseOnField());
        CR_YIELD();
      }
    }

  private:
    READS(RobotPose);
    READS(FieldDimensions);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    READS(ActiveTactic);

    DEFINES_PARAMS(ReadyStateKickoffTask,
    {,
      (std::array<unsigned,Settings::highestValidPlayerNumber>)({0}) lookAroundMs,
      (int)(8000) probeForVisualReadyMs, ///< if we get beyond this duration, we should be safe to walk out      
    });

    
    CommonSkills commonSkills {env};
    HeadSkills headSkills {env};
    GotoKickoffYTask gotoKickoffYTask {env};
    GotoKickoffTask gotoKickoffTask {env};


    unsigned getLookAroundMs()
    {
      return params.lookAroundMs[theGameInfo.playerIndex()];
    }

    // having this function called on every cycle allows for the possibility of game state mistakes
    // and changing whether it is an attacking or defending kickoff mid way through the ready state
    const Pose2f& getReadyPoseOnField()
    {
      return theActiveTactic.formationPose;
    }
  };




  // this is the main entry point for the ready state and it delegates to the appropriate
  // specialized behaviour to do the real work

  CRBEHAVIOUR(ReadyStateTask)
  {
    CRBEHAVIOUR_INIT(ReadyStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK)
        {
          if (!commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isGoalkeeper())
            penaltyShotGoalieReadyTask();
          else if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isBallPlayer())
            penaltyShotStrikerReadyTask();
          else
            penaltyShotOtherReadyTask();
        }
        else
          readyStateKickoffTask();

        CR_YIELD();
      }
    }

  private:
    READS(TeamBehaviorStatus);
    READS(GameInfo);

    CommonSkills commonSkills {env};

    PenaltyShotGoalieReadyTask penaltyShotGoalieReadyTask {env};
    PenaltyShotStrikerReadyTask penaltyShotStrikerReadyTask {env};
    PenaltyShotOtherReadyTask penaltyShotOtherReadyTask {env};
    ReadyStateKickoffTask readyStateKickoffTask {env};
  };


} // RE2024
} // CoroBehaviour
