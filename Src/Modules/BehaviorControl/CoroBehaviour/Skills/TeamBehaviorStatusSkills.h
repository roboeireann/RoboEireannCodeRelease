/**
 * @file: TeamBehaviorStatusSkills.h
 *
 * Record various team behaviour status information
 * (Note: american spelling on file to match underlying TeamBehaviorStatus representation)
 *
 * @author: Rudi Villing
 */

#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/TeamBehaviourCommon.h"



namespace CoroBehaviour
{

  struct TeamBehaviorStatusSkills
  {
    TeamBehaviorStatusSkills(BehaviourEnv& env) : env(env) {}

    // ------------------------------------------------------------------------
    // stateless functions (not coro/tasks) to call directly
    // ------------------------------------------------------------------------

    // see BH2021 Skills/Output/TeamBehaviorStatus/*

    void teamActivity(TeamBehaviorStatus::TeamActivity teamActivity)
    {
      theTeamBehaviorStatus.teamActivity = teamActivity;
      theLibCheck.inc(LibCheck::teamActivity);
    }

    void timeToReachBall(const TimeToReachBall& timeToReachBall)
    {
      theTeamBehaviorStatus.timeToReachBall = timeToReachBall;
      theLibCheck.inc(LibCheck::timeToReachBall);
    }

    void teammateRoles(const TeammateRoles& teammateRoles)
    {
      theTeamBehaviorStatus.teammateRoles = teammateRoles;
      theLibCheck.inc(LibCheck::teammateRoles);
    }

    void playerRole(const PlayerRole& playerRole)
    {
      theTeamBehaviorStatus.role = playerRole;
      theLibCheck.inc(LibCheck::role);
    }

    void setPassStatus(const PassStatus &passStatus)
    {
      theTeamBehaviorStatus.passStatus = passStatus;
      // FIXME: add in a libcheck for this?
    }


    // utility functions to calculate values that are useful for populating the
    // objects above

    /**
     * return the time to reach the ball in ms
     * 
     * This is straight line time to reach the ball and (naively) assumes
     * no obstacles. Additionally it does not account for the approach direction
     * so even when the robot reaches the ball position it might be on the wrong
     * side.
     */
    unsigned getTimeToBallPositionMs()
    {
      if (!theFieldBall.ballWasSeen(params.maxTimeSinceBallSeen))
        return std::numeric_limits<unsigned>::max();

      float distanceToBall = theFieldBall.positionRelative.norm();
      float timeToBallSecs = distanceToBall / theWalkingEngineOutput.maxSpeed.translation.x();
      return static_cast<unsigned>(timeToBallSecs * 1000.0f); // convert to ms
    }

    unsigned getRotationExtraMs()
    {
      if (!theFieldBall.ballWasSeen(params.maxTimeSinceBallSeen))
        return 0;

      // mainly we want to detect being wrong side of the ball so just check
      // orientation of pose


      Angle rotationUpfield = Angle(theRobotPose.rotation + theFieldBall.endPositionOnField.angle()).diffAbs(0);
      
      if (rotationUpfield > 45_deg)
      {
        float timeToRotateSecs = (rotationUpfield - 45_deg) / theWalkingEngineOutput.maxSpeed.rotation;
        return static_cast<unsigned>(timeToRotateSecs * 1000.0f); // convert to ms
      }
      else
        return 0;
    }

    /**
     * calculate the time to reach the ball position ignoring any final 
     * rotation required to be in a sensible position to play the ball
     */
    TimeToReachBall calcTimeToReachBallPosition()
    {
      if (theRobotInfo.isPenalized())
        return TimeToReachBall(); // max values

      unsigned timeToBallPosMs = getTimeToBallPositionMs();
      if (timeToBallPosMs != std::numeric_limits<unsigned>::max())
      {
        // FIXME: I think we have forgotten conditions about fallen striker etc.
        TimeToReachBall t;
        t.timeWhenReachBallStriker = theFrameInfo.time + timeToBallPosMs + getRotationExtraMs();
        t.timeWhenReachBall = t.timeWhenReachBallStriker + params.strikerReachBallBonusMs;
        return t;
      }
      else
        return TimeToReachBall();
    }

    const Teammate* selectCaptainTeammate()
    {
      int captain = !theRobotInfo.isPenalized() ? theRobotInfo.number : 0; // initial guess is that the current robot is captain
      const Teammate* captainTeammate = nullptr;

      // search for a better captain (which will select the goalie if available)
      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (!teammate.isPenalized && (teammate.number < captain)) // have we found a better captain?
        {
          captain = teammate.number;
          captainTeammate = &teammate;
        }
      }

      return captainTeammate;
    }



  private:
    struct Params
    {
      int maxTimeSinceBallSeen = 7000; // ms
      unsigned strikerReachBallBonusMs = 4000;
    }
    params;

    BehaviourEnv& env;

    READS(LibCheck);
    READS(FieldBall);
    READS(WalkingEngineOutput);
    READS(RobotInfo);
    READS(TeamData);
    READS(FrameInfo);
    READS(RobotPose);

    MODIFIES(TeamBehaviorStatus);
  };

} // CoroBehaviour
