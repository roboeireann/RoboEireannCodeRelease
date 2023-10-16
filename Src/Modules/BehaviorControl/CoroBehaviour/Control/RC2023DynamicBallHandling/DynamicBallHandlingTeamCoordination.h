/**
 * @file DynamicBallHandlingTeamCoordination.h
 *
 * This task implements the team coordination behaviour for the Robocup 2023
 * dynamic ball handling task. 
 *
 * @author Rudi Villing
 * @author Andy Lee Mitchell
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/TeamBehaviour2023.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

namespace CoroBehaviour
{
namespace RC2023
{

  CRBEHAVIOUR(DynamicBallHandlingTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(DynamicBallHandlingTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        teamBehaviourStatusSkills.teamActivity(TeamBehaviorStatus::dynamicBallHandlingTeam);
        teamBehaviourStatusSkills.timeToReachBall(teamBehaviourStatusSkills.calcTimeToReachBallPosition());
        updateTeammateRoles();
        updatePlayerRole();
        updatePassStatus();

        CR_YIELD();
      }
    }

  private:
    READS(RobotInfo);
    READS(TeamData);
    READS(FrameInfo);
    READS(BehaviorStatus);
    READS(TeamBehaviorStatus);
    READS(RobotPose);
    READS(GameInfo);
    READS(OwnTeamInfo);
    READS(FieldBall);
    READS(MotionInfo);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};

    enum { NO_PASS, PASS_LINE_UP, PASS_KICKED, PASS_SUBSTANTIAL } passState;
    Vector2f ballPassStartOnField;
    Vector2f ballPassTargetVector; // relative to start
    unsigned lastKickTimestamp = 0; ///< used to track when a kick has actually been performed

    DECL_TLOGGER_FN(tlogger, "DynamicBallHandlingTeamCoordination", TextLogging::INFO)

    void updateTeammateRoles()
    {
      TeammateRoles teammateRoles;
      const Teammate* captainTeammate = teamBehaviourStatusSkills.selectCaptainTeammate();

      if(!captainTeammate)
      {
        teammateRoles.captain = theRobotInfo.number;
        teammateRoles.timestamp = theFrameInfo.time;
        assignRoles(teammateRoles);
      }
      else
      {
        teammateRoles = captainTeammate->theTeamBehaviorStatus.teammateRoles;
      }

      teamBehaviourStatusSkills.teammateRoles(teammateRoles);
    }

    // get time to ball depending on whether the supplied teamBehaviourStatus
    // (from some particular robot) is striker or not
    unsigned getTimeToReachBall(int playerNumber, const TeamBehaviorStatus& teamBehaviourStatus)
    {
      // the ball player or pass receiver have equal chance to be ball player going forward
      // all other players have a penalty to prevent rapid oscillations between role assignments
      // NOTE: this is called before new ballPlayer assignments are made so it is relying on the
      // ballPlayer (=passKicker) and passReceiver from the last cycle
      if (teamBehaviourStatus.role.playsTheBall() || (teamBehaviourStatus.passStatus.passReceiver == playerNumber))
        return teamBehaviourStatus.timeToReachBall.timeWhenReachBallStriker;
      else
        return teamBehaviourStatus.timeToReachBall.timeWhenReachBall;
    }

    bool isGoalkeeper(int robotNumber)
    {
      return robotNumber == 1;
    }

    void assignRoles(TeammateRoles& teammateRoles)
    {   
        using RobotNumberToTime = std::pair<int, unsigned>;
        std::vector<RobotNumberToTime> timeToBall;

        teammateRoles[theRobotInfo.number] = PlayerRole::none;

        if (!theRobotInfo.isPenalized() && isGoalkeeper(theRobotInfo.number)) // are we the goalie?
          teammateRoles[theRobotInfo.number] = PlayerRole::goalkeeper;
        else // no, perhaps a teammate is goalie
        {
          for (const Teammate& teammate : theTeamData.teammates)
          {
            if (!teammate.isPenalized && isGoalkeeper(teammate.number))
            {
              teammateRoles[teammate.number] = PlayerRole::goalkeeper;
              TLOGD(tlogger(), "robot{}: assignRoles goalkeeper is {}\n", theRobotInfo.number, teammate.number);
              break;
            }
          }
        }

        // add in players to list
        timeToBall.emplace_back(theRobotInfo.number, getTimeToReachBall(theRobotInfo.number, theTeamBehaviorStatus));
        for (const Teammate& teammate : theTeamData.teammates)
        {
            timeToBall.emplace_back(teammate.number, getTimeToReachBall(teammate.number, teammate.theTeamBehaviorStatus));
        }


        // assigning striker based on which robot is closest to the ball
        if (!timeToBall.empty())
        {
            // sort based on time to ball. (Hysteresis already accounted for in getTimeToReachBall)
            std::sort(timeToBall.begin(), timeToBall.end(),
                    [](auto &left, auto &right) { return left.second < right.second; });

            int strikerNumber = timeToBall.front().first;

            teammateRoles[strikerNumber] = PlayerRole::ballPlayer;
        }
        
        // then assign supporters as the two other players
        struct RobotNumber
        {
            int number;
            int supporterIndex;

            RobotNumber(int number, int supporterIndex): number(number), supporterIndex(supporterIndex) {}
        };
        std::vector<RobotNumber> robotNumbersVec;

        // add the current player if they aren't the ball player
        if(teammateRoles[theRobotInfo.number] == PlayerRole::none) 
        {
            robotNumbersVec.emplace_back(theRobotInfo.number, theTeamBehaviorStatus.role.supporterIndex());
        }

        // add the other player that isn't the ball player
        for (const Teammate& teammate : theTeamData.teammates)
        {
            if(teammateRoles[teammate.number] == PlayerRole::none)
            {
                robotNumbersVec.emplace_back(teammate.number, teammate.theTeamBehaviorStatus.role.supporterIndex());
            }
        }

        // fill in the supporter roles
        int supporterIndex = 0;
        for (auto& robotNumber : robotNumbersVec)
          teammateRoles[robotNumber.number] = PlayerRole::firstSupporterRole + supporterIndex++;

    }

    void updatePlayerRole()
    {
      PlayerRole role;

      // we assume that the TeammateRoles have already been updated by the time this runs
      // so we can pull our values from there

      role.role = static_cast<PlayerRole::RoleType>(theTeamBehaviorStatus.teammateRoles[theRobotInfo.number]);
      role.numOfActiveSupporters = 0;
      for (int teammateRole : theTeamBehaviorStatus.teammateRoles.roles)
        if (teammateRole >= PlayerRole::firstSupporterRole)
          ++role.numOfActiveSupporters;

      teamBehaviourStatusSkills.playerRole(role);
    }

    void updatePassStatus()
    {
      PassStatus passStatus;

      // at this point the ball player for this cycle has been updated and
      // might be different to the ball player last cycle. The pass kicker
      // (who might no longer be the ballPlayer) has the best ball visibility 
      // and is in the best position to determine if a pass was made

      passStatus.numPasses = theTeamBehaviorStatus.passStatus.numPasses; 

      if ((passState == NO_PASS) && (theRobotInfo.number == theTeamBehaviorStatus.passStatus.passKicker) &&
          // ball is close to the robot
          (theFieldBall.positionRelative.norm() < 250.f) &&
          // ball is stopped or nearly stopped
          ((theFieldBall.positionRelative - theFieldBall.endPositionRelative).norm() < 100.f))
      {
        passState = PASS_LINE_UP;
        ballPassStartOnField = theFieldBall.positionOnField;
      }
      else if (passState == PASS_LINE_UP)
      {
        ballPassTargetVector = theBehaviorStatus.shootingTo - ballPassStartOnField;
        
        // has the ball moved during line up?
        if (theMotionInfo.lastKickTimestamp != lastKickTimestamp)
        {
          lastKickTimestamp = theMotionInfo.lastKickTimestamp;
          passState = PASS_KICKED;
        }
        else if ((theFieldBall.positionOnField - ballPassStartOnField).norm() > 150.f)
          passState = NO_PASS;
      }
      else if (passState == PASS_KICKED)
      {
        Vector2f ballVector = theFieldBall.positionOnField - ballPassStartOnField;
        Vector2f ballEndVector = theFieldBall.endPositionOnField - ballPassStartOnField;
        
        // was the ball kicked far enough in the general direction intended?
        if ((ballVector.norm() > 1000.f) &&
            (Angle(ballEndVector.angle()).diffAbs(ballPassTargetVector.angle()) < 15_deg))
        {
          passStatus.numPasses++;
          passState = NO_PASS;
        }
      }
      else
      {
        passState = NO_PASS;
      }

      // set the actual numPasses to the largest value reported by any robot
      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (teammate.theTeamBehaviorStatus.passStatus.numPasses > passStatus.numPasses)
          passStatus.numPasses = teammate.theTeamBehaviorStatus.passStatus.numPasses;
      }

      // update the pass kicker and receiver for this cycle
      if (theTeamBehaviorStatus.role.playsTheBall())
      {
        if (theBehaviorStatus.passTarget != -1)
        {
          passStatus.passKicker = theRobotInfo.number;
          passStatus.passReceiver = theBehaviorStatus.passTarget;
        }
      }
      else
      {
        for (const Teammate& teammate : theTeamData.teammates)
        {
          if (theTeamBehaviorStatus.teammateRoles[teammate.number] == PlayerRole::ballPlayer)
          {
            if (teammate.theBehaviorStatus.passTarget != -1)
            {
              passStatus.passKicker = teammate.number;
              passStatus.passReceiver = teammate.theBehaviorStatus.passTarget;
            }
            break;
          }
        }
      }
      
      teamBehaviourStatusSkills.setPassStatus(passStatus);
    }

  };

} // RC2023
} // CoroBehaviour2022