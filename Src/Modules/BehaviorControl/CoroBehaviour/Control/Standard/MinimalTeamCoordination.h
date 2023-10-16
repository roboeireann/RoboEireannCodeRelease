/**
 * @file MinimalTeamCoordination.h
 *
 * This task implements a minimal team coordination behaviour that doesn't adapt
 * to game situations much.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/TeamBehaviourCommon.h"

#include "Modules/BehaviorControl/CoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

#include "Tools/TextLogging.h"

namespace CoroBehaviour
{

  CRBEHAVIOUR(MinimalTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(MinimalTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        teamBehaviourStatusSkills.teamActivity(TeamBehaviorStatus::minimalTeam);
        teamBehaviourStatusSkills.timeToReachBall(teamBehaviourStatusSkills.calcTimeToReachBallPosition());
        updateTeammateRoles();
        updatePlayerRole();
        
        CR_YIELD();
      }
    }

  private:
    DEFINES_PARAMS(MinimalTeamCoordinationTask, 
    {,
      (unsigned)(3000) strikerBonusMs, ///< any non-striker needs to reach the ball this much quicker to replace the striker
      (float)(500.f) supporterHysteresis, ///< supporters must change x-value by this relative to each other to switch index
    });

    READS(RobotInfo);
    READS(TeamData);
    READS(FrameInfo);
    READS(TeamBehaviorStatus);
    READS(RobotPose);
    READS(GameInfo);
    READS(OwnTeamInfo);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};

    DECL_TLOGGER_FN(tlogger, "MinimalTeamCoord", TextLogging::INFO)

    /**
     * update the TeammateRoles part of the teamBehaviourStatus
     */
    void updateTeammateRoles()
    {
      TeammateRoles teammateRoles;
      const Teammate* captainTeammate = teamBehaviourStatusSkills.selectCaptainTeammate();

      if (!captainTeammate) // no captain teammate would mean we're the captain
      {
        if (theRobotInfo.isPenalized()) // we're the captain but also penalized so can't allocate roles
          teammateRoles = TeammateRoles();
        else
        {
          // assign roles
          teammateRoles.captain = theRobotInfo.number;
          teammateRoles.timestamp = theFrameInfo.time;
          assignRoles(teammateRoles);
        }
      }
      else // copy the captains teamRoles
      {
        TLOGD(tlogger(), "copy captainTeammate {}", captainTeammate->number);
        teammateRoles = captainTeammate->theTeamBehaviorStatus.teammateRoles;
      }

      teamBehaviourStatusSkills.teammateRoles(teammateRoles);
    }

    // get time to ball depending on whether the supplied teamBehaviourStatus
    // (from some particular robot) is striker or not
    unsigned getTimeWhenReachBall(const TeamBehaviorStatus& teamBehaviourStatus)
    {
      bool isStriker = teamBehaviourStatus.role.playsTheBall();
      return isStriker ? teamBehaviourStatus.timeToReachBall.timeWhenReachBallStriker
                       : teamBehaviourStatus.timeToReachBall.timeWhenReachBall;
    }

    bool isGoalkeeper(int robotNumber)
    {
      return robotNumber == 1;
    }

    /**
     * implement a very basic dynamic role assignment algorithm.
     * This functionality is only executed by the captain currently
     */
    void assignRoles(TeammateRoles& teammateRoles)
    {
      if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
      {
        if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
          teammateRoles[theRobotInfo.number] = PlayerRole::ballPlayer;
        else
          teammateRoles[theRobotInfo.number] = PlayerRole::goalkeeper;

        return;
      }

      // CAUTION: remember that there may be no active teammates so ensure
      // all code works even with no teammates
      
      teammateRoles[theRobotInfo.number] = PlayerRole::none;

      // assign the goalie
      if (!theRobotInfo.isPenalized() && isGoalkeeper(theRobotInfo.number)) // are we the goalie?
        teammateRoles[theRobotInfo.number] = PlayerRole::goalkeeper;
      else // no, perhaps a teammate is goalie
      {
        for (const Teammate& teammate : theTeamData.teammates)
        {
          if (!teammate.isPenalized && isGoalkeeper(teammate.number))
          {
            teammateRoles[teammate.number] = PlayerRole::goalkeeper;
            TLOGD(tlogger(), "robot{}: assignRoles goalkeeper is {}", theRobotInfo.number, teammate.number);
            break;
          }
        }
      }

      // assign the best striker based on distance to the ball
      // (can also be the goalie if it's closest)

      using RobotNumberToTime = std::pair<int, unsigned>;
      // using RobotNumberToX = std::pair<int, float>;

      std::vector<RobotNumberToTime> etaAtBall;

      // add the current robot as a potential striker if it is not penalized
      if (!theRobotInfo.isPenalized()) 
        etaAtBall.emplace_back(theRobotInfo.number, getTimeWhenReachBall(theTeamBehaviorStatus));

      // add other unpenalized robots
      for (const Teammate& teammate : theTeamData.teammates)
      {
        if (!teammate.isPenalized)
          etaAtBall.emplace_back(teammate.number, getTimeWhenReachBall(teammate.theTeamBehaviorStatus));
      }

      // if there are any unpenalized robots
      if (!etaAtBall.empty())
      {
        // sort based on time to ball. (Hysteresis already accounted for in getTimeWhenReachBall)
        std::sort(etaAtBall.begin(), etaAtBall.end(),
                  [](auto &left, auto &right) { return left.second < right.second; });


        // for (auto rob : etaAtBall)
        // {
        //   TLOGD(tlogging(), "robot{} {}ms, ", rob.first, rob.second);
        // }

        int strikerNumber = etaAtBall.front().first;

        // we prefer the goalie not to be striker, how does the second best alternative look?
        if ((teammateRoles[strikerNumber] == PlayerRole::goalkeeper) && (etaAtBall.size() >= 2))
        {
          int striker2 = etaAtBall[1].first;

          // if ((etaAtBall[1].second - etaAtBall[0].second) < 4000) // FIXME: GORE2022 HACK
            strikerNumber = striker2;    
        }

        teammateRoles[strikerNumber] = PlayerRole::ballPlayer;
      }

      // sort the robot x positions (how far forward on the field each robot is) for robots other than striker and goalie

      struct RobotX
      {
        int number;
        int supporterIndex;
        float x;

        RobotX(int number, int supporterIndex, float x) : number(number), supporterIndex(supporterIndex), x(x) {}
      };
      std::vector<RobotX> robotXValues;
      
      // add the current robot as a supporter if it is not penalized and not a striker or goalie
      if (!theRobotInfo.isPenalized() && (teammateRoles[theRobotInfo.number] == PlayerRole::none))
        robotXValues.emplace_back(theRobotInfo.number, theTeamBehaviorStatus.role.supporterIndex(),
                                  theRobotPose.translation.x());

      // add other unpenalized robots
      for (const Teammate& teammate : theTeamData.teammates)
      {
        PlayerRole teammateRole = teammate.theTeamBehaviorStatus.role;
        if (!teammate.isPenalized && (teammateRoles[teammate.number] == PlayerRole::none))//((teammateRole.role == PlayerRole::none) || (teammateRole.supporterIndex() >= 0)))
          robotXValues.emplace_back(teammate.number, teammate.theTeamBehaviorStatus.role.supporterIndex(),
                                    teammate.theRobotPose.translation.x());
      }

      if (!robotXValues.empty())
      {
        // sort by x value (i.e. how far forward on the field) ordered from furthest back to furthest forward 
        // NOTE: supporterIndex values are already sorted from furthest back to furthest forward (when the last role assignment was done) 
        std::sort(robotXValues.begin(), robotXValues.end(),
                  [&](auto &a, auto &b)
                  {
                    return a.supporterIndex < b.supporterIndex ? a.x < (b.x + params.supporterHysteresis)
                                                               : (a.x + params.supporterHysteresis) < b.x;
                  });

        // finally fill in the teammate roles from farthest back to farthest forward
        int supporterIndex = 0;
        for (auto& robotX : robotXValues)
          teammateRoles[robotX.number] = PlayerRole::firstSupporterRole + supporterIndex++;
      }


      TLOGD(tlogger(), "assignRoles: teammateRole.roles[1] = {}", teammateRoles.roles[1]);

      // ANNOTATION_FMT("assignRoles: captain={}, striker={}, robot1 is {}\n", theRobotInfo.number, strikerNumber, teammateRoles.roles[1]);
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

      // ANNOTATION_FMT("playerRole: {}, numActiveSupporters={}\n", role.role, role.numOfActiveSupporters);      

      teamBehaviourStatusSkills.playerRole(role);
    }    
  };

} // CoroBehaviour2022
