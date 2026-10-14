/**
 * @file MinimalTeamCoordination.h
 *
 * This task implements a minimal team coordination behaviour that doesn't adapt
 * to game situations much.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/TeamBehaviourCommon.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

#include "TextLogging/TextLogging.h"

namespace RE
{

  CRBEHAVIOUR(MinimalTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(MinimalTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // teamBehaviourStatusSkills.recordTeamActivity(TeamBehaviorStatus::minimalTeam);
        teamBehaviourStatusSkills.recordTimeToReachBall(teamBehaviourStatusSkills.calcTimeToReachBallPosition());
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

    READS(RETeamData);
    READS(FrameInfo);
    READS(TeamBehaviorStatus);
    READS(RobotPose);
    READS(GameState);
    // READS(REGameInfo);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};

    DECL_TLOGGER_VAR(tlogger, "MinimalTeamCoord", TextLogging::INFO);

    /**
     * update the TeammateRoles part of the teamBehaviourStatus
     */
    void updateTeammateRoles()
    {
      TeammateRoles teammateRoles;
      const RETeammate* captainTeammate = teamBehaviourStatusSkills.selectCaptainTeammate();

      if (!captainTeammate) // no captain teammate would mean we're the captain
      {
        if (theGameState.isPenalized()) // we're the captain but also penalized so can't allocate roles
          teammateRoles = TeammateRoles();
        else
        {
          // assign roles
          teammateRoles.captain = theGameState.playerNumber;
          teammateRoles.timestamp = theFrameInfo.time;
          assignRoles(teammateRoles);
        }
      }
      else // copy the captains teamRoles
      {
        TLOGD(tlogger, "copy captainTeammate {}", captainTeammate->number);
        teammateRoles = captainTeammate->theTeamBehaviorStatus.teammateRoles;
      }

      teamBehaviourStatusSkills.recordTeammateRoles(teammateRoles);
    }

    // get time to ball depending on whether the supplied teamBehaviourStatus
    // (from some particular robot) is striker or not
    unsigned getTimeWhenReachBall(const TeamBehaviorStatus& teamBehaviourStatus)
    {
      bool isStriker = teamBehaviourStatus.role.isBallPlayer();
      return isStriker ? teamBehaviourStatus.timeToReachBall.timeWhenReachBallStriker
                       : teamBehaviourStatus.timeToReachBall.timeWhenReachBall;
    }

    bool isGoalkeeper(int robotNumber)
    {
      return theGameState.ownTeam.isGoalkeeper(robotNumber);
    }

    /**
     * implement a very basic dynamic role assignment algorithm.
     * This functionality is only executed by the captain currently
     */
    void assignRoles(TeammateRoles& teammateRoles)
    {
      if (theGameState.isPenaltyShootout())
      {
        if (theGameState.isForOwnTeam())
          teammateRoles[theGameState.playerNumber] = PlayerRole::ballPlayer;
        else
          teammateRoles[theGameState.playerNumber] = PlayerRole::goalkeeper;

        return;
      }

      // CAUTION: remember that there may be no active teammates so ensure
      // all code works even with no teammates
      
      teammateRoles[theGameState.playerNumber] = PlayerRole::none;

      // assign the goalie
      if (!theGameState.isPenalized() && isGoalkeeper(theGameState.playerNumber)) // are we the goalie?
        teammateRoles[theGameState.playerNumber] = PlayerRole::goalkeeper;
      else // no, perhaps a teammate is goalie
      {
        for (const RETeammate& teammate : theRETeamData.teammates)
        {
          if (!teammate.isPenalized && isGoalkeeper(teammate.number))
          {
            teammateRoles[teammate.number] = PlayerRole::goalkeeper;
            TLOGD(tlogger, "robot{}: assignRoles goalkeeper is {}", theGameState.playerNumber, teammate.number);
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
      if (!theGameState.isPenalized()) 
        etaAtBall.emplace_back(theGameState.playerNumber, getTimeWhenReachBall(theTeamBehaviorStatus));

      // add other unpenalized robots
      for (const RETeammate& teammate : theRETeamData.teammates)
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
      if (!theGameState.isPenalized() && (teammateRoles[theGameState.playerNumber] == PlayerRole::none))
        robotXValues.emplace_back(theGameState.playerNumber, theTeamBehaviorStatus.role.supporterIndex(),
                                  theRobotPose.translation.x());

      // add other unpenalized robots
      for (const RETeammate& teammate : theRETeamData.teammates)
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


      TLOGD(tlogger, "assignRoles: teammateRole.roles[1] = {}", teammateRoles.roles[1]);

      // CRB_ANNOTATION_FMT("assignRoles: captain={}, striker={}, robot1 is {}\n", theGameState.playerNumber, strikerNumber, teammateRoles.roles[1]);
    }

    void updatePlayerRole()
    {
      PlayerRole role;

      // we assume that the TeammateRoles have already been updated by the time this runs
      // so we can pull our values from there

      role.role = static_cast<PlayerRole::Type>(theTeamBehaviorStatus.teammateRoles[theGameState.playerNumber]);
      role.numOfActiveSupporters = 0;
      for (int teammateRole : theTeamBehaviorStatus.teammateRoles.roles)
        if (teammateRole >= PlayerRole::firstSupporterRole)
          ++role.numOfActiveSupporters;

      // CRB_ANNOTATION_FMT("playerRole: {}, numActiveSupporters={}\n", role.role, role.numOfActiveSupporters);      

      teamBehaviourStatusSkills.recordPlayerRole(role);
    }    
  };

} // namespace RE
