/**
 * @file PlayingState.h
 *
 * This task implements the playing state behaviour for all robots.
 * It has been adapted from the RoboEireann 2019 behaviour 
 * to fit the BH2021 code release framework and coro behaviour 
 * engine.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2023/CoroBehaviour2023.h"

#include "Striker.h"
#include "Goalie.h"
#include "Supporter.h"
#include "PenaltyShotStriker.h"
#include "PenaltyShotGoalie.h"
#include "PenaltyShotOther.h"
#include "SetPlay.h"


#include "Modules/BehaviorControl/CoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/CoroBehaviour/Skills/HeadSkills.h"

#include "Tools/Math/Pose2f.h"


namespace CoroBehaviour
{
namespace RE2023
{
  /**
   * This is the entry point task for the playing state. It decides which
   * specialized behaviour to run
   */
  CRBEHAVIOUR(PlayingStateTask)
  {
    CRBEHAVIOUR_INIT(PlayingStateTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if ((theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) || (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK))
        {
          if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.playsTheBall())
            penaltyShotStrikerPlayingTask();
          else if (!commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isGoalkeeper())
            penaltyShotGoaliePlayingTask();
          else
            penaltyShotOtherPlayingTask();
        }
        else if (theGameInfo.setPlay != SET_PLAY_NONE) // other set plays - TODO: not implemented yet
        {
          if (theTeamBehaviorStatus.role.playsTheBall())
          {
            if (commonSkills.isOurTeamKick())
            {
              if ((theGameInfo.setPlay == SET_PLAY_KICK_IN) || (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK))
                setPlayStrikerFreeKickTask();
              else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK)
                setPlayStrikerCornerKickTask();
              else
                setPlaySupporterPlayingTask(); // yes - we use supporter task just to back off a bit
            }
            else
              setPlaySupporterPlayingTask(); // yes - we use supporter task just to back off a bit
          }
          else if (theTeamBehaviorStatus.role.isGoalkeeper())
          {
            if ((theGameInfo.setPlay == SET_PLAY_GOAL_KICK) && commonSkills.isOurTeamKick())
              // setPlayGoalieGoalKickTask();
              goaliePlayingTask();
            else
              goaliePlayingTask();
          }
          else // supporter
          {
            setPlaySupporterPlayingTask();
          }
        }
        else // normal play (not a penalty shot and not some other set play)
        {
          if (theTeamBehaviorStatus.role.isGoalkeeper())
            goaliePlayingTask();
          else if (theTeamBehaviorStatus.role.playsTheBall())
            strikerPlayingTask();
          else
            supporterPlayingTask();
        }
        
        CR_YIELD();
      }
    }

  private:
    READS(GameInfo);
    READS(TeamBehaviorStatus);
    
    CommonSkills commonSkills {env};

    PenaltyShotStrikerPlayingTask penaltyShotStrikerPlayingTask {env};
    PenaltyShotGoaliePlayingTask penaltyShotGoaliePlayingTask {env};
    PenaltyShotOtherPlayingTask penaltyShotOtherPlayingTask {env};

    SetPlayStrikerFreeKickTask setPlayStrikerFreeKickTask {env};
    SetPlayStrikerCornerKickTask setPlayStrikerCornerKickTask {env};
    SetPlaySupporterPlayingTask setPlaySupporterPlayingTask {env};

    GoaliePlayingTask goaliePlayingTask {env};
    StrikerPlayingTask strikerPlayingTask {env};
    SupporterPlayingTask supporterPlayingTask {env};
  };

} // RE2019
} // CoroBehaviour2023