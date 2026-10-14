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

#include "Modules/BehaviorControl/RECoroBehaviour/2025/CoroBehaviour2025.h"

#include "Striker.h"
#include "Goalie.h"
#include "Supporter.h"
#include "PenaltyShotStriker.h"
#include "PenaltyShotGoalie.h"
#include "PenaltyShotOther.h"
#include "SetPlay.h"
// #include "RLDefenderNeuralControl.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/CommonSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/HeadSkills.h"

#include "Math/Pose2f.h"


namespace RE
{
namespace v2025
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
        if ((theGameState.isPenaltyShootout()) || (theGameState.isPenaltyKick()))
        {
          if (commonSkills.isOurTeamKick() && theTeamBehaviorStatus.role.isBallPlayer())
            penaltyShotStrikerPlayingTask();
          else if (!commonSkills.isOurTeamKick() && theGameState.isGoalkeeper())
            penaltyShotGoaliePlayingTask();
          else
            penaltyShotOtherPlayingTask();
        }
        else if (theGameState.isFreeKick()) // other set plays - TODO: not implemented yet
        {
          // if (theExtendedGameInfo.timeSinceFreeKickStarted > 0 && theExtendedGameInfo.timeSinceFreeKickStarted < 7000) // simrobot hack DO NOT USE
          // if (commonSkills.isNoTeamKick() && !theTeamBehaviorStatus.role.isGoalkeeper() &&
          //     // theExtendedGameInfo.timeSinceFreeKickStarted < 7000) // wont work in SimRobot
          //     theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) < 7000)
          // {
          //   // get the visual referee to signal the set play
          //   setPlayLookAtRefereeTask();
          // }
          if (theTeamBehaviorStatus.role.isBallPlayer())
          {
            if (commonSkills.isOurTeamKick())
            {
              if (theGameState.isCornerKick())
                setPlayStrikerCornerKickTask();
              else if (theGameState.isFreeKick() && !theGameState.isGoalKick())
                setPlayStrikerFreeKickTask();
              else // goal kick
                setPlaySupporterPlayingTask(); // yes - we use supporter task just to back off a bit
            }
            else // not our kick so defending
              setPlaySupporterPlayingTask();
          }
          else if (theTeamBehaviorStatus.role.isGoalkeeper())
          {
            if ((theGameState.isGoalKick()) && commonSkills.isOurTeamKick())
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
            goaliePlayingTask(); // RL Goalkeeper task
          else if (theTeamBehaviorStatus.role.isBallPlayer())
            strikerPlayingTask();
            
          // RL DEFENDER BLOCK DONT USE UNLESS 7v7
          // else if (theREActiveTactic.formationRole == REFormations::FormationRole::centreBack || 
          //             theREActiveTactic.formationRole == REFormations::FormationRole::centreBackL || 
          //             theREActiveTactic.formationRole == REFormations::FormationRole::centreBackR)
          // {
          //   // if the ball was seen and should be playing as a defender
          //   // if (theFieldBall.ballWasSeen()) // FIXME - no RL behaviour for now
          //   //   defenderNeuralControlTask();
          //   // else
          //     supporterPlayingTask();
          // }
          else
            supporterPlayingTask();
        }
        
        CR_YIELD();
      }
    }

  private:
    // READS(REGameInfo);
    // READS(ExtendedGameInfo);
    READS(GameState);
    READS(FrameInfo);
    READS(TeamBehaviorStatus);
    READS(FieldBall);
    READS(REActiveTactic);
    
    CommonSkills commonSkills {env};

    PenaltyShotStrikerPlayingTask penaltyShotStrikerPlayingTask {env};
    PenaltyShotGoaliePlayingTask penaltyShotGoaliePlayingTask {env};
    PenaltyShotOtherPlayingTask penaltyShotOtherPlayingTask {env};

    SetPlayStrikerFreeKickTask setPlayStrikerFreeKickTask {env};
    SetPlayStrikerCornerKickTask setPlayStrikerCornerKickTask {env};
    SetPlaySupporterPlayingTask setPlaySupporterPlayingTask {env};

    // Visual Referee
    SetPlayLookAtRefereeTask setPlayLookAtRefereeTask {env};

    GoaliePlayingTask goaliePlayingTask {env};
    StrikerPlayingTask strikerPlayingTask {env};
    SupporterPlayingTask supporterPlayingTask {env};

    // RLDefenderNeuralControlTask defenderNeuralControlTask {env}; // FIXME disabled for now
  };

} // namespace v2025
} // namespace RE