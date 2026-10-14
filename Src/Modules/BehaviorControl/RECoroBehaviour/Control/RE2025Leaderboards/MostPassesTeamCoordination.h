/**
 * @file MostPassesTeamCoordination.h
 *
 * This task implements the team coordination behaviour for the RC2025
 * most passes leaderboard challenge.
 * 
 * Note the participating robots must be numbered 1 and 2.
 * 
 * It sets the formation poses for the 2 players and chooses which should be
 * ball player (with the other then being the receiver).
 * 
 * The general strategy is:
 * - kick back and forth to move up the field fairly quickly
 * - then once within a margin of the end zone (with a margin of time
 *   remaining), just pass straight back and forth
 * - when the time margin expires, the receiver goes to the end zone
 *   to receive the final pass
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/TeamBehaviour2025.h"

#include "Modules/BehaviorControl/RECoroBehaviour/Skills/TeamBehaviorStatusSkills.h"

#include "Representations/Configuration/REFormations.h"
#include "Representations/Configuration/RETactics.h"

#include "Debugging/DebugDrawings.h"
#include "TextLogging/TextLogging.h"
#include "Tools/RECommon/FmtCommonTypes.h"

namespace RE
{
namespace v2025
{
namespace Leaderboards
{
  
  CRBEHAVIOUR(MostPassesTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(MostPassesTeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        // NOTE: we completely ignore set plays

        if (theGameState.isInitial(/*orStandby*/ true) || theGameState.isReady() || theGameState.isSet())
        {
          CR_CHECKPOINT(initial_standby_ready_set);
          updateTeamInfoKickoff();
        }
        else if (theGameState.isPlaying())
        {
          CR_CHECKPOINT(normal_play);
          updateTeamInfoPlaying();
        }          

        updateOwnTacticInfo(); // set own roles based on team info
        draw();
        
        CR_YIELD();
      }
    }

  private:
    LOADS_PARAMS(MostPassesTeamCoordinationTask,
    {,      
      // don't forget to set setupPoses.cfg to positions that robots are expected to start
      (float) challengeDurationSecs,
      (float) safetySecsToEndZone, // target amount of time remaining when receiver gets final pass in target zone
      (Vector2f) insideEndZoneMargin, // mm - distance outside end zone that we'll switch to inside and be inside by this margin (to avoid mistakes)
      (float) passStraightOutsideEndZoneMargin, // mm - distance outside end zone that we'll switch to passing straight back and forth while time is OK
      (float) exclusionZoneMargin, // mm - distance outside exclusion zone that players should be positioned
      (float) xAdvancePerPass, // mm
      (float) teammateWalkSpeed, // mm/s - conservative to allow for alignment etc.
    });

    READS(FieldDimensions);
    READS(RETeamData);
    // READS(FrameInfo);
    READS(BehaviorStatus);
    READS(RobotPose);
    // READS(REGameInfo);
    READS(ExtendedGameState);
    READS(GameState);
    READS(FrameInfo);
    READS(FieldBall);
    // READS(MotionInfo);

    MODIFIES(TeamBehaviorStatus);
    MODIFIES(REActiveTactic);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};


    DECL_TLOGGER_VAR(tlogger, "MostPassesTeamCoordinationTask", TextLogging::DEBUG);

    // members related to positioning the formation according to the ball.
    // We make these members so that we can refer to them in drawing code.
    Vector2f assumedBallPos;
    bool assumedBallPosValid = false; // whether the assumed ball position is valid
    // Vector2f lastReceiverPosOnField {NAN,NAN}; // track the last pass pose as reference for next formation pose
    // int receiverIndexLastFrame = -1;

    // enum { NO_PASS, PASS_LINE_UP, PASS_KICKED, PASS_SUBSTANTIAL } passState;
    // Vector2f ballPassStartOnField;
    // Vector2f ballPassTargetVector; // relative to start
    // unsigned lastKickTimestamp = 0; ///< used to track when a kick has actually been performed

    void updateTeamInfoKickoff()
    {
      TLOGV(tlogger, "updateTeamInfoKickoff");

      updateActiveTacticCommon();

      // if (theREGameInfo.state != STATE_PLAYING)
      {
        theREActiveTactic.formationPoses = {
            Pose2f(-90_deg, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftTouchline),
            Pose2f(90_deg, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightTouchline),
            Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, 0),
            Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, 300),
            Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, -300)};
      }
      // else
      // {
      //   // robots 1 to 5 - only 1 and 2 are active
      //   theREActiveTactic.formationPoses = {
      //       // Pose2f(90_deg, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightTouchline),
      //       // Pose2f(-90_deg, theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftTouchline),
      //       Pose2f(-90_deg, theFieldDimensions.xPosOwnPenaltyArea,
      //              theFieldDimensions.centerCircleRadius + params.exclusionZoneMargin),
      //       Pose2f(90_deg, theFieldDimensions.xPosOwnPenaltyArea,
      //              -theFieldDimensions.centerCircleRadius - params.exclusionZoneMargin),
      //       Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, 0),
      //       Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, 300),
      //       Pose2f(0, theFieldDimensions.xPosOpponentGoalLine, -300)};
      // }
      theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);
      // theREActiveTactic.situationRoles[Settings::toPlayerIndex(1)] = PlayerRole::ballPlayer; // player 1 is initially the ball player at kickoff
    }

    /**
     * This is the main function to implement the team tactics for the DBHC.
     * - Player 1 chooses player 2 or 3 at random for the first pass, so initially
     *   they just get into a position to receive the pass.
     * - Thereafter, whichever player has the ball aims to pass it to the other
     *   while hopefully making progress up the field. The receiver should aim
     *   to be in a position to receive the ball without being intercepted.
     * - Once the 3rd pass has been completed, the ball player can shoot on
     *   goal.
     */
    void updateTeamInfoPlaying()
    {
      TLOGV(tlogger, "updateTeamInfoPlaying");
      
      updateActiveTacticCommon();

      // we need a valid ball to do anything much useful
      if (!assumedBallPosValid)
        return; // leave everything as it was

      // OK, so we have a valid ball and can make some decisions based on it

      // the formation poses are positioned from the ball. The ball player's
      // pose is at the ball (facing other player).
      // The receiver's pose is nominally slightly forward of the ball
      // NOTE: that we only need to adjust poses for robots 2 (left) and 3 (right) at this point.

      // check which player the ball-y is closest to
      int indexP1 = Settings::toPlayerIndex(1);
      int indexP2 = Settings::toPlayerIndex(2);

      int prevBallPlayerIndex = (theREActiveTactic.situationRoles[indexP1] == PlayerRole::ballPlayer)   ? indexP1
                                : (theREActiveTactic.situationRoles[indexP2] == PlayerRole::ballPlayer) ? indexP2
                                                                                                      : -1;

      // apply hysteresis when switching ball player (extra cost to player that wasn't prev ball player)
      float stabilityCostP1 = prevBallPlayerIndex == indexP1 ? 0 : 1000.f;
      float stabilityCostP2 = prevBallPlayerIndex == indexP2 ? 0 : 1000.f;
      float ballToP2 = std::abs(theREActiveTactic.formationPoses[indexP2].translation.y() - assumedBallPos.y());
      float ballToP1 = std::abs(theREActiveTactic.formationPoses[indexP1].translation.y() - assumedBallPos.y());
      int ballPlayerIndex = (ballToP1 + stabilityCostP1) < (ballToP2 + stabilityCostP2) ? indexP1 : indexP2;
      int receiverIndex = (ballPlayerIndex == indexP1) ? indexP2 : indexP1;
      int ballPlayerSide = (ballPlayerIndex == indexP1) ? 1 : -1; // player 1 is on the LHS, so if ball player then kicking side is LHS also
      int receiverSide = -ballPlayerSide;


      // handle pose changes if the ball player index has changed (because the ball swapped sides)
      if (ballPlayerIndex != prevBallPlayerIndex)
      {
        TLOGD(tlogger, "ballPlayerIndex changed from {} to {}", prevBallPlayerIndex, ballPlayerIndex);

        // ball player pose adn receiver post

        float playerY = theFieldDimensions.centerCircleRadius + params.exclusionZoneMargin; // for left side

        if (prevBallPlayerIndex == -1) // no prev ball player => no prev pos to work from
        {
          theREActiveTactic.formationPoses[ballPlayerIndex] = Pose2f(
              ballPlayerSide * -90_deg, theFieldDimensions.xPosOwnPenaltyArea, ballPlayerSide * playerY); // nominal starting ball pos

          theREActiveTactic.formationPoses[receiverIndex] =
              Pose2f(receiverSide * -90_deg, theFieldDimensions.xPosOwnPenaltyArea + params.xAdvancePerPass, receiverSide * playerY);
        }
        else // prevBallPlayerIndex was valid so we can work from the previous pose
        {
          int prevReceiverIndex = prevBallPlayerIndex == indexP2 ? indexP1 : indexP2;

          Vector2f lastBallPlayerPosOnField = theREActiveTactic.formationPoses[prevBallPlayerIndex].translation;
          Vector2f lastReceiverPosOnField = theREActiveTactic.formationPoses[prevReceiverIndex].translation;;

          // update ball player pose
          // new ball player was previous receiver, so start from old receiver pos

          theREActiveTactic.formationPoses[ballPlayerIndex] = Pose2f(ballPlayerSide * -90_deg, lastReceiverPosOnField.x(),
                                                                   ballPlayerSide * playerY); // nominal starting ball pos
          
          // update receiver pose
          // new receiver was previous ball player, but it's new pose should be based off the current
          // ball player pose


          // estimate receiver distance to end zone based on last formation pose (rather than actual position) so
          // that both teammates are using the same info
          Vector2f endZoneTargetOnField(theFieldDimensions.xPosOpponentPenaltyArea + params.insideEndZoneMargin.x(),
                                        receiverSide * (theFieldDimensions.yPosLeftPenaltyArea - params.insideEndZoneMargin.y()));

          Vector2f newReceiverPrevPosOnField = theREActiveTactic.formationPoses[receiverIndex].translation;
          float secsToFinalReceiverZone = (endZoneTargetOnField - newReceiverPrevPosOnField).norm() / params.teammateWalkSpeed;
          float secsRemaining = params.challengeDurationSecs - (theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted) / 1000.f);
          bool receiverTimeToEndZoneOK = (secsToFinalReceiverZone < (secsRemaining - params.safetySecsToEndZone));

          // TLOGD(tlogger, "secs to final zone {:.1f}, secs remaining less margin {:.1f}", secsToFinalReceiverZone,
          //       secsRemaining - params.safetySecsToEndZone);

          if (!receiverTimeToEndZoneOK)
            theREActiveTactic.formationPoses[receiverIndex] = Pose2f(receiverSide * -90_deg, endZoneTargetOnField);
          else // time is OK
          {
            const float passStraightZoneThresholdX =
                (theFieldDimensions.xPosOpponentPenaltyArea - params.passStraightOutsideEndZoneMargin);

            // initially set up in line with the ball player
            float receiverX = theREActiveTactic.formationPoses[ballPlayerIndex].translation.x();

            // now advance if that makes sense
            if (receiverX < passStraightZoneThresholdX)
              receiverX += std::min(params.xAdvancePerPass, passStraightZoneThresholdX - receiverX);

            theREActiveTactic.formationPoses[receiverIndex] =
                Pose2f(receiverSide * -90_deg, receiverX, receiverSide * playerY);
          }
        }

        // lastBallPlayerPosOnField = theREActiveTactic.formationPoses[ballPlayerIndex].translation;
        // lastReceiverPosOnField = theREActiveTactic.formationPoses[ballPlayerIndex].translation;
      }

      // roles
      theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);
      theREActiveTactic.situationRoles[ballPlayerIndex] = PlayerRole::ballPlayer;
    }


    int getSide(Vector2f pos)
    {
      return pos.y() > 0 ? 1 : -1;
    }


    /**
     * populate REActiveTactic own individual info and teamBehaviourStatus
     */
    void updateOwnTacticInfo()
    {
      int i = theGameState.playerIndex();

      theREActiveTactic.formationRole = theREActiveTactic.formationRoles[i];
      theREActiveTactic.situationRole = theREActiveTactic.situationRoles[i];

      theREActiveTactic.formationPose = theREActiveTactic.formationPoses[i];

      // populate teamBehaviourStatus (almost a legacy representation at this point)...

      PlayerRole pr;
      pr.role = theREActiveTactic.situationRole;
      pr.numOfActiveSupporters = 0; // we don't use this any more

      teamBehaviourStatusSkills.recordPlayerRole(pr);
      // default (meaningless) values for all of these since we don't use them from 2024 on
      // teamBehaviourStatusSkills.recordTeamActivity(TeamBehaviorStatus::noTeam);
      teamBehaviourStatusSkills.recordTimeToReachBall(TimeToReachBall());
      teamBehaviourStatusSkills.recordTeammateRoles(TeammateRoles());
    }


    void updateActiveTacticCommon()
    {
      theREActiveTactic.formationId = REFormations::customFormation;

      theREActiveTactic.attacking = true;
      updateAssumedBallPosition();

      theREActiveTactic.formationRoles = { REFormations::centreBack, REFormations::midR, REFormations::midL };
    }


    void updateAssumedBallPosition()
    {
      bool prevValid = assumedBallPosValid;

      // FIXME - is some of this logic duplicated from FieldBall or TeamBall?
      // calc ball effect and key parameters for formation movement

      assumedBallPosValid = true;

      if (theGameState.isPlaying()) // FIXME add in special cases for penalties etc here
      {
        assumedBallPos = Vector2f::Zero();
        assumedBallPosValid = false;
      }
      else if (theFieldBall.timeSinceBallWasSeen < 2000) // FIXME param
        assumedBallPos = theFieldBall.endPositionOnField;
      // else if (theFieldBall.timeSinceTeamBallWasValid < 1000)
      else if (theFieldBall.teamBallIsValid)
        assumedBallPos = theFieldBall.teamEndPositionOnField;
      else //if (theFieldBall.timeSinceBallWasSeen < 10000) // FIXME param
      {
        assumedBallPos = theFieldBall.endPositionOnField; // fall back to our own old ball if no valid team ball
        assumedBallPosValid = false;
      }
      // else
      //   assumedBallPos = Vector2f::Zero(); // no ball seen anywhere recently

      theREActiveTactic.assumedBallPositionOnField = assumedBallPos;

      if (assumedBallPosValid != prevValid)
        TLOGI(tlogger, "updateAssumedBallPosition: valid={}, pos={}", assumedBallPosValid, assumedBallPos);
    }




    /**
     * draw the formations decided by this team coordination behaviour
     */
    void draw()
    {
      // all in active tactic now
      DEBUG_DRAWING("behaviour:MostPassesTeamCoordination", "drawingOnField")
      {
//         // ColorRGBA ballColor = ColorRGBA::black.lighter(0.25).alpha(0.5f);
//         ColorRGBA ballColor = ColorRGBA::red.lighter(0.25).alpha(0.5f);
// 
//         CIRCLE("behaviour:MostPassesTeamCoordination", assumedBallPos.x(), assumedBallPos.y(), 120, 50, Drawings::solidPen,
//               ballColor, Drawings::noBrush, ColorRGBA::black);
// 
//         ColorRGBA posColor = ColorRGBA::black.alpha(0.5f);
// 
//         for (unsigned i = 0; i < theREActiveTactic.formationPoses.size(); i++)
//         {
//           const Pose2f& pose = theREActiveTactic.formationPoses[i];
//           int playerNumber = Settings::toPlayerNumber(i);
// 
//           CIRCLE("behaviour:MostPassesTeamCoordination", pose.translation.x(), pose.translation.y(), 150, 30,
//                   Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);
// 
//           if (playerNumber == theGameState.playerNumber) // self?          
//             CIRCLE("behaviour:MostPassesTeamCoordination", pose.translation.x(), pose.translation.y(), 220, 50,
//                     Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);
// 
//           DRAW_TEXT("behaviour:MostPassesTeamCoordination", pose.translation.x()-75, pose.translation.y()+220, 150,
//                     posColor, playerNumber);
// 
//           Vector2f directionVec = pose * Vector2f(350, 0);
// 
//           ARROW("behaviour:MostPassesTeamCoordination", pose.translation.x(), pose.translation.y(), directionVec.x(),
//                 directionVec.y(), 30, Drawings::solidPen, posColor);
// 
//           ++playerNumber;
//         }
      }
    }


  };

} // Leaderboards
} // namespace v2025
} // namespace RE