/**
 * @file DynamicBallHandlingTeamCoordination.h
 *
 * This task implements the team coordination behaviour for an updated form
 * (based on 2025 code) of the Robocup 2023 dynamic ball handling task. 
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/TeamBehaviour2025.h"

#include "Representations/Configuration/REFormations.h"
#include "Representations/Configuration/RETactics.h"

#include "Debugging/DebugDrawings.h"
#include "TextLogging/TextLogging.h"

namespace RE
{
namespace v2025 
{ 
namespace DBHC
{
  
  CRBEHAVIOUR(DynamicBallHandlingTeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(DynamicBallHandlingTeamCoordinationTask) {}

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
    LOADS_PARAMS(DynamicBallHandlingTeamCoordinationTask,
    {,
      // don't forget to set setupPoses.cfg to positions that robots are expected
      // to be at when placed on the field. KickoffPoses represents the poses
      // they'll try to get to as soon as they can move (if they are not trying
      // to get to the ball already)
      (std::vector<Pose2f>) kickoffPoses, // robots 1, 2, 3 in order
      (std::vector<REFormations::FormationRole>)/*({REFormations::centreBack, REFormations::midL, REFormations::midR})*/ formationRoles,
      (Rangef) xRange,
      (Rangef) yRange,
    });

    READS(FieldDimensions);
    READS(RETeamData);
    READS(FrameInfo);
    READS(BehaviorStatus);
    READS(RobotPose);
    // READS(REGameInfo);
    READS(GameState);
    READS(FieldBall);
    READS(MotionInfo);

    MODIFIES(TeamBehaviorStatus);
    MODIFIES(REActiveTactic);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};

    // members related to positioning the formation according to the ball.
    // We make these members so that we can refer to them in drawing code.
    Vector2f assumedBallPos;

    // enum { NO_PASS, PASS_LINE_UP, PASS_KICKED, PASS_SUBSTANTIAL } passState;
    // Vector2f ballPassStartOnField;
    // Vector2f ballPassTargetVector; // relative to start
    // unsigned lastKickTimestamp = 0; ///< used to track when a kick has actually been performed

    DECL_TLOGGER_VAR(tlogger, "DynamicBallHandlingTeamCoordination", TextLogging::INFO);


    void updateTeamInfoKickoff()
    {
      TLOGV(tlogger, "updateTeamInfoKickoff");

      updateActiveTacticCommon();

      theREActiveTactic.formationPoses = params.kickoffPoses;

      theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);
      theREActiveTactic.situationRoles[Settings::toPlayerIndex(1)] = PlayerRole::ballPlayer; // player 1 is initially the ball player at kickoff
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
      updateActiveTacticCommon();

      // if the team ball is well inside own half, then just go to kickoff poses
      if (theREActiveTactic.assumedBallPositionOnField.x() < -1000)
      {
        TLOGV(tlogger, "updateTeamInfoPlaying as for kickoff");

        theREActiveTactic.formationPoses = params.kickoffPoses;

        theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);
        theREActiveTactic.situationRoles[Settings::toPlayerIndex(1)] = PlayerRole::ballPlayer; // player 1 is initially the ball player at kickoff
      }
      else // we're into the main part of the game
      {
        TLOGV(tlogger, "updateTeamInfoPlaying really playing");
        
        // the formation poses are positioned from the ball. The ball player's
        // pose is at the ball (facing other player).
        // The receiver's pose is nominally slightly forward of the ball, but can
        // adjust depending on where opponents are
        // NOTE: that we only need to adjust poses for robots 2 (left) and 3 (right) at this point.

        // check which player the ball-y is closest to
        int indexP2 = Settings::toPlayerIndex(2);
        int indexP3 = Settings::toPlayerIndex(3);
        int ballPlayerIndex =
            std::abs(theREActiveTactic.formationPoses[indexP2].translation.y() - assumedBallPos.y()) <
                    std::abs(theREActiveTactic.formationPoses[indexP3].translation.y() - assumedBallPos.y())
                ? indexP2
                : indexP3;
        int receiverIndex = (ballPlayerIndex == indexP2) ? indexP3 : indexP2;
        int kickingSide = (ballPlayerIndex == indexP2) ? 1 : -1;

        theREActiveTactic.formationPoses[ballPlayerIndex] = Pose2f(
            kickingSide * -90_deg, assumedBallPos.x(), assumedBallPos.y() + kickingSide * 100.f); // 100mm behind ball

        float receiverX = assumedBallPos.x() + 200.f;
        // we need to interpolate the y position such that when x is 0, y is at its max value, 
        // and when x is its max value, then y is at its min value
        theREActiveTactic.formationPoses[receiverIndex] = Pose2f(
            kickingSide * 90_deg, receiverX, -kickingSide * params.yRange.inverseScale(receiverX, params.xRange));

        theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);
        theREActiveTactic.situationRoles[ballPlayerIndex] = PlayerRole::ballPlayer;
      }
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

      theREActiveTactic.formationRoles = params.formationRoles;
    }

    void updateAssumedBallPosition()
    {
      // FIXME - is some of this logic duplicated from FieldBall or TeamBall?
      // calc ball effect and key parameters for formation movement

      if (theGameState.isPlaying()) // FIXME add in special cases for penalties etc here
        assumedBallPos = Vector2f::Zero();
      else if (theFieldBall.timeSinceBallWasSeen < 2000) // FIXME param
        assumedBallPos = theFieldBall.endPositionOnField;
      else if (theFieldBall.teamBallIsValid)
        assumedBallPos = theFieldBall.teamEndPositionOnField;
      else if (theFieldBall.timeSinceBallWasSeen < 10000) // FIXME param
        assumedBallPos = theFieldBall.endPositionOnField; // fall back to our own old ball if no valid team ball
      else
        assumedBallPos = Vector2f::Zero(); // no ball seen anywhere recently

      theREActiveTactic.assumedBallPositionOnField = assumedBallPos;
    }


//     void updateTeammateRoles()
//     {
//       TeammateRoles teammateRoles;
//       const RETeammate* captainTeammate = teamBehaviourStatusSkills.selectCaptainTeammate();
// 
//       if(!captainTeammate)
//       {
//         teammateRoles.captain = theGameState.playerNumber;
//         teammateRoles.timestamp = theFrameInfo.time;
//         assignRoles(teammateRoles);
//       }
//       else
//       {
//         teammateRoles = captainTeammate->theTeamBehaviorStatus.teammateRoles;
//       }
// 
//       teamBehaviourStatusSkills.recordTeammateRoles(teammateRoles);
//     }
// 
//     // get time to ball depending on whether the supplied teamBehaviourStatus
//     // (from some particular robot) is striker or not
//     unsigned getTimeToReachBall(int playerNumber, const TeamBehaviorStatus& teamBehaviourStatus)
//     {
//       // the ball player or pass receiver have equal chance to be ball player going forward
//       // all other players have a penalty to prevent rapid oscillations between role assignments
//       // NOTE: this is called before new ballPlayer assignments are made so it is relying on the
//       // ballPlayer (=passKicker) and passReceiver from the last cycle
//       if (teamBehaviourStatus.role.isBallPlayer() || (teamBehaviourStatus.passStatus.passReceiver == playerNumber))
//         return teamBehaviourStatus.timeToReachBall.timeWhenReachBallStriker;
//       else
//         return teamBehaviourStatus.timeToReachBall.timeWhenReachBall;
//     }
// 
//     bool isGoalkeeper(int robotNumber)
//     {
//       return robotNumber == 1;
//     }
// 
//     void assignRoles(TeammateRoles& teammateRoles)
//     {   
//         using RobotNumberToTime = std::pair<int, unsigned>;
//         std::vector<RobotNumberToTime> timeToBall;
// 
//         teammateRoles[theGameState.playerNumber] = PlayerRole::none;
// 
//         if (!theREGameInfo.isPenalized() && isGoalkeeper(theGameState.playerNumber)) // are we the goalie?
//           teammateRoles[theGameState.playerNumber] = PlayerRole::goalkeeper;
//         else // no, perhaps a teammate is goalie
//         {
//           for (const RETeammate& teammate : theRETeamData.teammates)
//           {
//             if (!teammate.isPenalized && isGoalkeeper(teammate.number))
//             {
//               teammateRoles[teammate.number] = PlayerRole::goalkeeper;
//               TLOGD(tlogger, "robot{}: assignRoles goalkeeper is {}\n", theGameState.playerNumber, teammate.number);
//               break;
//             }
//           }
//         }
// 
//         // add in players to list
//         timeToBall.emplace_back(theGameState.playerNumber, getTimeToReachBall(theGameState.playerNumber, theTeamBehaviorStatus));
//         for (const RETeammate& teammate : theRETeamData.teammates)
//         {
//             timeToBall.emplace_back(teammate.number, getTimeToReachBall(teammate.number, teammate.theTeamBehaviorStatus));
//         }
// 
// 
//         // assigning striker based on which robot is closest to the ball
//         if (!timeToBall.empty())
//         {
//             // sort based on time to ball. (Hysteresis already accounted for in getTimeToReachBall)
//             std::sort(timeToBall.begin(), timeToBall.end(),
//                     [](auto &left, auto &right) { return left.second < right.second; });
// 
//             int strikerNumber = timeToBall.front().first;
// 
//             teammateRoles[strikerNumber] = PlayerRole::ballPlayer;
//         }
//         
//         // then assign supporters as the two other players
//         struct RobotNumber
//         {
//             int number;
//             int supporterIndex;
// 
//             RobotNumber(int number, int supporterIndex): number(number), supporterIndex(supporterIndex) {}
//         };
//         std::vector<RobotNumber> robotNumbersVec;
// 
//         // add the current player if they aren't the ball player
//         if(teammateRoles[theGameState.playerNumber] == PlayerRole::none) 
//         {
//             robotNumbersVec.emplace_back(theGameState.playerNumber, theTeamBehaviorStatus.role.supporterIndex());
//         }
// 
//         // add the other player that isn't the ball player
//         for (const RETeammate& teammate : theRETeamData.teammates)
//         {
//             if(teammateRoles[teammate.number] == PlayerRole::none)
//             {
//                 robotNumbersVec.emplace_back(teammate.number, teammate.theTeamBehaviorStatus.role.supporterIndex());
//             }
//         }
// 
//         // fill in the supporter roles
//         int supporterIndex = 0;
//         for (auto& robotNumber : robotNumbersVec)
//           teammateRoles[robotNumber.number] = PlayerRole::firstSupporterRole + supporterIndex++;
// 
//     }
// 
//     void updatePlayerRole()
//     {
//       PlayerRole role;
// 
//       // we assume that the TeammateRoles have already been updated by the time this runs
//       // so we can pull our values from there
// 
//       role.role = static_cast<PlayerRole::Type>(theTeamBehaviorStatus.teammateRoles[theGameState.playerNumber]);
//       role.numOfActiveSupporters = 0;
//       for (int teammateRole : theTeamBehaviorStatus.teammateRoles.roles)
//         if (teammateRole >= PlayerRole::firstSupporterRole)
//           ++role.numOfActiveSupporters;
// 
//       // CRB_ANNOTATION_FMT("playerRole: {}, numActiveSupporters={}\n", role.role, role.numOfActiveSupporters);      
// 
//       teamBehaviourStatusSkills.recordPlayerRole(role);
//     }
// 
//     void updatePassStatus()
//     {
//       REPassStatus passStatus;
// 
//       // at this point the ball player for this cycle has been updated and
//       // might be different to the ball player last cycle. The pass kicker
//       // (who might no longer be the ballPlayer) has the best ball visibility 
//       // and is in the best position to determine if a pass was made
// 
//       passStatus.numPasses = theTeamBehaviorStatus.passStatus.numPasses; 
// 
//       if ((passState == NO_PASS) && (theGameState.playerNumber == theTeamBehaviorStatus.passStatus.passKicker) &&
//           // ball is close to the robot
//           (theFieldBall.positionRelative.norm() < 250.f) &&
//           // ball is stopped or nearly stopped
//           ((theFieldBall.positionRelative - theFieldBall.endPositionRelative).norm() < 100.f))
//       {
//         passState = PASS_LINE_UP;
//         ballPassStartOnField = theFieldBall.positionOnField;
//       }
//       else if (passState == PASS_LINE_UP)
//       {
//         ballPassTargetVector = theBehaviorStatus.shootingTo - ballPassStartOnField;
//         
//         // has the ball moved during line up?
//         if (theMotionInfo.lastKickTimestamp != lastKickTimestamp)
//         {
//           lastKickTimestamp = theMotionInfo.lastKickTimestamp;
//           passState = PASS_KICKED;
//         }
//         else if ((theFieldBall.positionOnField - ballPassStartOnField).norm() > 150.f)
//           passState = NO_PASS;
//       }
//       else if (passState == PASS_KICKED)
//       {
//         Vector2f ballVector = theFieldBall.positionOnField - ballPassStartOnField;
//         Vector2f ballEndVector = theFieldBall.endPositionOnField - ballPassStartOnField;
//         
//         // was the ball kicked far enough in the general direction intended?
//         if ((ballVector.norm() > 1000.f) &&
//             (Angle(ballEndVector.angle()).diffAbs(ballPassTargetVector.angle()) < 15_deg))
//         {
//           // if (++passStatus.numPasses > 255)
//             // passStatus.numPasses = 0; // FIXME - change this to a sequence number?
//           passStatus.numPasses++;
//           passState = NO_PASS;
//         }
//       }
//       else
//       {
//         passState = NO_PASS;
//       }
// 
//       // set the actual numPasses to the largest value reported by any robot
//       for (const RETeammate& teammate : theRETeamData.teammates)
//       {
//         if (teammate.theTeamBehaviorStatus.passStatus.numPasses > passStatus.numPasses)
//           passStatus.numPasses = teammate.theTeamBehaviorStatus.passStatus.numPasses;
//       }
// 
// 
//       // update the pass kicker and receiver for this cycle
//       if (theTeamBehaviorStatus.role.isBallPlayer())
//       {
//         if (theBehaviorStatus.passTarget != -1)
//         {
//           passStatus.passKicker = theGameState.playerNumber;
//           passStatus.passReceiver = theBehaviorStatus.passTarget;
//         }
//       }
//       else
//       {
//         for (const RETeammate& teammate : theRETeamData.teammates)
//         {
//           if (theTeamBehaviorStatus.teammateRoles[teammate.number] == PlayerRole::ballPlayer)
//           {
//             if (teammate.theBehaviorStatus.passTarget != -1)
//             {
//               passStatus.passKicker = teammate.number;
//               passStatus.passReceiver = teammate.theBehaviorStatus.passTarget;
//             }
//             break;
//           }
//         }
//       }
//       
//       teamBehaviourStatusSkills.recordPassStatus(passStatus);
//     }



    /**
     * draw the formations decided by this team coordination behaviour
     */
    void draw()
    {
      DEBUG_DRAWING("behaviour:DynamicBallHandlingTeamCoordination", "drawingOnField")
      {
        ColorRGBA ballColor = ColorRGBA::black.lighter(0.25).alpha(0.5f);

        CIRCLE("behaviour:DynamicBallHandlingTeamCoordination", assumedBallPos.x(), assumedBallPos.y(), 120, 30, Drawings::solidPen,
              ballColor, Drawings::noBrush, ColorRGBA::black);

        ColorRGBA posColor = ColorRGBA::black.alpha(0.5f);

        for (unsigned i = 0; i < theREActiveTactic.formationPoses.size(); i++)
        {
          const Pose2f& pose = theREActiveTactic.formationPoses[i];
          int playerNumber = Settings::toPlayerNumber(i);

          CIRCLE("behaviour:DynamicBallHandlingTeamCoordination", pose.translation.x(), pose.translation.y(), 150, 30,
                  Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

          if (playerNumber == theGameState.playerNumber) // self?          
            CIRCLE("behaviour:DynamicBallHandlingTeamCoordination", pose.translation.x(), pose.translation.y(), 220, 30,
                    Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

          DRAW_TEXT("behaviour:DynamicBallHandlingTeamCoordination", pose.translation.x()-75, pose.translation.y()+220, 150,
                    posColor, playerNumber);

          Vector2f directionVec = pose * Vector2f(350, 0);

          ARROW("behaviour:DynamicBallHandlingTeamCoordination", pose.translation.x(), pose.translation.y(), directionVec.x(),
                directionVec.y(), 30, Drawings::solidPen, posColor);

          ++playerNumber;
        }
      }
    }


  };

} // DBHC
} // namespace v2025
} // namespace RE