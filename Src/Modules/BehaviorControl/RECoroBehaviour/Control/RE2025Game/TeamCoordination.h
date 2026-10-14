/**
 * @file TeamCoordination.h
 *
 * This task implements a minimal team coordination behaviour that doesn't adapt
 * to game situations much.
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/RECoroBehaviour/2025/TeamBehaviour2025.h"

// #include "Modules/BehaviorControl/RECoroBehaviour/ActiveTacticSkills.h"
#include "Modules/BehaviorControl/RECoroBehaviour/Skills/TeamBehaviorStatusSkills.h"
#include "Representations/Configuration/REFormations.h"
#include "Representations/Configuration/REGameConfig.h"
#include "Representations/Configuration/RETactics.h"

#include "Debugging/DebugDrawings.h"
#include "TextLogging/TextLogging.h"
#include "Tools/RECommon/FmtCommonTypes.h"

namespace RE
{
namespace v2025
{
  // =================================================================================================================

  CRBEHAVIOUR(TeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(TeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if (theGameState.isInitial(/*orStandby:*/ true))
        {
          CR_CHECKPOINT(initial_or_standby);
          updateKickoffSetupTeamInfo();
        }
        else if (!theGameState.isFreeKick())
        {
          if (theGameState.isReady() || theGameState.isSet() || theGameState.isKickOff())
          {
            CR_CHECKPOINT(prepare_for_kickoff);
            updateKickoffSetupTeamInfo();
          }
          else if (theGameState.isPlaying())
          {
            CR_CHECKPOINT(normal_play);
            // TODO deal with kickoff
            // else
            updatePlayingTeamInfo();
          }          
        }
        else
        {
          CR_CHECKPOINT(set_play);
          // FIXME - deal with set plays etc
          updatePassiveTeamInfo();
        }

        updateOwnTacticInfo(); // set own roles based on team info
        draw();
        
        CR_YIELD();
      }
    }

  private:
    TextLogger& tlogger = TextLogging::get("TeamCoordinationTask", TextLogging::INFO);

    DEFINES_PARAMS(TeamCoordinationTask, 
    {,
      (unsigned)(2000) ballPlayerBonusMs, ///< any non-ballPlayer needs to reach the ball this much quicker to replace the ballPlayer
      (unsigned)(4000) getUpMs, ///< it takes about this long to get up after falling
      (int)(8000) afterPenaltyMs, ///< don't allow striker for at least this delay after return from penalty
      // (float)(500.f) supporterHysteresis, ///< supporters must change x-value by this relative to each other to switch index

      (float)(200.f) maxGoalkeeperY, ///< goal keeper not allowed to move further left/right than this by default
      (Angle)(20_deg) ballAngleSplitUpfield, ///< max angle between look direction and ball when splitting look between upfield and the ball
      (Angle)(150_deg) ballAngleLookDirect, ///< above this angle we don't split anymore but look directly at the ball

      (float)(0.7f) walkSpeedFudgeFactor, ///< correction to max walking speed needed (at least in Sim) for ball player assignment

      (Rangef)(-1000.f, 1000.f) attackingHysteresis, // the lower and upper thresholds for switching attack to defense
    });


    READS(FieldDimensions);
    READS(RobotDimensions);
    READS(REFormations);
    READS(RETactics);
    READS(REGameConfig);
    READS(RETeamData);
    READS(FrameInfo);
    READS(RobotPose);
    // READS(REGameInfo);
    READS(GameState);
    READS(ExtendedGameState);
    READS(FieldBall);
    READS(WalkingEngineOutput);
    READS(RETeammatesLocationModel);
    READS(RESentTeamMessage2025);
    READS(BehaviorStatus);
//    READS(FallDownState);
//    READS(GroundContactState);
//    READS(MotionInfo);
//    READS(MotionRequest);

    MODIFIES(TeamBehaviorStatus);
    MODIFIES(REActiveTactic);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};
    // TeamPlaceholderTask teamKickoffTask {env, "TeamKickoffPlaceholder"};
    // TeamPlaceholderTask teamKickInTask {env, "TeamKickInPlaceholder"};

    // REFormations::FormationId prevFrameFormationId = REFormations::singlePlayerFormation;
    // int numFormationRoles = 1; // initially
    // unsigned prevFrameNumActivePlayers = 1;

    std::vector<REFormations::FormationRole> prevFramePlayerIndexToFormationRole;


    // members related to positioning the formation according to the ball.
    // We make these members so that we can refer to them in drawing code.
    Vector2f assumedBallPos;
    Vector2f clampedBallPos;
    Vector2f scaledBallPos;
    Vector2f shiftedAnchor;

    unsigned numKickingRobotsSinceLastSetPlay = 0;

    /**
     * set up the team according to the appropriate kickoff formation.
     * 
     * The formation does not depend on the ball location in this case (we assume the center spot).
     * Although there isn't any need for a ball player or striker yet, we nominate one
     * because they will be responsible for team comms and will be expected to be
     * the ball player after kickoff
     */
    void updateKickoffSetupTeamInfo()
    {
      if (theGameState.isForOwnTeam())
        updateTeamTacticInfo(theRETactics.ourKickoffFormation, /* useBall */ false);
      else
        updateTeamTacticInfo(theRETactics.opponentKickoffFormation, /* useBall */ false);
    }

    void updatePassiveTeamInfo()
    {
      updateTeamTacticInfo(theRETactics.defendingFormation, /* useBall */ true);
    }

    void updatePlayingTeamInfo()
    {
      // TODO - tmp hack
      if (theREActiveTactic.attacking)
        updateTeamTacticInfo(theRETactics.attackingFormation, /* useBall */ true);
      else
        updateTeamTacticInfo(theRETactics.defendingFormation, /* useBall */ true);
    }


    void updateTeamTacticInfo(const RETactics::Formation &tacticsFormation, bool useBall)
    {
      theREActiveTactic.formationId = findBestFormation(tacticsFormation.formationIds);

      updateAssumedBall();

      theREActiveTactic.attacking = useBall ? checkIfAttacking() : true; // FIXME - not really implemented yet

      const REFormations::Formation &formation = theREFormations.formations[theREActiveTactic.formationId];
      std::vector<Vector2f> formationPositionsOnField = formation.positions;

      if (useBall)
        // modify formations based on ball effect
        updateBallEffect(tacticsFormation, formationPositionsOnField);
      else
      {
        assumedBallPos = clampedBallPos = scaledBallPos = theREActiveTactic.assumedBallPositionOnField = Vector2f::Zero();
        shiftedAnchor = Vector2f(theREFormations.formations[theREActiveTactic.formationId].xAnchor, 0);
      }

      // correct the goalie position
      limitGoalkeeperPosition(formationPositionsOnField[0]);
      // TODO limit other positions to sensible on field positions, limit number of defenders in the box, etc.

      // populate the poses
      theREActiveTactic.formationPoses.clear();
      for (const Vector2f& pos : formationPositionsOnField)
      {
        theREActiveTactic.formationPoses.push_back(getFormationPose(pos, theREActiveTactic.assumedBallPositionOnField));
      }

      // populate the formation roles corresponding to each pose
      theREActiveTactic.formationRoles = formation.roles;

      // indicate which players will play in which formation position (only changes when the formation changes
      // of the number of players changes)
 //     updatePlayerFormationRoleAssignments();

      // assign situation roles
      // std::vector<PlayerRole::Type> prevSituationRoles = theREActiveTactic.situationRoles;
      // prevSituationRoles.resize(theREActiveTactic.formationRoles.size(), PlayerRole::none);
      // reset the existing situation roles before reassigning
      theREActiveTactic.situationRoles.assign(theREActiveTactic.formationRoles.size(), PlayerRole::none);

      assignNormalGoalkeeper();
      assignBallPlayer(useBall);
      assignSupporterIndices();
    }


    /**
     * Assign the "best" formation role to each player
     */
    void updatePlayerFormationRoleAssignments()
    {
      // When this function is called, the formationId, formationRoles, and formationPoses of theREActiveTactic
      // have already been updated to their latest values.
      //
      // Our objective is to update playerFormationRoleAssignments to the latest based on our memory
      // of previous assignments and the currently available players.

//       if (prevFramePlayerIndexToFormationRole.size() < theREGameInfo.playersPerTeam)
//         prevFramePlayerIndexToFormationRole.resize(theREGameInfo.playersPerTeam, REFormations::noFormationRole);
// 
//       size_t numActivePlayers = theRETeamData.activeTeammates.size() + (theGameState.isPenalized() ? 0 : 1);
// 
//       // has the formation or number of players changed? Nothing to do if not.
//       if ((prevFrameFormationId == theREActiveTactic.formationId) && (prevFrameNumActivePlayers == numActivePlayers))
//         return;
// 
//       // make it easy to get the index position of each formationRole and corresponding position
//       std::vector<int> formationRoleToIndex{REFormations::numFormationRole, -1};
//       for (unsigned i = 0; i < theREActiveTactic.formationRoles.size(); i++)
//         formationRoleToIndex[theREActiveTactic.formationRoles[i]] = i;
// 
// 
//       constexpr int NO_PLAYER = 0;
//       std::vector<int> unassignedTeammateNumbers;
//       theREActiveTactic.playerNumbers.assign(theREActiveTactic.formationRoles.size(), NO_PLAYER); // not yet assigned since 0 is not a valid player number
// 
//       // try to assign players based on their previous roles.
//       for (RETeammate* pTeammate : theRETeamData.activeTeammates)
//       {
//         REFormations::FormationRole fr = prevFramePlayerIndexToFormationRole[pTeammate->playerIndex()];
//         if ((fr != REFormations::noFormationRole) && (formationRoleToIndex[fr] != -1))
//           theREActiveTactic.playerNumbers[formationRoleToIndex[fr]] = pTeammate->number; // mark formationRole assigned in current frame
//         else
//           unassignedTeammateNumbers.push_back(pTeammate->number); // teammate is currently unassigned
//       }
// 
//       // which formation roles are unassigned now?
//       std::vector<int> unassignedFormationRoleIndexes;
// 
//       for (unsigned i=0; i<theREActiveTactic.playerNumbers.size(); i++)
//       {
//         if (theREActiveTactic.playerNumbers[i] == NO_PLAYER)
//           unassignedFormationRoleIndexes.push_back(i); // the corresponding formation role (and position etc) are unassigned
//       }
// 
//       // create a matrix of costs based on route each unassigned teammate must take
//       // to get to formation position. It is indexed by corresponding index of
//       // the unassigned teammates vector
//       // Row index specifies which teammate and column index specifies which formationRole
//       Eigen::MatrixXf costs(unassignedTeammateNumbers.size(), unassignedFormationRoleIndexes.size());
// 
//       for (unsigned iUTN = 0; iUTN < static_cast<unsigned>(costs.rows()); iUTN++)
//       {
//         for (unsigned iUFRI = 0; iUFRI < costs.cols(); iUFRI++)
//         {
//           const int playerNumber = unassignedTeammateNumbers[iUTN];
//           Pose2f playerPose = theRETeammatesLocationModel.locationsByNumber[playerNumber]->pose;
// 
//           const int formationRoleIndex = unassignedFormationRoleIndexes[iUFRI];
//           Pose2f formationPose = theREActiveTactic.formationPoses[formationRoleIndex];
// 
//           float cost = (playerPose.translation - formationPose.translation)
//                            .squaredNorm(); // TODO make this based on time and incorporate angles?
// 
//           costs(iUTN, iUFRI) = cost;
//         }
//       }
// 
//       // finally, try to assign unassigned players based on best match to available roles/positions
//       // initially, a first past the post method
//       // TODO: for the sake of making progress we'll take the default order of roles
// 
//       for (unsigned iUFRI = 0; iUFRI < costs.cols(); iUFRI++)
//       {
//         Eigen::Index minCostIndex;
//         costs.col(iUFRI).minCoeff(&minCostIndex); // find the row in this col which has minimum cost
// 
//         // assign the player
//         int playerNumber = unassignedTeammateNumbers[minCostIndex];
//         theREActiveTactic.playerNumbers[unassignedFormationRoleIndexes[iUFRI]] = playerNumber; // now it is assigned
// 
//         // ensure this player cannot be assigned a second time (in case of ties)
//         costs.row(minCostIndex).setConstant(sqr(10000.f));
//       }
// 
//       // save prevFramePlayerIndexToFormationRole
//       prevFramePlayerIndexToFormationRole.assign(theREGameInfo.playersPerTeam, REFormations::noFormationRole);
//       for (unsigned iPN = 0; iPN < theREActiveTactic.playerNumbers.size(); iPN++)
//       {
//         const int playerNumber = theREActiveTactic.playerNumbers[iPN];
//         const REFormations::FormationRole formationRole = theREActiveTactic.formationRoles[iPN];
//         prevFramePlayerIndexToFormationRole[Settings::toPlayerIndex(playerNumber)] = formationRole;
//       }
    }

    void updateAssumedBall()
    {
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

    bool checkIfAttacking()
    {
      // possible future logic:
      // We are attacking if...
      // - the ball is far on the attacking half
      // - our ball player kicked the ball recently and the ball is not heading
      //   towards the defending half (or hasn't changed direction since it last stopped?)
      //   This needs care for rebounds off of competing robots.
      // - the ball is nearer to our ball player than to any opposition robot.
      //   Needs care if the ballPlayer assignment is changing in this cycle
      // Otherwise we are defending?

      // FIXME - hack while early developing, needs logic to work out if we are in attack
      // implements basic hysteresis
      if (theREActiveTactic.attacking && (assumedBallPos.x() < params.attackingHysteresis.min))
        return false; // defending
      else if (!theREActiveTactic.attacking && (assumedBallPos.x() > params.attackingHysteresis.max))
        return true;

      return theREActiveTactic.attacking; // leave as-is
    }

    void updateBallEffect(const RETactics::Formation &tacticsFormation, std::vector<Vector2f> &formationPositionsOnField)
    {
      // calc ball effect and key parameters for formation movement
      // assumedBallPos has already been updated

      // clamp the ball pos according to the ballEffect
      clampedBallPos = Vector2f(tacticsFormation.ballEffect.xRange.clamped(assumedBallPos.x()), assumedBallPos.y());

      // scale the ball pos according to the ballEffect
      scaledBallPos =
          Vector2f((clampedBallPos.x() - tacticsFormation.ballEffect.xAnchor) * tacticsFormation.ballEffect.scale.x(),
                   clampedBallPos.y() * tacticsFormation.ballEffect.scale.y());

      // calc the formation shift and scale parameters based on ball effect.

      const REFormations::Formation &formation = theREFormations.formations[theREActiveTactic.formationId];

      ACTGRAPH_FMT("formationId: {}", TypeRegistry::getEnumName(theREActiveTactic.formationId));

      // clamp the yShift (derived from the ball effect) to apply based on the valid range
      shiftedAnchor.y() = Rangef(-formation.yShift, formation.yShift).clamped(scaledBallPos.y());
      // set the scaling of the formation based on where we are in the available shift range.
      // Range::scale() divides by (tRange.max - tRange.min), i.e. formation.yShift here - guard
      // against a formation configured with yShift = 0 (a fully static formation with no
      // ball-tracking shift), which would otherwise divide by zero and produce NaN.
      float yScale = (formation.yShift > 0.f)
          ? Rangef(1.0f, formation.yScale).scale(std::abs(shiftedAnchor.y()), Rangef(0, formation.yShift))
          : 1.0f;

      // TLOGV(tlogger, "yShift = {:.0f}, yScale = {:.2f}", yShift, yScale);

      // ACTGRAPH_FMT("scaledBallPos: {}", scaledBallPos);
      // ACTGRAPH_FMT("xShift range: min {:.0f} max {:.0f}", formation.xShift.min, formation.xShift.max);

      // clamp the xShift (derived from the ball effect) to apply based on the valid range
      float xShift = formation.xShift.clamp(scaledBallPos.x());

      // ACTGRAPH_FMT("xShift: {}", xShift);

      // set the xScale to apply based on where we are in the available xShift range
      // and whether it is in the subrange from min to zero, or from zero to max.
      // Same divide-by-zero guard as yScale above - formation.xShift.min/max = 0 (a fully
      // static formation) must not be passed as the "to" range of Range::scale().
      float xScale =
          xShift <= 0
              ? (formation.xShift.min < 0.f
                     ? Rangef(1.0f, formation.xScale.min).scale(std::abs(xShift), Rangef(0, std::abs(formation.xShift.min)))
                     : 1.0f)
              : (formation.xShift.max > 0.f
                     ? Rangef(1.0f, formation.xScale.max).scale(xShift, Rangef(0, formation.xShift.max))
                     : 1.0f);

      shiftedAnchor.x() = formation.xAnchor + xShift;

      // ACTGRAPH_FMT("shiftedAnchor: {}", shiftedAnchor);

      // update the formation positions according to the ball effect
      for (auto& pos : formationPositionsOnField)
      {
        pos.x() = ((pos.x() - formation.xAnchor) * xScale) + shiftedAnchor.x();
        pos.y() = (pos.y() * yScale) + shiftedAnchor.y();
      }
    }



    REFormations::FormationId findBestFormation(const std::vector<REFormations::FormationId>& formationIds)
    {
      size_t numPlayersOnField = theRETeamData.activeTeammates.size();
      if (!theGameState.isPenalized())
        numPlayersOnField += 1; // self is not in activeTeammates so add 1 if it is an active player

      // Before the first GameController packet is received, theGameControllerData (and therefore
      // theRETeamData.activeTeammates) defaults to every player slot looking "active", which can
      // transiently inflate numPlayersOnField up to Settings::numPlayerNumbers. Clamp to the
      // configured roster size so that startup race doesn't look like a genuine oversized team.
      numPlayersOnField = std::min(numPlayersOnField, static_cast<size_t>(theREGameConfig.playersPerTeam));

      // find the formation which most closely matches the number of players
      size_t iBest = 0;

      // ensure that at least the first available formation is big enough for the number of players
      if (theREFormations.formations[formationIds[iBest]].positions.size() < numPlayersOnField)
      {
        TLOG_ABORT(tlogger, "**** REFormations[{}] only supports {} players and we have {} players on the field",
                    TypeRegistry::getEnumName(formationIds[iBest]),
                    theREFormations.formations[formationIds[iBest]].positions.size(), numPlayersOnField);
      }

      for (size_t i = 1; i < formationIds.size(); i++)
      {
        if ((theREFormations.formations[formationIds[i]].positions.size() >= static_cast<size_t>(numPlayersOnField)) &&
            (theREFormations.formations[formationIds[i]].positions.size() < theREFormations.formations[formationIds[iBest]].positions.size()))
        {
          iBest = i;
        }
      }
      // at this point iBest should refer to the formation that most closely fits the number of players (either equal to
      // or supporting the smallest number of additional players)

      return formationIds[iBest];
    }


    // Team convention: player 1 always plays goalkeeper
    static constexpr int ownGoalkeeperPlayerNumber = 1;

    void assignNormalGoalkeeper()
    {
      // Assign by fixed player number rather than searching formationRoles for the slot
      // labelled "goalie" - every other assignment in this file (assignBallPlayer,
      // assignSupporterIndices) indexes situationRoles by
      // Settings::toPlayerIndex(playerNumber) directly, so this stays consistent with that.
      theREActiveTactic.situationRoles[Settings::toPlayerIndex(ownGoalkeeperPlayerNumber)] = PlayerRole::goalkeeper;
    }


    void assignBallPlayer(bool useBall)
    {
      int bestPlayerNumber = -1;

      bool recentlyPenalized =
          theGameState.isPenalized() ||
          (theFrameInfo.getTimeSince(theExtendedGameState.timeWhenLastUnpenalized) <= params.afterPenaltyMs);

      if (!useBall)
      {
        // just assume the highest number non-penalized player is the ball player
        // (TODO: Do we need something more sophisticated here?)

        if (!recentlyPenalized) // FIXME: should we treat MOTION_IN_SET/STANDBY differently here
          bestPlayerNumber = theGameState.playerNumber;

        for (auto pTeammate : theRETeamData.activeTeammates)
        {
          if ((pTeammate->number > bestPlayerNumber) && (!pTeammate->isPenalized))
            bestPlayerNumber = pTeammate->number;
        }

        // NOTE: we might not have found a valid bestPlayerNumber - but dealt with below
      }
      else // do use the ball
      {
        // TODO: deal with fallen robots
        // TODO: deal with robots should not go too far out of position - needs to interact with passing
        // behaviour
        // TODO: deal with close races and robots near to the ball
        bestPlayerNumber = -1;
        int bestTimeToBall = std::numeric_limits<int>::max();

        // check self first
        // we prefer not to turn a goal keeper into the ball player, but we'll do it
        // if we're the only active robot
        int tmp;

        if (!recentlyPenalized &&
            ((theGameState.playerNumber != ownGoalkeeperPlayerNumber) || theRETeamData.activeTeammates.empty()))
        {
          bool isUpright = (theRESentTeamMessage2025.frameTime > 0) ? theRESentTeamMessage2025.teamMessage.isUpright : true;
          tmp = getTimeToBall(theRobotPose, theREActiveTactic.situationRole, isUpright); // includes hysteresis
          if (tmp < bestTimeToBall)
          {
            bestPlayerNumber = theGameState.playerNumber;
            bestTimeToBall = tmp;
          }

          TLOGV(tlogger, "assignBallPlayer: robot {} (self): {} ms from ball", theGameState.playerNumber, tmp);
        }

        // then check active teammates
        for (auto& location : theRETeammatesLocationModel.locations)
        {
          RETeammate& teammate = *theRETeamData.teammatesByNumber[location.playerNumber];

          if (teammate.number == ownGoalkeeperPlayerNumber) // we don't want the goalkeeper to be striker
            continue;

          tmp = getTimeToBall(location.pose, teammate.theREActiveTacticStatus.situationRole,
                              teammate.isUpright); // includes hysteresis
          if (tmp < bestTimeToBall)
          {
            bestPlayerNumber = location.playerNumber;
            bestTimeToBall = tmp;
          }
          TLOGV(tlogger, "assignBallPlayer: robot {}: {} ms from ball", location.playerNumber, tmp);
        }

        // NOTE: we might not have found a valid bestPlayerNumber - but dealt with below
      }

      if (bestPlayerNumber > -1)
        TLOGV(tlogger, "assignBallPlayer: ballPlayer should be robot {}", bestPlayerNumber);
      else
      {
        // if we didn't find a ball player, we'll have to do it ourselves
        bestPlayerNumber = theGameState.playerNumber;
        TLOGV(tlogger, "assignBallPlayer: ballPlayer is self (robot {}), no alternative found", bestPlayerNumber);
      }

      // update the situationRoles
      theREActiveTactic.situationRoles[Settings::toPlayerIndex(bestPlayerNumber)] = PlayerRole::ballPlayer;
    }


    /**
     * Every remaining player (i.e. not the goalkeeper or ball player) is a supporter.
     * Give each one a distinct legacy supporterN role in formation order so that
     * PlayerRole::supporterIndex() can tell them apart. Without this, every supporter's
     * situationRole stays PlayerRole::none, supporterIndex() reports -1 for all of them,
     * and SupporterKickInTacticsTask::getTacticPose() (Supporter.h) falls back to the same
     * interceptor position for the whole team during a set play.
     */
    void assignSupporterIndices()
    {
      int supporterIndex = 0;
      for (PlayerRole::Type& situationRole : theREActiveTactic.situationRoles)
      {
        if (situationRole == PlayerRole::none)
          situationRole = static_cast<PlayerRole::Type>(PlayerRole::firstSupporterRole + supporterIndex++);
      }
    }


    /**
     * calculate an approximate time/cost to get into position at the ball.
     * The role parameter allows us to apply hysteresis before overriding the
     * current ball player,
     */
    unsigned getTimeToBall(const Pose2f &pose, PlayerRole::Type role, bool isUpright, bool lineUpUpfield = true)
    {
      // straight line time to the ball (ignoring obstacles, the side we're approaching from, and orientation)
      // -------------------------------------------------------------------
      // FIXME: maybe we should be using assumedBall or teamBall???
      Vector2f ballPositionOnField = theFieldBall.recentBallEndPositionOnField();
                                              // .endPositionOnField;

      // need to account for the foot length which is also encapsulated in the 
      // "average" kickPose distance when getting close to the ball
      // float distanceToBall = std::max((ballPositionOnField - pose.translation).norm() - theRobotDimensions.footLength, 0.f);
      float distanceToBall = std::max((ballPositionOnField - pose.translation).norm() - 190.f, 0.f);
      float timeToBallSecs = distanceToBall / (theWalkingEngineOutput.maxSpeed.translation.x() * params.walkSpeedFudgeFactor);
      unsigned straightTimeToBall = static_cast<unsigned>(timeToBallSecs * 1000.0f); // convert to ms

      // re-orientation before walk to ball
      // -------------------------------------------------------------------
      Angle ballToRobotAngle = (pose.translation - ballPositionOnField).angle();
      Angle rotation = 0_deg;

      // is the robot more or less ahead of or behind of the ball?
      if ((std::abs(ballToRobotAngle) < 45_deg) || (std::abs(ballToRobotAngle) > 135_deg))
      {
        rotation = Angle(ballToRobotAngle + 180_deg).diffAbs(pose.rotation); // rotate to orient at the ball in this case
        if (rotation < 10_deg)
          rotation = 0_deg;
      }
      else
      {
        // rotate to walk sideways if we need to rotate more than 90 deg
        rotation = Angle(ballToRobotAngle + 180_deg).diffAbs(pose.rotation);
        if (rotation > 90_deg)
          rotation -= 90_deg;
      }

      float preRotationTimeSecs = rotation / theWalkingEngineOutput.maxSpeed.rotation;
      unsigned preRotationTime = static_cast<unsigned>(preRotationTimeSecs * 1000.f);

      // line up to kick upfield, if requested
      // -------------------------------------------------------------------
      unsigned lineUpUpfieldTime = 0;
      if (lineUpUpfield)
      {
        Angle robotToBallAngle = Angle(ballToRobotAngle + 180_deg).normalize();
        // FIXME: this is a very rough approximation to how ball approach and encircling works
        Angle absEncircleAngle = std::max(Angle(std::abs(robotToBallAngle) - 45_deg), 0_deg);

        // TLOGV(tlogger, "getTimeToBall: ballToRobotAngle {}, robotToBallAngle {}, absEncircleAngle {}", ballToRobotAngle,
        //       robotToBallAngle, absEncircleAngle);

        // assume we encircle at 250mm radius
        const float arcLength = 250.f * absEncircleAngle;
        const float correctionFactor = 0.5f; // we assume the walk is slower than theWalkingEngineOutput claims
        const float arcTimeSecs = arcLength / (theWalkingEngineOutput.maxSpeed.translation.y() * correctionFactor);
        lineUpUpfieldTime = static_cast<unsigned>(arcTimeSecs * 1000.f);
      }

      // add it all up
      unsigned totalTime = preRotationTime + straightTimeToBall + lineUpUpfieldTime;

      // bool upright =
      //     (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering ||
      //      theFallDownState.state == FallDownState::squatting) &&
      //     (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp &&
      //      theMotionRequest.motion != MotionRequest::getUp);

      if (!isUpright)
        totalTime += params.getUpMs;

      if (role != PlayerRole::ballPlayer)
        totalTime += params.ballPlayerBonusMs;

      TLOGV(tlogger, "getTimeToBall: {} preRotation {} + straight {} + lineUp {} --> {}",
            (role == PlayerRole::ballPlayer) ? "ballplayer" : "other", preRotationTime, straightTimeToBall,
            lineUpUpfieldTime, totalTime);

      return totalTime;
    }




    void limitGoalkeeperPosition(Vector2f& goaliePos)
    {
      // ensure the goalkeeper does not try to leave the field/goal mouth due to formation alone
      if (goaliePos.x() < theFieldDimensions.xPosOwnGoalLine)
        goaliePos.x() = theFieldDimensions.xPosOwnGoalLine;

      if (goaliePos.y() > params.maxGoalkeeperY)
        goaliePos.y() = params.maxGoalkeeperY;
      else if (goaliePos.y() < -params.maxGoalkeeperY)
        goaliePos.y() = -params.maxGoalkeeperY;
    }


    Pose2f getFormationPose(const Vector2f& playerPos, const Vector2f& assumedBallPosOnField)
    {
      Angle assumedBallAngle = (assumedBallPosOnField - playerPos).angle();
      // Note that upfield is angle 0 in field coordinates so we don't explicitly mention it in most calculations

      // TLOGV(tlogger, "playerPos {}, assumedBallPos {}, assumedBallAngle {}",
      //       playerPos, assumedBallPosOnField, assumedBallAngle);

      Pose2f pose = Pose2f(0, playerPos);

      if (std::fabs(assumedBallAngle) <= (2 * params.ballAngleSplitUpfield))
        pose.rotation = assumedBallAngle / 2;
      else if (std::fabs(assumedBallAngle) > params.ballAngleLookDirect)
        pose.rotation = assumedBallAngle;
      else // split between looking upfield and the ball but not more than ballAngleSplitUpfield from ball
        pose.rotation = (assumedBallAngle > 0)
                                  ? assumedBallAngle - params.ballAngleSplitUpfield
                                  : assumedBallAngle + params.ballAngleSplitUpfield;

      return pose;
    }


    void updateTeamPlayStatus()
    {
      // have we just started a set play?
      if (theGameState.isFreeKick() && !theExtendedGameState.wasFreeKick())
      {
        numKickingRobotsSinceLastSetPlay = 0;
      }
      else // continuing since prev set play
      {
        Vector2f shootingTo(0,0);
        int playerWhoIsShooting = -1;

        // TODO does the current robot report a kick
        if (theBehaviorStatus.shootingTo.has_value() && (theBehaviorStatus.shootingTo->squaredNorm() > sqr(300.f)))
        {
          shootingTo = *theBehaviorStatus.shootingTo;
          playerWhoIsShooting = theGameState.playerNumber;
        }
        else
        {
        // TODO or do teammates report a kick

        }

        (void)(playerWhoIsShooting); // TODO - tmp force use of var

        // TODO check if ball supports a kick being executed
      }
    }

    /**
     * populate REActiveTactic based on tactic TeamInfo
     */
    void updateOwnTacticInfo()
    {
      // FIXME naive: use player index to select from TeamInfo (i.e. formation role never changes)
      // In future we need to look this up via a map
      int i = Settings::toPlayerIndex(theGameState.playerNumber);

      // theREActiveTactic.formationId - already updated with TeamTacticInfo
      theREActiveTactic.formationRole = theREActiveTactic.formationRoles[i];
      theREActiveTactic.situationRole = theREActiveTactic.situationRoles[i];

      theREActiveTactic.formationPose = theREActiveTactic.formationPoses[i];

      PlayerRole pr;
      pr.role = theREActiveTactic.situationRole;
      pr.numOfActiveSupporters = 0; // we don't use this any more

      updateTeamPlayStatus();

      RETeamPlayStatus teamPlayStatus;
      teamPlayStatus.numKickingRobotsSinceLastSetPlay = numKickingRobotsSinceLastSetPlay;

      teamBehaviourStatusSkills.recordPlayerRole(pr);
      // TODO add skill
      // teamBehaviourStatusSkills.recordTeamPlayStatus(teamPlayStatus);

      // default (meaningless) values for all of these since we don't use them from 2024 on
      // teamBehaviourStatusSkills.recordTeamActivity(TeamBehaviorStatus::noTeam);
      teamBehaviourStatusSkills.recordTimeToReachBall(TimeToReachBall());
      teamBehaviourStatusSkills.recordTeammateRoles(TeammateRoles());
    }



    /**
     * draw the formations decided by this team coordination behaviour
     */
    void draw()
    {
      DEBUG_DRAWING("behaviour:TeamCoordination:formation", "drawingOnField")
      {
        ColorRGBA ballColor = ColorRGBA::black.lighter(0.25).alpha(0.5f);

        CIRCLE("behaviour:TeamCoordination:formation", assumedBallPos.x(), assumedBallPos.y(), 120, 30, Drawings::solidPen,
              ballColor, Drawings::noBrush, ColorRGBA::black);

        CIRCLE("behaviour:TeamCoordination:formation", clampedBallPos.x(), clampedBallPos.y(), 160, 30, Drawings::dashedPen,
              ballColor, Drawings::noBrush, ColorRGBA::black);

        CIRCLE("behaviour:TeamCoordination:formation", scaledBallPos.x(), scaledBallPos.y(), 200, 30, Drawings::dottedPen,
              ballColor, Drawings::noBrush, ColorRGBA::black);

        // for (const Vector2f& pos : activeTactic.formationPositionsOnField)
        // {
        //   // LARGE_DOT("behaviour:TeamCoordination:formation", pos.x(), pos.y(), ColorRGBA::black, ColorRGBA::gray.alpha(0.5f));
        //   CROSS("behaviour:TeamCoordination:formation", pos.x(), pos.y(), 50, 30, Drawings::solidPen, posColor);
        // }

        ColorRGBA posColor = ColorRGBA::black.alpha(0.5f);

        CIRCLE("behaviour:TeamCoordination:formation", shiftedAnchor.x(), shiftedAnchor.y(), 50, 1, Drawings::noPen,
              posColor, Drawings::solidBrush, posColor);

        int playerNumber = 1;
        for (const Pose2f& pose : theREActiveTactic.formationPoses)
        {
          CIRCLE("behaviour:TeamCoordination:formation", pose.translation.x(), pose.translation.y(), 150, 30,
                  Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

          if (playerNumber == theGameState.playerNumber) // self?          
            CIRCLE("behaviour:TeamCoordination:formation", pose.translation.x(), pose.translation.y(), 220, 30,
                    Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

          DRAW_TEXT("behaviour:TeamCoordination:formation", pose.translation.x()-75, pose.translation.y()+220, 150,
                    posColor, playerNumber);

          Vector2f directionVec = pose * Vector2f(350, 0);

          ARROW("behaviour:TeamCoordination:formation", pose.translation.x(), pose.translation.y(), directionVec.x(),
                directionVec.y(), 30, Drawings::solidPen, posColor);

          ++playerNumber;
        }
      }
    }
  };

} // namespace v2025
} // namespace RE
