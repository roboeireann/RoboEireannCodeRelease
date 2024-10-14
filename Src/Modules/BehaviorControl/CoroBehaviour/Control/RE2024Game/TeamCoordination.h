/**
 * @file TeamCoordination.h
 *
 * This task implements the initial team coordination using the formation
 * and tactics architecture
 *
 * @author Rudi Villing
 */


#pragma once

#include "Modules/BehaviorControl/CoroBehaviour/2024/TeamBehaviour2024.h"

// #include "Modules/BehaviorControl/CoroBehaviour/ActiveTacticSkills.h"
#include "Representations/Configuration/Formations.h"
#include "Representations/Configuration/Tactics.h"

#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/TextLogging.h"
#include "Tools/FmtCommonTypes.h"

namespace CoroBehaviour
{
namespace RE2024
{
  // =================================================================================================================

  CRBEHAVIOUR(TeamCoordinationTask)
  {
    CRBEHAVIOUR_INIT(TeamCoordinationTask) {}

    void operator()(void)
    {
      CRBEHAVIOUR_LOOP()
      {
        if ((theGameInfo.state == STATE_INITIAL) || (theGameInfo.state == STATE_STANDBY))
        {
          CR_CHECKPOINT(initial_or_standby);
          updateKickoffSetupTeamInfo();
        }
        else if (theGameInfo.setPlay == SET_PLAY_NONE)
        {
          if ((theGameInfo.state == STATE_READY) || (theGameInfo.state == STATE_SET))
          {
            CR_CHECKPOINT(prepare_for_kickoff);
            updateKickoffSetupTeamInfo();
          }
          else if ((theGameInfo.state == STATE_PLAYING) && (theGameInfo.setPlay == SET_PLAY_NONE))
          {
            // TODO deal with kickoff
            // else
            updatePlayingTeamInfo();
          }          
        }
        else
        {
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
    READS(Formations);
    READS(Tactics);
    READS(TeamData);
    READS(FrameInfo);
    READS(RobotPose);
    READS(GameInfo);
    READS(ExtendedGameInfo);
    READS(FieldBall);
    READS(WalkingEngineOutput);
    READS(TeammatesLocationModel);
    READS(SentTeamMessage2024);

    MODIFIES(TeamBehaviorStatus);
    MODIFIES(ActiveTactic);

    TeamBehaviorStatusSkills teamBehaviourStatusSkills {env};

    std::vector<Formations::FormationRole> prevFramePlayerIndexToFormationRole;


    // members related to positioning the formation according to the ball.
    // We make these members so that we can refer to them in drawing code.
    Vector2f assumedBallPos;
    Vector2f clampedBallPos;
    Vector2f scaledBallPos;
    Vector2f shiftedAnchor;



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
      if (theGameInfo.isOurKick())
        updateTeamTacticInfo(theTactics.ourKickoffFormation, /* useBall */ false);
      else
        updateTeamTacticInfo(theTactics.opponentKickoffFormation, /* useBall */ false);
    }

    void updatePassiveTeamInfo()
    {
      updateTeamTacticInfo(theTactics.defendingFormation, /* useBall */ true);
    }

    void updatePlayingTeamInfo()
    {
      // TODO - tmp hack
      if (theActiveTactic.attacking)
        updateTeamTacticInfo(theTactics.attackingFormation, /* useBall */ true);
      else
        updateTeamTacticInfo(theTactics.defendingFormation, /* useBall */ true);
    }


    void updateTeamTacticInfo(const Tactics::Formation &tacticsFormation, bool useBall)
    {
      theActiveTactic.formationId = findBestFormation(tacticsFormation.formationIds);

      updateAssumedBall();

      theActiveTactic.attacking = useBall ? checkIfAttacking() : true; // FIXME - not really implemented yet

      const Formations::Formation &formation = theFormations.formations[theActiveTactic.formationId];
      std::vector<Vector2f> formationPositionsOnField = formation.positions;

      if (useBall)
        // modify formations based on ball effect
        updateBallEffect(tacticsFormation, formationPositionsOnField);
      else
      {
        assumedBallPos = clampedBallPos = scaledBallPos = theActiveTactic.assumedBallPositionOnField = Vector2f::Zero();
        shiftedAnchor = Vector2f(theFormations.formations[theActiveTactic.formationId].xAnchor, 0);
      }

      // correct the goalie position
      limitGoalkeeperPosition(formationPositionsOnField[0]);
      // TODO limit other positions to sensible on field positions, limit number of defenders in the box, etc.

      // populate the poses
      theActiveTactic.formationPoses.clear();
      for (const Vector2f& pos : formationPositionsOnField)
      {
        theActiveTactic.formationPoses.push_back(getFormationPose(pos, theActiveTactic.assumedBallPositionOnField));
      }

      // populate the formation roles corresponding to each pose
      theActiveTactic.formationRoles = formation.roles;

      // indicate which players will play in which formation position (only changes when the formation changes
      // of the number of players changes)
 //     updatePlayerFormationRoleAssignments();

      // assign situation roles
      // std::vector<PlayerRole::Type> prevSituationRoles = theActiveTactic.situationRoles;
      // prevSituationRoles.resize(theActiveTactic.formationRoles.size(), PlayerRole::none);
      // reset the existing situation roles before reassigning
      theActiveTactic.situationRoles.assign(theActiveTactic.formationRoles.size(), PlayerRole::none);

      assignNormalGoalkeeper();
      assignBallPlayer(useBall);
    }


    /**
     * Assign the "best" formation role to each player
     */
    void updatePlayerFormationRoleAssignments()
    {
      // When this function is called, the formationId, formationRoles, and formationPoses of theActiveTactic
      // have already been updated to their latest values.
      //
      // Our objective is to update playerFormationRoleAssignments to the latest based on our memory
      // of previous assignments and the currently available players.

      // FIXME: implementation of this function is broken and was not used in RC2024 so removed from code release
    }

    void updateAssumedBall()
    {
      // calc ball effect and key parameters for formation movement

      if (theGameInfo.state != STATE_PLAYING) // FIXME add in special cases for penalties etc here
        assumedBallPos = Vector2f::Zero();
      else if (theFieldBall.timeSinceBallWasSeen < 2000) // FIXME param
        assumedBallPos = theFieldBall.endPositionOnField;
      else if (theFieldBall.isTeamBallValid)
        assumedBallPos = theFieldBall.teamEndPositionOnField;
      else if (theFieldBall.timeSinceBallWasSeen < 10000) // FIXME param
        assumedBallPos = theFieldBall.endPositionOnField; // fall back to our own old ball if no valid team ball
      else
        assumedBallPos = Vector2f::Zero(); // no ball seen anywhere recently

      theActiveTactic.assumedBallPositionOnField = assumedBallPos;
    }

    bool checkIfAttacking()
    {
      // FIXME - hack while early developing, needs logic to work out if we are in attack
      // implements basic hysteresis
      if (theActiveTactic.attacking && (assumedBallPos.x() < params.attackingHysteresis.min))
        return false; // defending
      else if (!theActiveTactic.attacking && (assumedBallPos.x() > params.attackingHysteresis.max))
        return true;

      return theActiveTactic.attacking; // leave as-is
    }

    void updateBallEffect(const Tactics::Formation &tacticsFormation, std::vector<Vector2f> &formationPositionsOnField)
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

      const Formations::Formation &formation = theFormations.formations[theActiveTactic.formationId];

      ACTGRAPH_FMT("formationId: {}", TypeRegistry::getEnumName(theActiveTactic.formationId));

      // clamp the yShift (derived from the ball effect) to apply based on the valid range
      shiftedAnchor.y() = Rangef(-formation.yShift, formation.yShift).clamped(scaledBallPos.y());
      // set the scaling of the formation based on where we are in the available shift range
      float yScale = Rangef(1.0f, formation.yScale).scale(std::abs(shiftedAnchor.y()), Rangef(0, formation.yShift));

      // TLOGV(tlogger, "yShift = {:.0f}, yScale = {:.2f}", yShift, yScale);

      // ACTGRAPH_FMT("scaledBallPos: {}", scaledBallPos);
      // ACTGRAPH_FMT("xShift range: min {:.0f} max {:.0f}", formation.xShift.min, formation.xShift.max);

      // clamp the xShift (derived from the ball effect) to apply based on the valid range
      float xShift = formation.xShift.clamp(scaledBallPos.x());

      // ACTGRAPH_FMT("xShift: {}", xShift);

      // set the xScale to apply based on where we are in the available xShift range
      // and whether it is in the subrange from min to zero, or from zero to max
      float xScale =
          xShift <= 0
              ? Rangef(1.0f, formation.xScale.min).scale(std::abs(xShift), Rangef(0, std::abs(formation.xShift.min)))
              : Rangef(1.0f, formation.xScale.max).scale(xShift, Rangef(0, formation.xShift.max));

      shiftedAnchor.x() = formation.xAnchor + xShift;

      // ACTGRAPH_FMT("shiftedAnchor: {}", shiftedAnchor);

      // update the formation positions according to the ball effect
      for (auto& pos : formationPositionsOnField)
      {
        pos.x() = ((pos.x() - formation.xAnchor) * xScale) + shiftedAnchor.x();
        pos.y() = (pos.y() * yScale) + shiftedAnchor.y();
      }
    }



    Formations::FormationId findBestFormation(const std::vector<Formations::FormationId>& formationIds)
    {
      size_t numPlayersOnField = theTeamData.activeTeammates.size();
      if (!theGameInfo.isPenalized())
        numPlayersOnField += 1; // self is not in activeTeammates so add 1 if it is an active player

      // find the formation which most closely matches the number of players
      size_t iBest = 0;

      // ensure that at least the first available formation is big enough for the number of players
      if (theFormations.formations[formationIds[iBest]].positions.size() < numPlayersOnField)
      {
        TLOGF_ABORT(tlogger, "**** Formations[{}] only supports {} players and we have {} players on the field",
                    TypeRegistry::getEnumName(formationIds[iBest]),
                    theFormations.formations[formationIds[iBest]].positions.size(), numPlayersOnField);
      }

      for (size_t i = 1; i < formationIds.size(); i++)
      {
        if ((theFormations.formations[formationIds[i]].positions.size() >= static_cast<size_t>(numPlayersOnField)) &&
            (theFormations.formations[formationIds[i]].positions.size() < theFormations.formations[formationIds[iBest]].positions.size()))
        {
          iBest = i;
        }
      }
      // at this point iBest should refer to the formation that most closely fits the number of players (either equal to
      // or supporting the smallest number of additional players)

      return formationIds[iBest];
    }


    void assignNormalGoalkeeper()
    {
      // assign the goalkeeper
      auto& formationRoles = theActiveTactic.formationRoles;
      auto iter = std::find_if(formationRoles.begin(), formationRoles.end(),
                               [](Formations::FormationRole r) { return r == Formations::goalie; });

      if (iter != formationRoles.end())
        theActiveTactic.situationRoles[iter - formationRoles.begin()] = PlayerRole::goalkeeper;
    }


    void assignBallPlayer(bool useBall)
    {
      int bestPlayerNumber = -1;

      if (!useBall)
      {
        // just assume the highest number non-penalized player is the ball player
        // (TODO: Do we need something more sophisticated here?)

        if (!theGameInfo.isPenalized() &&
            (theExtendedGameInfo.timeSinceLastPenaltyEnded >
             params.afterPenaltyMs)) // TODO: should we treat MOTION_IN_SET/STANDBY differently here
          bestPlayerNumber = theGameInfo.playerNumber;

        for (auto pTeammate : theTeamData.activeTeammates)
        {
          if ((pTeammate->number > bestPlayerNumber) && (!pTeammate->isPenalized))
            bestPlayerNumber = pTeammate->number;
        }
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
        bool recentlyPenalized =
            theGameInfo.isPenalized() || (theExtendedGameInfo.timeSinceLastPenaltyEnded <= params.afterPenaltyMs);

        if (!recentlyPenalized && (!theGameInfo.isGoalkeeper() || theTeamData.activeTeammates.empty()))
        {
          // what role do all other teammates believe we have
          PlayerRole::Type currentRole = PlayerRole::none;
          bool isUpright = true;
          if (theSentTeamMessage2024.frameTime > 0)
          {
            currentRole = theSentTeamMessage2024.teamMessage.activeTacticStatus.situationRole;
            isUpright = theSentTeamMessage2024.teamMessage.isUpright;
          }
          // tmp = getTimeToBall(theRobotPose, theActiveTactic.situationRole); // includes hysteresis
          tmp = getTimeToBall(theRobotPose, currentRole, isUpright); // includes hysteresis
          if (tmp < bestTimeToBall)
          {
            bestPlayerNumber = theGameInfo.playerNumber;
            bestTimeToBall = tmp;
          }

          TLOGV(tlogger, "assignBallPlayer: robot {} (self): {} ms from ball", theGameInfo.playerNumber, tmp);
        }

        // then check active teammates
        for (auto& location : theTeammatesLocationModel.locations)
        {
          Teammate& teammate = *theTeamData.teammatesByNumber[location.playerNumber];

          if (teammate.isGoalkeeper) // we don't want the goalkeeper to be striker
            continue;

          tmp = getTimeToBall(location.pose, teammate.theActiveTacticStatus.situationRole,
                              teammate.isUpright); // includes hysteresis
          if (tmp < bestTimeToBall)
          {
            bestPlayerNumber = location.playerNumber;
            bestTimeToBall = tmp;
          }
          TLOGV(tlogger, "assignBallPlayer: robot {}: {} ms from ball", location.playerNumber, tmp);
        }

        TLOGV(tlogger, "assignBallPlayer: ballPlayer should be robot {}", bestPlayerNumber);

        // if we didn't find a ball player, we'll have to do it ourselves
        if (bestPlayerNumber == -1)
          bestPlayerNumber = theGameInfo.playerNumber;
      }

      // update the situationRoles
      theActiveTactic.situationRoles[Settings::toPlayerIndex(bestPlayerNumber)] = PlayerRole::ballPlayer;
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
      if (goaliePos.x() < theFieldDimensions.xPosOwnGroundLine)
        goaliePos.x() = theFieldDimensions.xPosOwnGroundLine;

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


    /**
     * populate ActiveTactic based on tactic TeamInfo
     */
    void updateOwnTacticInfo()
    {
      // FIXME naive: use player index to select from TeamInfo (i.e. formation role never changes)
      // In future we need to look this up via a map
      int i = theGameInfo.playerIndex();

      // theActiveTactic.formationId - already updated with TeamTacticInfo
      theActiveTactic.formationRole = theActiveTactic.formationRoles[i];
      theActiveTactic.situationRole = theActiveTactic.situationRoles[i];

      theActiveTactic.formationPose = theActiveTactic.formationPoses[i];

      PlayerRole pr;
      pr.role = theActiveTactic.situationRole;
      pr.numOfActiveSupporters = 0; // we don't use this any more

      teamBehaviourStatusSkills.playerRole(pr);
      // default (meaningless) values for all of these since we don't use them from 2024 on
      teamBehaviourStatusSkills.teamActivity(TeamBehaviorStatus::noTeam);
      teamBehaviourStatusSkills.timeToReachBall(TimeToReachBall());
      teamBehaviourStatusSkills.teammateRoles(TeammateRoles());
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
        for (const Pose2f& pose : theActiveTactic.formationPoses)
        {
          CIRCLE("behaviour:TeamCoordination:formation", pose.translation.x(), pose.translation.y(), 150, 30,
                  Drawings::solidPen, posColor, Drawings::noBrush, ColorRGBA::black);

          if (playerNumber == theGameInfo.playerNumber) // self?          
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

} // RE2024
} // CoroBehaviour2022
