/**
 * @file GameState.h
 *
 * This file declares a representation of the game state in a way that is
 * useful for behaviors etc.
 *
 * @author Arne Hasselbring
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Framework/Settings.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"
#include "Streaming/Function.h"

#include <algorithm>
#include <array>

STREAMABLE(GameState,
{
  ENUM(State,
  {,
    beforeHalf, ///< The half hasn't started yet. 
    standby, ///< Wait for the signal to walk in from the touchlines. - deprecated 2026, but retained unused for now
    afterHalf, ///< The half is over. 
    timeout, ///< A team or the referee has taken a timeout. 

    playing, ///< Regular play, no special privileges or restrictions (except for potential illegal goals). 
    // stopped, ///< A GC stop signal is active

    setupOwnKickOff, ///< The READY phase before an own kick-off. 
    setupOpponentKickOff, ///< The READY phase before an opponent kick-off. 
    setupDropBall, ///< The READY phase before a drop-ball. - deprecated 2026

    waitForOwnKickOff, ///< The SET phase before an own kick-off. 
    waitForOpponentKickOff, ///< The SET phase before an opponent kick-off. 
    waitForDropBall, ///< The SET phase before a drop-ball. - deprecated 2026

    ownKickOff, ///< The phase of an own kick-off in which the opponent must not enter the center circle. 
    opponentKickOff, ///< The phase of an opponent kick-off in which we must not enter the center circle. 

    setupOwnPenaltyKick, ///< The READY phase before an own penalty kick. 
    setupOpponentPenaltyKick, ///< The READY phase before an opponent penalty kick. 

    waitForOwnPenaltyKick, ///< The SET phase before an own penalty kick. 
    waitForOpponentPenaltyKick, ///< The SET phase before an opponent penalty kick. 

    ownPenaltyKick, ///< The phase of an own penalty kick in which the goalkeeper must stay on its line and nobody is allowed to enter the penalty area. 
    opponentPenaltyKick, ///< The phase of an opponent penalty kick in which the goalkeeper must stay on its line and nobody is allowed to enter the penalty area. 

    // ownPushingFreeKick, ///< A pushing free kick for the own team.  // deprecated 2026
    // opponentPushingFreeKick,

    ownDirectFreeKick, /// a direct free kick for our own team - added 2026
    opponentDirectFreeKick,

    ownIndirectFreeKick, /// an indirect free kick for our own team - added 2026
    opponentIndirectFreeKick,

    ownThrowIn, ///< A throw-in for the own team. 
    opponentThrowIn, ///< A throw-in for the opponent team. 
    ownGoalKick, ///< A goal kick for the own team. 
    opponentGoalKick, ///< A goal kick for the opponent team. 
    ownCornerKick, ///< A corner kick for the own team. 
    opponentCornerKick, ///< A corner kick for the opponent team. 

    beforePenaltyShootout, ///< A penalty shootout will begin. 
    waitForOwnPenaltyShot, ///< The SET phase before an own penalty kick in a penalty shootout. 
    waitForOpponentPenaltyShot, ///< The SET phase before an opponent penalty kick in a penalty shootout. 
    ownPenaltyShot, ///< An own penalty kick in a penalty shootout. 
    opponentPenaltyShot, ///< An opponent penalty kick in a penalty shootout. 
    afterOwnPenaltyShot, ///< The phase after an own penalty kick in a penalty shootout. 
    afterOpponentPenaltyShot, ///< The phase after on opponent penalty kick in a penalty shootout. 
  });

  ENUM(Phase,
  {,
    firstHalf, ///< The first half of a game. 
    secondHalf, ///< The second half of a game. 
    penaltyShootout, ///< A penalty shoot-out. 
  });

  ENUM(PlayerState,
  {,
    unstiff, ///< The robot is unstiff.
    prepare, ///< added for the BoosterK1 - this mode must be entered before handing over control to our motion code
    calibration, ///< The robot is in calibration mode.

    penalizedManual, ///< The robot has been penalized using the button interface (no GameController is active).
    penalizedIllegalPositioning, // penalized for being in an illegal position - renamed 2026
    penalizedMotionInSet, ///< penalized for motion in set (TODO:Check inplace?). - renamed 2026
    penalizedMotionInStop, ///< penalized for motion while stop signal active (TODO:Check inplace?)- added 2026
    penalizedLocalGameStuck, ///< The robot is penalized for causing a local game stuck
    penalizedIncapableRobot, ///< penalized for being incapable of standing up/playing - renamed 2026
    penalizedRequestForPickup, ///< The robot is picked up.
    penalizedBallHolding, ///< penalized for ball holding.
    penalizedLeavingTheField, ///< penalized for leaving the field.
    penalizedPlayingWithArmsHands, // added 2026 - possibly renamed penalizedIllegalBallContact

    penalizedPushing, ///< penalized for pushing - renamed 2026
    penalizedCautioned, // added 2026
    penalizedSentOff, // added 2026

    // deprecated penalties for 2026 - kept only if used
    // penalizedIllegalPositionInSet, ///< an illegal position in set - deprecated 2026
    // penalizedPlayerStance, ///< an illegal stance - deprecated 2026
    // penalizedIllegalMotionInStandby, ///< illegal motion in standby (in place) - deprecated 2026

    substitute, ///< The robot is a substitute. 
    active, ///< The robot is playing according to the global game state. 
  });

  ENUM(CompetitionType, // added 2026
  {,
    competitionTypeSmall,
    competitionTypeMedium,
    competitionTypeLarge,
  });

  // ENUM(CompetitionPhase, // deprecated 2026 - not deleted since it might come back
  // {,
  //   roundRobin, ///< Preliminary games, the clock is not stopped in ready and set - deprecated 2026
  //   playOff, ///< Final games. The clock is stopped in ready and set. - deprecated 2026
  // });

  STREAMABLE(Team,
  {
    using Color = Settings::TeamColor;

    /**
     * Checks whether a given player is the goalkeeper in a team.
     * @param playerNumber The player jersey number.
     * @return Whether the player has goalkeeper privileges.
     */
    bool isGoalkeeper(int playerNumber) const
    {
      return playerNumber == goalkeeperNumber;
    }

    /**
     * Returns the number of players on the team that are active (not penalized etc)
     * @return The number of players ...
     */
    unsigned int numOfActivePlayers() const
    {
      return static_cast<unsigned int>(std::count_if(playerStates.begin(), playerStates.end(), [](const PlayerState playerState){return playerState == active;}));
    }

    /**
     * Returns the jersey color of a given player in a team.
     * @param playerNumber The player number.
     * @return The jersey color of the given player.
     */
    Color color(int playerNumber) const
    {
      return isGoalkeeper(playerNumber) ? goalkeeperColor : fieldPlayerColor;
    }

    /**** separate header from streamable fields - comma at end of comment ****/,

    (std::array<PlayerState, Settings::numPlayerNumbers>)({}) playerStates, ///< The states of the players, indexed by jersey number - Settings::lowestValidPlayerNumber. 
    (std::array<unsigned, Settings::numPlayerNumbers>)({}) timeWhenPlayerStatesStarted, ///< For each player: Time when this state started. 
    (int)(0) number, ///< The team number in the GameController. 
    (Color)(Color::green) fieldPlayerColor, ///< The jersey color of field players. 
    (Color)(Color::white) goalkeeperColor, ///< The jersey color of the goalkeeper. 
    (int)(1) goalkeeperNumber, ///< The jersey number of the goalkeeper. 
    (unsigned int)(0u) score, ///< The current score. 
    (unsigned int)(12000u) messageBudget, ///< The default remaining message budget at the start of a game. - modified 2026
  });

  /** Constructor. */
  GameState();

  /** Draw the representation. */
  void draw() const;

  static bool isInitial(State state, bool orStandby = true)
  {
    return state == beforeHalf ||
           (orStandby && state == standby) ||
           state == timeout ||
           state == beforePenaltyShootout;
  }

  bool isInitial(bool orStandby = true) const
  {
    return isInitial(state, orStandby);
  }

  static bool isReady(State state)
  {
    return state == setupOwnKickOff ||
           state == setupOpponentKickOff ||
           state == setupDropBall ||
           state == setupOwnPenaltyKick ||
           state == setupOpponentPenaltyKick;
  }

  bool isReady() const
  {
    return isReady(state);
  }

  static bool isSet(State state)
  {
    return state == waitForOwnKickOff ||
           state == waitForOpponentKickOff ||
           state == waitForDropBall ||
           state == waitForOwnPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == waitForOwnPenaltyShot ||
           state == waitForOpponentPenaltyShot;
  }

  bool isSet() const
  {
    return isSet(state);
  }

  static bool isPlaying(State state)
  {
    return state == playing ||
           state == ownKickOff ||
           state == opponentKickOff ||
           state == ownPenaltyKick ||
           state == opponentPenaltyKick ||
          //  state == ownPushingFreeKick ||
          //  state == opponentPushingFreeKick ||
           state == ownDirectFreeKick || state == opponentDirectFreeKick ||
           state == ownIndirectFreeKick || state == opponentIndirectFreeKick ||
           state == ownThrowIn ||
           state == opponentThrowIn ||
           state == ownGoalKick ||
           state == opponentGoalKick ||
           state == ownCornerKick ||
           state == opponentCornerKick ||
           state == ownPenaltyShot ||
           state == opponentPenaltyShot;
  }

  bool isPlaying() const
  {
    return isPlaying(state);
  }

  static bool isFinished(State state)
  {
    return state == afterHalf ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isFinished() const
  {
    return isFinished(state);
  }

  static bool isMotionStoppedState(State state, bool isGCStopActive = false)
  {
    return isGCStopActive || // added 2026
           state == beforeHalf ||
           state == standby ||
           state == afterHalf ||
           state == timeout ||
           state == beforePenaltyShootout ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isMotionStoppedState() const
  {
    return isMotionStoppedState(state, stopped); // stopped means GC signals playing stopped
  }

  static bool isKickOff(State state)
  {
    return state == setupOwnKickOff ||
           state == setupOpponentKickOff ||
           state == waitForOwnKickOff ||
           state == waitForOpponentKickOff ||
           state == waitForDropBall ||
           state == ownKickOff ||
           state == opponentKickOff;
  }

  bool isKickOff() const
  {
    return isKickOff(state);
  }

  static bool isPenaltyKick(State state)
  {
    return state == setupOwnPenaltyKick ||
           state == setupOpponentPenaltyKick ||
           state == waitForOwnPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == ownPenaltyKick ||
           state == opponentPenaltyKick;
  }

  bool isPenaltyKick() const
  {
    return isPenaltyKick(state);
  }

  static bool isPenaltyKickExecution(State state)
  {
    return state == ownPenaltyKick ||
           state == opponentPenaltyKick;
  }

  bool isPenaltyKickExecution() const
  {
    return isPenaltyKickExecution(state);
  }

  static bool isFreeKick(State state)
  {
    return state == ownDirectFreeKick ||
           state == opponentDirectFreeKick ||
           state == ownIndirectFreeKick ||
           state == opponentIndirectFreeKick ||
          //  state == ownPushingFreeKick || // deprecated 2026
          //  state == opponentPushingFreeKick ||
           state == ownThrowIn ||
           state == opponentThrowIn ||
           state == ownGoalKick ||
           state == opponentGoalKick ||
           state == ownCornerKick ||
           state == opponentCornerKick;
  }

  bool isFreeKick() const
  {
    return isFreeKick(state);
  }

  // a free kick with a well-defined ball position
  static bool isConstrainedFreeKick(State state)
  {
    return isFreeKick(state) && !isDirectFreeKick(state) && !isIndirectFreeKick(state);
  }

  bool isConstrainedFreeKick() const
  {
    return isConstrainedFreeKick(state);
  }

//   static bool isPushingFreeKick(State state)
//   {
//     return state == ownPushingFreeKick ||
//            state == opponentPushingFreeKick;
//   }
// 
//   bool isPushingFreeKick() const
//   {
//     return isPushingFreeKick(state);
//   }

  // added 2026 - replaces pushing free kick but is more than that
  static bool isIndirectFreeKick(State state)
  {
    return state == ownIndirectFreeKick ||
           state == opponentIndirectFreeKick;
  }

  bool isIndirectFreeKick() const
  {
    return isIndirectFreeKick(state);
  }

  // added 2026
  static bool isDirectFreeKick(State state)
  {
    return state == ownDirectFreeKick ||
           state == opponentDirectFreeKick;
  }

  bool isDirectFreeKick() const
  {
    return isDirectFreeKick(state);
  }


  static bool isThrowIn(State state)
  {
    return state == ownThrowIn ||
           state == opponentThrowIn;
  }

  bool isThrowIn() const
  {
    return isThrowIn(state);
  }

  static bool isGoalKick(State state)
  {
    return state == ownGoalKick ||
           state == opponentGoalKick;
  }

  bool isGoalKick() const
  {
    return isGoalKick(state);
  }

  static bool isCornerKick(State state)
  {
    return state == ownCornerKick ||
           state == opponentCornerKick;
  }

  bool isCornerKick() const
  {
    return isCornerKick(state);
  }

  static bool isPenaltyShootout(State state)
  {
    return state == beforePenaltyShootout ||
           state == waitForOwnPenaltyShot ||
           state == waitForOpponentPenaltyShot ||
           state == ownPenaltyShot ||
           state == opponentPenaltyShot ||
           state == afterOwnPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isPenaltyShootout() const
  {
    return isPenaltyShootout(state);
  }

  static bool isForOwnTeam(State state)
  {
    return state == setupOwnKickOff ||
           state == waitForOwnKickOff ||
           state == ownKickOff ||
           state == setupOwnPenaltyKick ||
           state == waitForOwnPenaltyKick ||
           state == ownPenaltyKick ||
           state == ownDirectFreeKick || // added 2026
           state == ownIndirectFreeKick || // added 2026
          //  state == ownPushingFreeKick || // deleted 2026
           state == ownThrowIn ||
           state == ownGoalKick ||
           state == ownCornerKick ||
           state == waitForOwnPenaltyShot ||
           state == ownPenaltyShot ||
           state == afterOwnPenaltyShot;
  }

  bool isForOwnTeam() const
  {
    return isForOwnTeam(state);
  }

  static bool isForOpponentTeam(State state)
  {
    return state == setupOpponentKickOff ||
           state == waitForOpponentKickOff ||
           state == opponentKickOff ||
           state == setupOpponentPenaltyKick ||
           state == waitForOpponentPenaltyKick ||
           state == opponentPenaltyKick ||
           state == opponentDirectFreeKick || // added 2026
           state == opponentIndirectFreeKick || // added 2026
          //  state == opponentPushingFreeKick || // deleted 2026
           state == opponentThrowIn ||
           state == opponentGoalKick ||
           state == opponentCornerKick ||
           state == waitForOpponentPenaltyShot ||
           state == opponentPenaltyShot ||
           state == afterOpponentPenaltyShot;
  }

  bool isForOpponentTeam() const
  {
    return isForOpponentTeam(state);
  }

  static bool isPenalized(PlayerState state)
  {
    return state == penalizedManual ||
           state == penalizedIllegalPositioning || // renamed from penalizedIllegalPosition
           state == penalizedMotionInSet || // renamed from penalizedIllegalMotionInSet
           state == penalizedMotionInStop || // added 2026
           state == penalizedLocalGameStuck ||
           state == penalizedIncapableRobot || // renamed from penalizedInactivePlayer
           state == penalizedRequestForPickup ||
           state == penalizedBallHolding || // added 2026
           state == penalizedLeavingTheField ||
           state == penalizedPlayingWithArmsHands || // possibly renamed from penalizedIllegalBallContact
           state == penalizedPushing || // renamed from penalizedPlayerPushing
           state == penalizedCautioned || // added 2026
           state == penalizedSentOff || // added 2026
          //  state == penalizedIllegalPositionInSet || // deprecated 2026
          //  state == penalizedIllegalMotionInStandby || // deprecated 2026
          //  state == penalizedPlayerStance || // deprecated 2026
           state == substitute;
  }

  bool isPenalized() const
  {
    return isPenalized(playerState);
  }

  static State stateForOtherTeam(const State state);

  State stateForOtherTeam() const
  {
    return stateForOtherTeam(state);
  }

  bool isGoalkeeper() const
  {
    return ownTeam.isGoalkeeper(playerNumber);
  }

  Team::Color color() const
  {
    return ownTeam.color(playerNumber);
  }

  int playerIndex() const { return Settings::toPlayerIndex(playerNumber); }

  std::string getPlayerNumberAndColorString() const;

  /// duration of current state excluding stopped time
  FUNCTION(int()) getStateDuration;

  /// duration of current state excluding stopped time
  FUNCTION(int()) getPlayerStateDuration;


  /**** separate header from streamable fields - comma at end of comment ****/,

  (Phase)(firstHalf) phase, ///< The current phase of the game. 
  (unsigned)(0) timeWhenPhaseEnds, ///< Time when the current phase is expected to end. 
  (State)(beforeHalf) state, ///< The current state of the game. 
  (unsigned)(0) timeWhenStateStarted, ///< Time when the current state started. 
  (unsigned)(0) timeWhenStateEnds, ///< Time when the current state is expected to end. Only valid for some states! 
  (bool)(false) kickOffSetupFromTouchlines, ///< During a kick-off setup: Is it from the touchlines (i.e. start of half / after timeout as opposed to after a goal / global game stuck)? 
  (Team) ownTeam, ///< The state of the own team. 
  (Team) opponentTeam, ///< The state of the opponent team. 
  (int)(0) playerNumber, ///< The jersey number of this player. 
  (PlayerState)(unstiff) playerState, ///< The current state of this player. 
  (unsigned)(0) timeWhenPlayerStateStarted, ///< Time when the current player state started. 
  (bool)(false) gameControllerActive, ///< Whether a GameController is active. 
  (bool)(true) leftHandTeam, ///< Whether own team defends the left goal from the GameController's PoV. 
  // (CompetitionPhase)(roundRobin) competitionPhase, ///< The phase of the tournament we are in. 
  (bool)(false) whistled, ///< State was switched due to a whistle. 
  (unsigned)(0) timeWhenPenaltyEnds, ///< Time when the own penalty is expected to end (0 -> not penalized). 
  (bool)(false) kickingTeamKnown, ///< Whether the kicking team was sent by the GameController. 
  (bool)(false) stopped, ///< Whether the GC has signalled that the game should be stopped temporarily
  (unsigned)(0) stoppedTimeInState, ///< the total amount of time that play was stopped while in the current state (could be across multiple stops)
  (unsigned)(0) stoppedTimeInPlayerState, ///< the total amount of time that play was stopped while in the current player state
});

STREAMABLE(ExtendedGameState,
{
  bool wasInitial(bool orStandby = true) const
  {
    return GameState::isInitial(stateLastFrame, orStandby);
  }

  bool wasReady() const
  {
    return GameState::isReady(stateLastFrame);
  }

  bool wasSet() const
  {
    return GameState::isSet(stateLastFrame);
  }

  bool wasPlaying() const
  {
    return GameState::isPlaying(stateLastFrame);
  }

  bool wasFinished() const
  {
    return GameState::isFinished(stateLastFrame);
  }

  bool wasFreeKick() const
  {
    return GameState::isFreeKick(stateLastFrame);
  }

  bool wasPenalized() const
  {
    return GameState::isPenalized(playerStateLastFrame);
  }

  /**** separate header from streamable fields - comma at end of comment ****/,


  (GameState::State)(GameState::beforeHalf) stateBeforeCurrent, ///< the game state before the current state - could be many frames in the past 
  (GameState::State)(GameState::beforeHalf) stateLastFrame, ///< The state of the game in the last frame. 
  (GameState::PlayerState)(GameState::unstiff) playerStateLastFrame, ///< The state of this player in the last frame. 
  (unsigned)(1200u) messageBudgetLastFrame, ///< Message budget of the own team in the last frame. 
  (bool)(false) returnFromGameControllerPenalty, ///< Is the robot returning from a GameController penalty this frame? 
  (bool)(false) returnFromManualPenalty, ///< Is the robot returning from a manual penalty this frame? 
  (unsigned)(0) timeWhenLastUnpenalized, ///< when did robot last transition from penalized to unpenalized 
  (std::array<unsigned, GameState::numOfStates>)({}) timeWhenStateStarted, ///< For each state: Time when this state started last. 
  (std::array<unsigned, GameState::numOfPlayerStates>)({}) timeWhenPlayerStateStarted, ///< For each player state: Time when this state started last. 
});
