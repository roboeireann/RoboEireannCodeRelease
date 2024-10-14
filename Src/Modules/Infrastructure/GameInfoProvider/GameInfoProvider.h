/**
 * @file GameInfoProvider.h
 * 
 * Modifies the GameInfo to handle state changes by means other than the
 * received game control messages.
 * 
 * Based in part on WhistleHandler and KickoffStateProvider from the 
 * B-Human Code Release 2021.
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/GameConfig.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Sensing/GyroState.h"
#include "Representations/Modeling/BallInGoal.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Perception/StandbyReadyGesture.h"

#include "Tools/RingBuffer.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Module/Module.h"
#include "Tools/EdgeTrigger.h"

MODULE(GameInfoProvider,
{,
  USES(BehaviorStatus),
  REQUIRES(ReceivedGameControlData),
  REQUIRES(TeamData),
  REQUIRES(BallSpecification),
  REQUIRES(FieldDimensions),
  REQUIRES(GameConfig),
  REQUIRES(FrameInfo),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(GyroState),
  REQUIRES(BallInGoal),
  USES(BallModel),
  USES(RobotPose),
  USES(TeamBallModel),
  REQUIRES(Whistle),
  REQUIRES(MotionInfo),
  REQUIRES(StandbyReadyGesture),

  PROVIDES(GameInfo),

  LOADS_PARAMETERS(
  {,
    (int) gameControllerTimeout, /**< this should match or be longer than GameDataProvider::gameControllerTimeout. */

    // button parameters from (originally from GameDataProvider)
    (unsigned) unstiffHeadButtonPressDuration, /**< How long the head buttons need to be pressed until the robot transitions to unstiff (in ms). */
    (unsigned) calibrationHeadButtonPressDuration, /**< How long the front head button needs to be pressed until the robot transitions to calibration (in ms). */
    (int) finishedToUnstiffDuration, /**< How long the game state needs to be finished until the robot transitions to unstiff (in ms). */

    // whistle related parameters (originally from WhistleHandler)
    (bool) useWhistleForKickOff, ///< whether to use the whistle for kickoff and penalty kicks
    (bool) useWhistleAfterGoal, /**< Whether the whistle can switch the state to ready. */
    (int) ignoreWhistleDuration, ///< the min time after one whistle before another can be recognised
    (int) gameControllerOperatorDelay, /**< Max expected delay before the GC operator enters the referee's decisions. */
    // (int) acceptPastWhistleDelay, /**< [????] How old can whistles be to be accepted after canceling READY (in ms)? */
    (bool) sayGuessedStateChange, ///< for debugging when whistle-based (guessed) state changes occur
    (bool) requireOwnWhistle, ///< must hear whistle directly to react
    (int) requiredWhistleConsensus, ///< min number of whistles needed to react (including own if required) 
    (int) whistleMatchDuration, ///< time window over which we consider whistles to match (this should include any expected communication delay)

    // ball related parameters
    (bool) useBallInGoalCheck, /**< Consider ball position when checking whether a goal was announced. */
    (int) acceptBallInGoalDelay, /**< Max time (ms) since ball must have been seen in a goal, to accept as a valid goal when referee whistles */

    (bool) useVisualReady, ///< whether to use the visual ready signal if it is detected
    (int) visualReadyMatchDuration, ///< The last time limit if a gesture detection was seen by other robots.

    // kickoff and set-play related parameters (originally from KickoffStateProvider)
    (int) ballOutOfCenterCircleCounterThreshold,   /**< The number of times the ball needs to be seen at a certain position */
    (float) ballOutOfCenterCircleTolerance,        /**< Distance added to center circle radius for determining, if the ball is out of the center circle */
    // (float) robotOutOfCenterCircleTolerance,       /**< Distance added to center circle radius for determining, if the robot is close enough to decide whether the ball is out of the center circle */
    (int) ballSaveInterval,                        /**< Time between two ball measurements that are added to the buffer */
    (float) ballMovedTolerance,                 /**< Distance away from kick-off point the ball needs to be seen to be considered as moved */
    (float) ballMovedCloseToRobotThreshold,     /**< If the ball is less than a center circle radius minus this threshold away it is assumed to have been moved by the opponent. */
    (float) standStillGyroThreshold,               /**< maximum Value of the deviation of the y-gyro to assume that the robot stands still. */
  }),
});

class GameInfoProvider : public GameInfoProviderBase
{
private:
  bool initialised = false;

  unsigned timeWhenStateChanged = 0; /**< The time when the effective game state last changed. */
  unsigned timeWhenReceivedStateChanged = 0; ///< time when the state last changed in the raw game control data
  unsigned timeLastPacketReceived = 0;
  // unsigned prevFrameReceivedGameState = STATE_INITIAL; /**< The previous raw game state. */
  EdgeTriggeru receivedGameStateTrigger { STATE_INITIAL };
  // uint8_t guessedGameState = STATE_NOT_GUESSED; /**< The guessed game state. Use if different from STATE_NOT_GUESSED. */
  unsigned kickingTeam = 0; /**<  The guessed number of the team that has kick-off. */
  std::vector<unsigned> illegalMotionTimes; /**< The times when players (both teams) were last penalized for Illegal Motion. */
  bool anyIllegalMotion = false; ///< true if any player is done for illegal motion in SET or STANDBY while we are in a guessed state
  unsigned lastWhistleTime = 0;

  // used to help kickoff and ball free tracking
  // unsigned prevFrameSetPlay = SET_PLAY_NONE;
  EdgeTriggeru receivedSetPlayTrigger { SET_PLAY_NONE };
  unsigned kickoffStartTime = 0;
  unsigned setPlayStartTime = 0;

  // for button handling
  bool ignoreChestButton = false;
  unsigned whenNotInFinishedState = 0; ///< most recent frame time that the state was not FINISHED

  // from KickoffStateProvider
  // int ballOutOfCenterCircleCounter;       ///< Filtered Number of recent ball sightings out of center circle in playing state
  RingBuffer<Vector2f, 30> ballPositions; ///< Buffering the ball positions during opponent's kickoff for computing a possible ball motion
  unsigned int lastBallPositionTime;      ///< Point of time when the ballPositions buffer has been changed (by adding) the last time
  bool ballMoved = true;                  ///< is the ball free (in a kickoff or set play)
  // bool ballOutOfCenterCircle;             ///< Are we allowed to score a goal?


  /**
   * update the game info based on the receivedGameControlData, whistle,
   * ball movement, visual referee signals, etc.
   * @param theGameInfo The representation updated.
   */
  void update(GameInfo& theGameInfo) override;

  // reset the contents of the game info back to a default state
  void resetGameInfo(GameInfo& gameInfo);

  /** Handle the official button interface for changing the player mode. */
  void handlePlayerModeButtons(GameInfo& gameInfo);

  /** Handle the official button interface for manual config and state changes. */
  void handleManualStateButtons(GameInfo& gameInfo);

  /** copy across data from received game controller packet, but save our existing overrides (e.g. due to whistle) */
  void updateGameInfoFromReceivedGameControlData(GameInfo& gameInfo);

  void updateBallMotion(GameInfo& gameInfo);
  void resetBallMotion();

  // /** check if the ball has been moved in kickoff or a set play */
  // bool checkBallHasMoved();
  void updateKickoffAndSetPlayBallFree(GameInfo& gameInfo);
  void startKickoff(GameInfo& gameInfo);
  void startSetPlay(GameInfo& gameInfo);
  void setBallFree(GameInfo& gameInfo);

  void updateIllegalMotionPenalties(GameInfo& gameInfo);

  void updateEndOfGuessedState(GameInfo& gameInfo);

  /** check if the whistle was heard by this robot (and perhaps teammates also) */
  bool checkWhistle(GameInfo& gameInfo);
  
  /** check if this robot or any teammate signalled visual ready */
  bool checkVisualReadySignalled(GameInfo& gameInfo);

  void updateGuessedState(GameInfo& gameInfo);

  /**
   * Check whether at least one player received a Illegal-Motion-in-Set-penalty since the last
   * change of the game state.
   */
  bool checkForIllegalMotionPenalty();

  /**
   * Check whether the ball was considered to be in the goal during the last
   * acceptBallInGoalDelay ms.
   * @return true if it was
   */
  bool checkBallInGoal();
};
