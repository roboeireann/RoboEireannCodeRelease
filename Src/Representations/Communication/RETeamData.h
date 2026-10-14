/**
 * @file RETeamData.h
 *
 * Representation to keep track of data communicated from teammates via team comms
 * 
 * Partly based on RETeamData from BH2021 code release but now modified quite a bit
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/REActiveTactic.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Perception/RefereeGestures/REStandbyReadyGesture.h"
#include "Representations/Perception/RefereeGestures/RESetPlayRefereeGesture.h"

#include "Framework/Settings.h"

#include "Streaming/AutoStreamable.h"
// #include "Streaming/Function.h"
#include "Streaming/Enum.h"

// #include "Tools/RECommunication/TimeSyncInfo.h"

STREAMABLE(RETeammate, // COMMA public MessageHandler
{
  Vector2f getEstimatedPosition(unsigned time) const;

  ENUM(Status,
  {,
    PENALIZED,                        /** OK   : I receive packets, but robot is penalized */
    FALLEN,                           /** GOOD : Robot is playing but has fallen or currently no ground contact */
    PLAYING,                          /** BEST : RETeammate is standing/walking and has ground contact :-) */
    ABSENT, //< the teammate with the specified number is not in the game
  });

  int playerIndex() { return number - Settings::lowestValidPlayerNumber; }

  /* ----------- streamable fields follow, not comma at end of this comment -------------*/,

  (int)(-1) number,
  (bool)(false) isGoalkeeper, /**< This is for a teammate what \c theGameState.isGoalkeeper() is for the player itself. */
  (bool)(true) isPenalized,
  (bool)(true) isUpright,

  (bool)(false) kickedSinceRestart,

  (unsigned)(0) timeWhenLastPacketSent,
  (unsigned)(0) timeWhenLastPacketReceived,
  (unsigned)(0) timeWhenLastUpdated, // the info might be updated by something other than a packet being received
  (Status)(PENALIZED) status,
  (unsigned)(0) timeWhenStatusChanged,

  (signed char)(0) sequenceNumber, // DEPRECATED and not populated from TeamMessageHandler2023b onwards
  (signed char)(0) returnSequenceNumber, // DEPRECATED and not populated from TeamMessageHandler2023b onwards

  (RobotPose) theRobotPose,
  (BallModel) theBallModel,
  (FrameInfo) theFrameInfo,
  (ObstacleModel) theObstacleModel,
  (BehaviorStatus) theBehaviorStatus,
  (Whistle) theWhistle, // partly guessed info
  (REStandbyReadyGesture) theREStandbyReadyGesture, // partly guessed info
  (RESetPlayRefereeGesture) theRESetPlayRefereeGesture, // partly guessed info
  (TeamBehaviorStatus) theTeamBehaviorStatus, // DEPRECATED and mainly not used from 2024 onwards (only partially filled in when used)
  (REActiveTacticStatus) theREActiveTacticStatus, // supersedes TeamBehaviorStatus
});



/**
 * @struct TeammateData
 * Collection of teammate information
 */
STREAMABLE(RETeamData,
{
  // static constexpr std::size_t numSequenceNumbers = Settings::highestValidPlayerNumber - Settings::lowestValidPlayerNumber + 1,
  std::vector<RETeammate*> activeTeammates; /**< An unordered(!) vector of all teammates that are NOT PENALIZED - points to teammates element */
  std::vector<RETeammate*> teammatesByNumber; /**< An ordered vector of all possible teammates by teammate.number - points to teammates element or nullptr if not known */

  int countActiveTeammatesKickedSinceRestart() const; ///< only count teammates, not self

  void draw() const;

  void onRead();

  /* ------ streamable members follow (note comma at end of comment) ------- */,

  (std::vector<RETeammate>) teammates, /**< An unordered(!) vector of all teammates that are in the game and not timed out */
  (unsigned)(0)receivedMessages,   /**< The number of received (not self) team messages in total */

  (std::array<unsigned, Settings::highestValidPlayerNumber>)
      teammateMessageTimestamps, /**< The timestamps of the last received team messages. */
  (std::array<unsigned, Settings::highestValidPlayerNumber>)
      teammateMessageCounts, /**< The number of messages received from each teammate */
});
