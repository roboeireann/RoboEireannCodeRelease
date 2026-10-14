/**
 * @file RERETeamMessageHandler2025.h
 *
 * This module converts between various representations and the
 * message sent between teammates (which was the SPLStandardMessage up to 2022).
 * It handles both encoding and decoding of messages to a more compressed form.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Aidan Colgan (modifications)
 * @author Rudi Villing (modifications)
 * @note This file is part of the B-Human 2021 Code Release modified by us
 * @copyright 2025-2026 RoboEireann
 */

#pragma once

// #include "Tools/Communication/TeamMessageSocketHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts (might as well, though we don't really support windows)
#include "Tools/Communication/TeamMessageChannel.h" // include this first to prevent WinSock2.h/Windows.h conflicts

#include "Representations/Configuration/REGameConfig.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/SetupPoses.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

#include "Representations/Modeling/RETeammatesLocationModel.h"

#include "Representations/Communication/RETeamData.h"
#include "Representations/Communication/GameControllerData.h"

#include "Representations/Infrastructure/GameState.h"

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"


#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Perception/RefereeGestures/REStandbyReadyGesture.h"
#include "Representations/Perception/RefereeGestures/RESetPlayRefereeGesture.h"

#include "Representations/Communication/RETeamMessage2025.h"
#include "Representations/Communication/REMessageDesirability.h"

#include "Tools/RECommunication/TimeSyncInfo.h"

#include "Framework/Module.h"



MODULE(RETeamMessageHandler2025b,
{,
  // for calculations
  REQUIRES(REGameConfig),
  REQUIRES(BallSpecification),
  REQUIRES(SetupPoses),

  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(WalkingEngineOutput),
  USES(GameControllerData), // as it came from the game controller
  
  USES(GameState), // replaces REGameInfo sometimes because of way states have changed
  USES(ExtendedGameState),
  USES(MotionRequest),
  USES(RETeammatesLocationModel),  

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),

  // send directly
  USES(RobotPose),
  USES(BallModel),
  USES(TeamBallModel),
  USES(BehaviorStatus),
  USES(TeamBehaviorStatus),
  USES(REActiveTactic), // only the REActiveTacticStatus subset gets sent
  USES(ObstacleModel),
  // USES(RobotHealth),
  USES(Whistle),
  USES(REStandbyReadyGesture),
  USES(RESetPlayRefereeGesture),

  // REQUIRES(ReceivedTeamMessages2025),
  // PROVIDES(ReceivedTeamMessages2025)
  REQUIRES(RETeamData), // forces update(RETeamData) to run first
  PROVIDES(RETeamData), // note that RETeamData will only include info about teammates
  PROVIDES(RETeamMessage2025OutputGenerator),
  PROVIDES(RESentTeamMessage2025),
  PROVIDES(REMessageDesirability),

  LOADS_PARAMETERS(
  {,
    (bool) sendAtMinPlayerInterval, ///< if true, this is like the old behaviour of 1 message per robot per interval
    (int) maxPlayerSendInterval, /**< max time in ms between messages from a single robot to prevent network timeout */
    (int) minPlayerSendInterval, /**<  Time in ms between two messages that are sent to the teammates by a single robot */
    // (int) minTeamSendInterval, /**<  Time in ms between two messages that are sent to the teammates by any player */
    (int) maxPlayerSendIntervalJitter, ///< [ms] randomly jitter maxPlayerSendInterval by +/- this amount before sending
    (int) intervalJitter, ///< [ms] randomly jitter intervals by this +/- amount so all robots do not send simultaneously
    (int) messageRateExtraSecs, ///< extra secs that artifically increase the message budget until near the end of the half, e.g. to allow messages to be sent in ready
    (int) networkTimeout, /**< Time in ms after which teammates are considered as unconnected (in states where communication is expected) */
    (unsigned) minBallSeenPercentage, ///< the min ball see percentage to be valid
    (int) ballDisappearedTimeout, ///< the maximum time since the ball should have been visible in the image to be valid
    (int) ballLastSeenTimeout, ///< the maximum time since the ball was seen to be valid
    (float) ballMovementThreshold, ///< above this value the ball is considered to have moved enough to deserve an update
    (int) maxTeamBallSendInterval, ///< should be less than ballLastSeenTimeout in teamBallLocator.cfg
    (int) maxBallPlayerBallSendInterval, ///< should be less than maxTeamBallSendInterval and give enough time for team communication before maxTeamBallSendInterval times out
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
    (int) minDistanceLastTeamBall, /**<Minimum distance from last team ball for local ball to be notable*/
    (int) returnFromPenaltyDelay, /**< time after returning from penalty before sending message */
    (int) maxTimeWithoutMessageBeforePlaying, /**< send a message if more than this time has elapsed on entry to playing state */
  }),
});









/**
 * @class RETeamMessageHandler2025
 * A modules for sending some representation to teammates
 */

class RETeamMessageHandler2025b : public RETeamMessageHandler2025bBase
{
  using TimeSyncInfo = RE::TimeSyncInfo;
  
public:
  RETeamMessageHandler2025b();

private:
  struct ReceivedTeamMessage : public RETeamMessage2025
  {
    enum ErrorCode
    {
      //add more parsing errors if there is a need of distinguishing
      parsingError,
      magicNumberDidNotMatch,
      myOwnMessage,
      timeSyncFailure
    };
    
    ErrorCode lastErrorCode;

    unsigned timeReceived;    
    unsigned frameTimeReceived;    
  };
  
  TimeSyncInfo timeSyncInfo;

  // ReceivedTeamMessageBytesBuffer inputTeamMessageBytesBuffer;
  // TeamMessageBytes outputTeamMessageBytes;
  // TeamMessageSocketHandler teamMessageSocketHandler;

  TeamMessageChannel::Container inputTeamMessageContainer;
  TeamMessageChannel::Container outputTeamMessageContainer;
  TeamMessageChannel teamMessageChannel;


  bool isUpright = true;
  unsigned frameTimeWhenLastUpright = 0;

  unsigned sentMessagesLogged = 0;
  unsigned sentMessagesLogTime = 0;
  unsigned receivedMessagesLogged = 0;
  unsigned receivedMessagesLogTime = 0;
  int receivedMessagePlayerNum = -1;


  unsigned timeOfMostRecentRestart() const;

  // --------------------------------------------------------------------------
  // output stuff
  // --------------------------------------------------------------------------

  mutable RESentTeamMessage2025 sentTeamMessage;

  void update(RETeamMessage2025OutputGenerator& outputGenerator) override;
  void update(RESentTeamMessage2025 &theSentTeamMessage) override { theSentTeamMessage = sentTeamMessage; }

  /** 
   * populate the team message fields based on our required representations 
   */
  void generateOutputMessage(RETeamMessage2025OutputGenerator& outputGenerator) const;
  
  /**
   * convert the team message into its encoded/compressed bytes for transmission
   */
  // void writeOutputMessage(RETeamMessage2025OutputGenerator& outputGenerator, TeamMessageBytes& teamMessageBytes) const;
  // void checkOutputMessage(const RETeamMessage2025& originalMessage, const TeamMessageBytes& msgBytes) const; // for debugging the decoding only
  // void sendMessage(RETeamMessage2025OutputGenerator& outputGenerator, const TeamMessageBytes& teamMessageBytes);
  void writeOutputMessageBytes(RETeamMessage2025OutputGenerator& outputGenerator, TeamMessageChannel::Container& teamMessageBytes) const;
  void checkOutputMessageBytes(const RETeamMessage2025& originalMessage, const TeamMessageChannel::Container& msgBytes) const; // for debugging the decoding only
  void sendMessageBytes(RETeamMessage2025OutputGenerator& outputGenerator);

  // --------------------------------------------------------------------------
  // input stuff
  // --------------------------------------------------------------------------

  unsigned timeLastAlert = 0;

  ReceivedTeamMessage receivedTeamMessage; // temporary received team message, gets overwritten for each message
  unsigned lastGCPacketReceivedTime = 0;

  void update(RETeamData& teamData) override; // team data will only include info about teammates, not self
  void maintainTeammateList(RETeamData& teamData) const;
  
  /// @brief encoded message bytes decoded to a ReceivedTeamMessage
  // bool readInputMessageBytes(const ReceivedTeamMessageBytes& msgBytes, ReceivedTeamMessage& msg);
  bool readInputMessageBytes(const TeamMessageChannel::Container& msgBytes, ReceivedTeamMessage& msg);

  RETeammate& getTeammate(RETeamData& teamData, int playerNum) const;
  void parseMessageIntoTeammate(const ReceivedTeamMessage& msg, RETeammate& teammate);


  // ==========================================================================
  // Message Desirability functionality
  // ==========================================================================

  REMessageDesirability messageDesirability;
  
  bool messageDesirabilityUpdated = false;
  REFormations::FormationRole prevFormationRole = REFormations::noFormationRoleSpecified;
  PlayerRole::Type prevSituationRole = PlayerRole::none;
  bool wasPenalized = false;
  unsigned lastTeamMessageTime = 0;
  mutable bool kickedSinceRestart = false;
  mutable bool prevFrameKickedSinceRestart = false;
  unsigned frameTimeLastKickingTeamUpdate = 0;
  uint8_t prevKickingTeam = KICKING_TEAM_NONE;

  void update(REMessageDesirability &theMessageDesirability) override
  {
    if (!messageDesirabilityUpdated)
      updateMessageDesirability();
      
    theMessageDesirability = messageDesirability;
  }

  void updateMessageDesirability(); ///< updates the internal messageDesirability
  void updateLastTeamMessageTime();
  void updateMessageRate();

  bool checkMessageBudgetAllowsSending();
  bool checkStateAllowsSending();
  bool checkMessageRateAllowsSending();

  bool checkReadySetPlay();

  bool checkPingNeeded();
  bool checkBallMoved();
  bool checkRoleChanged();
  bool checkPlayerMoved();
  bool checkReturnFromPenalty(); //Player back from penalty and should be added to teammate list
  bool checkUprightStatusChanged();
  bool checkStandbyReadyGesture();
  bool checkKickedSinceRestart();
  bool checkSetPlayRefereeGesture();
  
  // utility function to help with frame time wrap arounds
  bool timeSinceTimeout(unsigned timeToCheck, int timeout, const char* var = nullptr);

  unsigned addJitter(unsigned time, unsigned jitterAmount);
};
