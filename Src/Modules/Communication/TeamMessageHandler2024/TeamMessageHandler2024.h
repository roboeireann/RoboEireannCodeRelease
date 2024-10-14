/**
 * @file TeamMessageHandler2024.h
 *
 * This module converts between various representations and the
 * message sent between teammates (which was the SPLStandardMessage up to 2022).
 * It handles both encoding and decoding of messages to a more compressed form.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Communication/TeamMessageSocketHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts (might as well, though we don't really support windows)

#include "Representations/Configuration/GameConfig.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/SetupPoses.h"

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

#include "Representations/Modeling/TeammatesLocationModel.h"

#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"

#include "Representations/Infrastructure/ExtendedGameInfo.h"

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"


#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/Perception/StandbyReadyGesture.h"

#include "Representations/Communication/TeamMessage2024.h"
#include "Representations/Communication/MessageDesirability.h"

#include "Tools/Communication/TimeSyncInfo.h"

#include "Tools/Module/Module.h"



MODULE(TeamMessageHandler2024,
{,
  // for calculations
  REQUIRES(GameConfig),
  REQUIRES(BallSpecification),
  REQUIRES(SetupPoses),

  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(WalkingEngineOutput),
  USES(GameInfo), // includes whistle based guesses
  USES(ReceivedGameControlData), // as it came from the game controller
  USES(ExtendedGameInfo),
  USES(MotionRequest),
  USES(TeammatesLocationModel),  

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),

  // send directly
  USES(RobotPose),
  USES(BallModel),
  USES(TeamBallModel),
  USES(BehaviorStatus),
  USES(TeamBehaviorStatus),
  USES(ActiveTactic), // only the ActiveTacticStatus subset gets sent
  USES(ObstacleModel),
  // USES(RobotHealth),
  USES(Whistle),
  USES(StandbyReadyGesture),

  // REQUIRES(ReceivedTeamMessages2024),
  // PROVIDES(ReceivedTeamMessages2024)
  REQUIRES(TeamData), // forces update(TeamData) to run first
  PROVIDES(TeamData), // note that TeamData will only include info about teammates
  PROVIDES(TeamMessage2024OutputGenerator),
  PROVIDES(SentTeamMessage2024),
  PROVIDES(MessageDesirability),

  LOADS_PARAMETERS(
  {,
    (bool) sendAtMinPlayerInterval, ///< if true, this is like the old behaviour of 1 message per robot per interval
    (int) maxPlayerSendInterval, /**< max time in ms between messages from a single robot to prevent network timeout */
    (int) minPlayerSendInterval, /**<  [ms] between two messages that are sent to the teammates by a single robot */
    (int) maxPlayerSendIntervalJitter, ///< [ms] randomly jitter maxPlayerSendInterval by +/- this amount before sending
    (int) intervalJitter, ///< [ms] randomly jitter intervals by this +/- amount so all robots do not send simultaneously
    (int) messageRateExtraSecs, ///< extra secs that artifically increase the message budget until near the end of the half, e.g. to allow messages to be sent in ready
    (int) networkTimeout, /**< [ms] time after which teammates are considered as disconnected */
    (unsigned) minBallSeenPercentage, ///< the min ball seen percentage to be valid
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
 * @class TeamMessageHandler2024
 * A modules for sending some representation to teammates
 */

class TeamMessageHandler2024 : public TeamMessageHandler2024Base
{
public:
  TeamMessageHandler2024();

private:
  struct ReceivedTeamMessage : public TeamMessage2024
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

  ReceivedTeamMessageBytesBuffer inputTeamMessageBytesBuffer;
  TeamMessageBytes outputTeamMessageBytes;
  TeamMessageSocketHandler teamMessageSocketHandler;

  bool isUpright = true;
  unsigned frameTimeWhenLastUpright = 0;


  // --------------------------------------------------------------------------
  // output stuff
  // --------------------------------------------------------------------------

  mutable SentTeamMessage2024 sentTeamMessage;

  void update(TeamMessage2024OutputGenerator& outputGenerator) override;
  void update(SentTeamMessage2024 &theSentTeamMessage) override { theSentTeamMessage = sentTeamMessage; }

  /** 
   * populate the team message fields based on our required representations 
   */
  void generateOutputMessage(TeamMessage2024OutputGenerator& outputGenerator) const;
  
  /**
   * convert the team message into its encoded/compressed bytes for transmission
   */
  void writeOutputMessageBytes(TeamMessage2024OutputGenerator& outputGenerator, TeamMessageBytes& teamMessageBytes) const;
  void checkOutputMessageBytes(const TeamMessage2024& originalMessage, const TeamMessageBytes& msgBytes) const; // for debugging the decoding only

  void sendMessageBytes(TeamMessage2024OutputGenerator& outputGenerator, const TeamMessageBytes& teamMessageBytes);

  // --------------------------------------------------------------------------
  // input stuff
  // --------------------------------------------------------------------------

  unsigned timeLastAlert = 0;

  ReceivedTeamMessage receivedTeamMessage; // temporary received team message, gets overwritten for each message
  unsigned lastGCPacketReceivedTime = 0;

  void update(TeamData& teamData) override; // team data will only include info about teammates, not self
  // void initTeammatesFromGameInfo(TeamData& teamData);
  // void initTeammateFromGameInfo(Teammate& teammate);
  // void updateTeammatesFromGameInfo(TeamData& teamData);
  void maintainTeammateList(TeamData& teamData) const;
  
  /// @brief encoded message bytes decoded to a ReceivedTeamMessage
  bool readInputMessageBytes(const ReceivedTeamMessageBytes& msgBytes, ReceivedTeamMessage& msg);

  Teammate& getTeammate(TeamData& teamData, int playerNum) const;
  void parseMessageIntoTeammate(const ReceivedTeamMessage& msg, Teammate& teammate);


  // ==========================================================================
  // Message Desirability functionality
  // ==========================================================================

  MessageDesirability messageDesirability;
  
  bool messageDesirabilityUpdated = false;
  Formations::FormationRole prevFormationRole = Formations::noFormationRoleSpecified;
  PlayerRole::Type prevSituationRole = PlayerRole::none;
  bool wasPenalized = false;
  unsigned lastTeamMessageTime = 0;
  mutable bool kickedSinceRestart = false;
  mutable bool prevFrameKickedSinceRestart = false;

  void update(MessageDesirability &theMessageDesirability) override
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

  // utility function to help with frame time wrap arounds
  bool timeSinceTimeout(unsigned timeToCheck, int timeout, const char* var = nullptr);

  unsigned addJitter(unsigned time, unsigned jitterAmount);
};
