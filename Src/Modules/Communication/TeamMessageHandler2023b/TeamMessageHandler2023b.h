/**
 * @file TeamMessageHandler2023b.h
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

#include "TeamMessageSocketHandler.h" // include this first to prevent WinSock2.h/Windows.h conflicts (might as well, though we don't really support windows)

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"

#include "Representations/Infrastructure/ExtendedGameInfo.h"

#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Representations/Communication/RobotInfo.h"

#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/Whistle.h"

#include "Representations/Communication/TeamMessage2023.h"
#include "Representations/Communication/MessageDesirability.h"

#include "Tools/Communication/TimeSyncInfo.h"

#include "Tools/Module/Module.h"



MODULE(TeamMessageHandler2023b,
{,
  // for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  USES(OwnTeamInfo),
  USES(GameInfo), // includes whistle based guesses
  USES(RawGameInfo), // as it came from the game controller
  USES(ExtendedGameInfo),
  USES(MotionRequest),

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  USES(RobotInfo),

  // send directly
  USES(RobotPose),
  USES(BallModel),
  USES(TeamBallModel),
  USES(BehaviorStatus),
  USES(TeamBehaviorStatus),
  USES(ObstacleModel),
  // USES(RobotHealth),
  USES(Whistle),

  REQUIRES(TeamData), // forces update(TeamData) to run first
  PROVIDES(TeamData),
  PROVIDES(TeamMessage2023OutputGenerator),
  PROVIDES(SentTeamMessage2023),
  PROVIDES(MessageDesirability),

  LOADS_PARAMETERS(
  {,
    (unsigned) totalMessageBudget, /**< assumed message budget for entire game */
    (int) maxSendInterval, /**< max time in ms between messages to prevent network timeout */
    (int) minSendInterval, /**<  Time in ms between two messages that are sent to the teammates */
    (int) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
    (int) minDistanceLastTeamBall, /**<Minimum distance from last team ball for local ball to be notable*/
    (int) returnFromPenaltyDelay, /**< time after returning from penalty before sending message */
    (int) maxTimeWithoutMessageBeforePlaying, /**< send a message if more than this time has elapsed on entry to playing state */
  }),
});









/**
 * @class TeamMessageHandler2023
 * A modules for sending some representation to teammates
 */

class TeamMessageHandler2023b : public TeamMessageHandler2023bBase
{
public:
  TeamMessageHandler2023b();

private:
  struct ReceivedTeamMessage : public TeamMessage2023
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
  };
  
  TimeSyncInfo timeSyncInfo;

  ReceivedTeamMessageBytesBuffer inputTeamMessageBytesBuffer;
  TeamMessageBytes outputTeamMessageBytes;
  TeamMessageSocketHandler teamMessageSocketHandler;


  // --------------------------------------------------------------------------
  // output stuff
  // --------------------------------------------------------------------------

  mutable SentTeamMessage2023 sentTeamMessage;

  void update(TeamMessage2023OutputGenerator& outputGenerator) override;
  void update(SentTeamMessage2023 &theSentTeamMessage) override { theSentTeamMessage = sentTeamMessage; }

  /** 
   * populate the team message fields based on our required representations 
   */
  void generateOutputMessage(TeamMessage2023OutputGenerator& outputGenerator) const;
  
  /**
   * convert the team message into its encoded/compressed bytes for transmission
   */
  void writeOutputMessageBytes(TeamMessage2023OutputGenerator& outputGenerator, TeamMessageBytes& teamMessageBytes) const;

  void sendMessageBytes(TeamMessage2023OutputGenerator& outputGenerator, const TeamMessageBytes& teamMessageBytes);

  // --------------------------------------------------------------------------
  // input stuff
  // --------------------------------------------------------------------------

  unsigned timeLastAlert = 0;

  ReceivedTeamMessage receivedTeamMessage; // temporary received team message, gets overwritten for each message

  void update(TeamData& teamData) override;
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
  int previousStriker = -1;
  bool wasPenalized = false;

  void update(MessageDesirability &theMessageDesirability) override
  {
    if (!messageDesirabilityUpdated)
      updateMessageDesirability();
      
    theMessageDesirability = messageDesirability;
  }

  void updateMessageDesirability(); ///< updates the internal messageDesirability
  bool checkMessageBudgetAllowsSending();
  bool checkStateAllowsSending();
  bool checkMinMessageIntervalAllowsSending();

  bool checkReadySetPlay();

  bool checkPingNeeded();
  bool checkBallMoved();
  bool checkRoleChanged();
  bool checkPlayerMoved();
  bool checkReturnFromPenalty(); //Player back from penalty and should be added to teammate list
};
