/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file TeamMessageHandler2023.h
 *
 * the module that provide manages sending and receiving of team messages
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#pragma once

#include "SPLMessageHandler.h"
// #include "MessageDesirabilityHelper.h"

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeamTalk.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"
#include "Representations/Communication/BHumanMessage.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Communication/BNTP.h"
#include "Tools/Communication/CompressedTeamCommunicationStreams.h"
#include "Tools/Communication/RobotStatus.h"
#include "Tools/Communication/TimeSyncInfo.h"

#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Communication/MessageDesirability.h"
#include "Representations/Modeling/BallModel.h"

MODULE(TeamMessageHandler2023,
{,
  // for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  USES(OwnTeamInfo),
  USES(MotionRequest),
  USES(RawGameInfo),
  USES(GameInfo),
  USES(ExtendedGameInfo),
  USES(TeamData),
  USES(MessageDesirability),

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  USES(RobotInfo),

  // send directly
  USES(BallModel),
  USES(TeamBallModel),
  USES(BehaviorStatus),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(TeamBehaviorStatus),
  USES(Whistle),

  PROVIDES(BHumanMessageOutputGenerator),
  PROVIDES(TeamData),
  PROVIDES(MessageDesirability),

  LOADS_PARAMETERS(
  {,
    (unsigned) totalMessageBudget, /**< assumed message budget for entire game */
    (int) maxSendInterval, /**< max time in ms between messages to prevent network timeout */
    (int) maxSendIntervalReadySet, /**< max send interval before ping needed in ready and set state */
    (int) minSendInterval, /**<  min Time in ms between two messages that are sent to the teammates */
    (int) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
    (int) minDistanceLastTeamBall, /**<Minimum distance from last team ball for local ball to be notable*/
  }),
});



class TimeSyncInfoParticle : public TimeSyncInfo, public BHumanMessageParticle<MessageID::undefined>
{
public:
  // writing to message
  void operator>>(BHumanMessage& m) const override
  {
    if(gameControllerInfos.empty())
    {
      m.theBHumanStandardMessage.seqNumGC = 0;
      m.theBHumanStandardMessage.timeReceivedGC = 0;
    }
    else
    {
      m.theBHumanStandardMessage.seqNumGC = gameControllerInfos.front().seqNumGC;
      m.theBHumanStandardMessage.timeReceivedGC = gameControllerInfos.front().timeReceivedGC;
    }
  }

  // reading from message
  void operator<<(const BHumanMessage& m) override
  {
    if (m.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
        m.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
      return;

    updateTeammateInfo(m.theBSPLStandardMessage.playerNum, m.theBHumanStandardMessage.timestamp,
                       m.theBHumanStandardMessage.seqNumGC, m.theBHumanStandardMessage.timeReceivedGC);
  }
};





/**
 * @class TeamMessageHandler2023
 * A modules for sending some representation to teammates
 */

class TeamMessageHandler2023 : public TeamMessageHandler2023Base
{
public:
  TeamMessageHandler2023();

private:
  struct ReceivedBHumanMessage : public BHumanMessage
  {
    // const SynchronizationMeasurementsBuffer* bSMB = nullptr;
//     const TimeSyncInfo::TeammateInfo* timeSyncTeammateInfo = nullptr;
// 
//     unsigned toLocalTimestamp(unsigned remoteTimestamp) const override
//     {
//       // if(bSMB)
//       //   // return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
//       //   return timeSyncInfo.estimateLocalTime(remoteTimestamp, theBSPLStandardMessage.playerNum);
//       // else
//       //   return 0u;
//       return timeSyncTeammateInfo ? timeSyncTeammateInfo->estimateLocalTime(remoteTimestamp) : 0u;
//     };

    enum ErrorCode
    {
      //add more parsing errors if there is a need of distinguishing
      parsingError,
      magicNumberDidNotMatch,
      myOwnMessage,
      timeSyncFailure
    } lastErrorCode;
  };
  
  SPLMessageHandler::Buffer inTeamMessages;
  RoboCup::SPLStandardMessage outTeamMessage;
  SPLMessageHandler theSPLMessageHandler;

  TimeSyncInfoParticle theTimeSyncInfo;
  mutable RobotStatus theRobotStatus;

  CompressedTeamCommunication::TypeRegistry teamCommunicationTypeRegistry;
  const CompressedTeamCommunication::Type* teamMessageType;

  // --------------------------------------------------------------------------
  // output stuff
  // --------------------------------------------------------------------------

  void update(BHumanMessageOutputGenerator& outputGenerator) override;

  void generateMessage(BHumanMessageOutputGenerator& outputGenerator) const;
  void writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage& m) const;
  void sendMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage& m);

  // --------------------------------------------------------------------------
  // input stuff
  // --------------------------------------------------------------------------

  ReceivedBHumanMessage receivedMessageContainer; // temporary container while processing a received message
  unsigned timeWhenLastMimimi = 0;

  static void regTeamMessage();

  void update(TeamData& teamData) override;

  void maintainBMateList(TeamData& teamData) const;
  bool readSPLStandardMessage(const ReceivedSPLStandardMessage& m);
  Teammate& getBMate(TeamData& teamData) const;
  void parseMessageIntoBMate(Teammate& bMate);


  // ==========================================================================
  // Message Desirability functionality
  // ==========================================================================

  MessageDesirability messageDesirability;
  
  unsigned frameTimeLastSent = 0;
  int previousStriker = -1;
  bool wasPenalized = false;

  void update(MessageDesirability &theMessageDesirability) override { theMessageDesirability = messageDesirability; }

  bool shouldSendThisFrame(); ///< entry point for checking if message should be sent, also updates theMessageDesirability
  bool checkMessageBudgetAllowsSending();
  bool checkStateAllowsSending();
  bool checkMinMessageIntervalAllowsSending();

  bool checkReadySetPlay();

  bool checkPingNeeded();
  bool checkBallMoved();
  bool checkRoleChanged();
  bool checkReturnFromPenalty(); //Player back from penalty and should be added to teammate list
};

