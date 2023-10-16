/**
 * @file TeamMessageHandler2023b.cpp
 * 
 * The general architecture is based on the original TeamMessageHandler by B-Human,
 * but replacing the particle stuff and generally making the different parts of
 * communication more explicit.
 * 
 * The TeamMessage2023OutputGenerator representation controls the sending of
 * data to teammates while the TeamData representation assists in the receiving of messages
 * from teammates. In both cases, callable functions in the representations
 * do most of work.
 * 
 * The procedure for receiving a message from teammates is:
 * - Cognition::beforeFrame calls TeamMessageCommunicator::receive to receive messages/packets
 *   from the WLAN. This is done before camera images are read. Each message is 
 *   added to the inTeamMessages buffer.
 * - Cognition::beforeModules calls TeamData::updateWithReceivedMessage for each (encoded) message
 *   in the inTeamMessages buffer
 * - updateWithReceivedMessage calls readSPLStandardMessage/readInputBytes to decode the bytes to a TeamMessage2023 struct,
 *   parseIntoBMate to copy relevant values from TeamMessage2023 to Teammate struct
 * 
 * The procedure for sending a message to teammates is:
 * - Cognition::afterModules calls TeamMessage2023OutputGenerator::sendIfNeeded.
 *   This happens at the end of each cycle
 * - sendIfNeeded checks shouldSendThisFrame and if this is true it calls
 *   generateOutputMessage to fill in the data to be sent, 
 *   writeOutputBytes to convert that data to encoded bytes (compressing numbers etc where possible)
 *   and TeamMessageCommunicator::send to actually send the data on the WLAN
 *
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#include "TeamMessageHandler2023b.h"
#include "BitStreamCodec.h"

#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Math/Constants.h"
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Platform/SystemCall.h"

#include "Tools/TextLogging.h"
#include "Tools/Debugging/Annotation.h"

//#define SITTING_TEST
//#define SELF_TEST

#define USE_VARIABLE(var_) (void)var_

DECL_TLOGGER(tlogger, "TeamMessageHandler2023b", TextLogging::WARNING);

MAKE_MODULE(TeamMessageHandler2023b, communication);


// this is a helper struct to collect Bitstream types to make coding/decoding a little bit more readable.
// Each member is a handler for a distinct type or range. If we want to encode
// multiple values with the same type and range we should usually reuse the same helper
// member.
struct TeamMessageCodec
{
  BitStream::Bits<unsigned> header {"header", 5};
  BitStream::Bits<unsigned> version {"version", 3};
  BitStream::Integer<int> sizeOfMessageBody {"sizeOfMessageBody", 0, 255};

  BitStream::Integer<int> playerNum {"playerNum", -1, Settings::highestValidPlayerNumber}; // sometimes we mark invalid players/slots using -1
  BitStream::Integer<int> teamNum {"teamNum", -1, 100};
  BitStream::Integer<uint8_t> seqNumGC {"seqNumGC", 0, 255};
  BitStream::Timestamp timeReceivedGC {"timeReceivedGC", 0, 0xFFFFFFFF, 1, 24}; // this is a truncated version of the real timestamp with ms precision that will wrap every 4.5 hours
  BitStream::Timestamp timestamp {"timestamp", 0, 0xFFFFFFFF, 1, 24}; // this is a truncated version of the real timestamp with ms precision that will wrap every 4.5 hours

  BitStream::Bool isPenalized { "isPenalized" };
  BitStream::Bool isUpright { "isUpgright" };

  BitStream::Floatf robotPoseX { "robotPoseX", -6400, 6400, 8}; // allow for centre spot to outside field lines, 50mm resolution
  BitStream::Floatf robotPoseY { "robotPoseY", -6400, 6400, 8}; // allow for centre spot to outside field lines, 50mm resolution
  BitStream::Floatf robotPosePosSD { "robotPosePosSD", -6400, 6400, 8};
  BitStream::Floatf robotPoseRotation { "robotPoseRotation", -Constants::pi, Constants::pi, 8};
  BitStream::Floatf robotPoseRotationSD { "robotPoseRotationSD", -Constants::pi, Constants::pi, 8};
  BitStream::ContiguousEnum<RobotPose::LocalizationQuality> robotPoseQuality { "robotPoseQuality", RobotPose::numLocalizationQuality};

  BitStream::Floatf ballPosX { "ballPosX", -12800, 12800, 10}; // allow for diagonal distance between field corners and 25 mm resolution
  BitStream::Floatf ballPosY { "ballPosY", -12800, 12800, 10}; // allow for diagonal distance between field corners and 25 mm resolution
  BitStream::Floatf ballPosSD { "ballPosSD", -3200, 3200, 8};
  BitStream::Floatf ballVelocityX { "ballVelocityX", -6400, 6400, 9};
  BitStream::Floatf ballVelocityY { "ballVelocityY", -6400, 6400, 9};
  BitStream::Floatf ballPosLastSeenX { "ballPosLastSeenX", -12800, 12800, 10};
  BitStream::Floatf ballPosLastSeenY { "ballPosLastSeenY", -12800, 12800, 10};

  BitStream::RelativeTimestamp ballFrameTimeLastSeen 
      { "ballFrameTimeLastSeen", BitStream::RelativeTimestamp::beforeRef, 25500, 100 };

  BitStream::RelativeTimestamp frameTime { "frameTime", BitStream::RelativeTimestamp::beforeRef, 255 };

  BitStream::Integer<int> behaviourStatusPassTarget {"behaviourStatusPassTarget", -1, Settings::highestValidPlayerNumber};


  BitStream::RelativeTimestamp teamBehaviourStatusTimeWhenReachBallStriker {
      "teamBehaviourStatusTimeWhenReachBallStriker", BitStream::RelativeTimestamp::afterRef, 25500, 100};
  BitStream::RelativeTimestamp teamBehaviourStatusTimeWhenReachBall {
      "teamBehaviourStatusTimeWhenReachBall", BitStream::RelativeTimestamp::afterRef, 8000, 250};
  BitStream::Integer<int> teamBehaviourStatusCaptain {
      "teamBehaviourStatusCaptain", -1, Settings::highestValidPlayerNumber };
  BitStream::ContiguousEnum<PlayerRole::RoleType> teamBehaviourStatusPlayerRole {
      "teamBehaviourStatusPlayerRole", PlayerRole::numRoleType };
  BitStream::Integer<unsigned> teamBehaviourStatusTeammateRolesLen {
      "teamBehaviourStatusTeammateRolesLen", 0, Settings::highestValidPlayerNumber };
  BitStream::ContiguousEnum<PlayerRole::RoleType> teamBehaviourStatusTeammateRole {
      "teamBehaviourStatusTeammateRole", PlayerRole::numRoleType };
  BitStream::Integer<int> teamBehaviourStatusPassKicker {
      "teamBehaviourStatusPassKicker", -1, Settings::highestValidPlayerNumber };
  BitStream::Integer<int> teamBehaviourStatusPassReceiver {
      "teamBehaviourStatusPassReceiver", -1, Settings::highestValidPlayerNumber };
  BitStream::Integer<unsigned> teamBehaviourStatusNumPasses { "teamBehaviourStatusNumPasses", 0, 255 };


  BitStream::Bool whistleDetectionActive { "whistleDetectionActive" };
  BitStream::Floatf whistleConfidenceLastWhistleDetection { "whistleConfidenceLastWhistleDetection", 0, 2.55f, 8};
  BitStream::RelativeTimestamp whistleFrameTimeLastWhistleDetection {
      "whistleFrameTimeLastWhistleDetection", BitStream::RelativeTimestamp::beforeRef, 25500, 100};


  inline float replaceInfNan(float value, float replaceValue)
  {
    return std::isfinite(value) ? value : replaceValue;
  }
};

static TeamMessageCodec codec; // local to this file

const unsigned HEADER = 0b01110;
const unsigned VERSION = 0b001;



TeamMessageHandler2023b::TeamMessageHandler2023b()
{
#ifndef TARGET_ROBOT
  teamMessageSocketHandler.startLocal(Global::getSettings().getPort(),
                                  static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  teamMessageSocketHandler.start(Global::getSettings().getPort());
#endif
}


// ============================================================================
// ============================================================================
// SENDING TEAM MESSAGES
// ============================================================================
// ============================================================================

void TeamMessageHandler2023b::update(TeamMessage2023OutputGenerator& outputGenerator)
{
  // DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataMin");
  // DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataDesired");
  // DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataActual");

  if (!messageDesirabilityUpdated)
    updateMessageDesirability();

  outputGenerator.teamMessage.isUpright =
      (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering ||
      theFallDownState.state == FallDownState::squatting) &&
      (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp &&
      theMotionRequest.motion != MotionRequest::getUp);

  // send will be called in Cognition after all modules have executed
  outputGenerator.sendIfNeeded = [this, &outputGenerator]()
  {
    // TLOGD(tlogger, "sendIfNeeded");
    if (messageDesirability.shouldSend)
    { 
      TLOGV(tlogger, "outputGenerator shouldSend so about to send");    
      generateOutputMessage(outputGenerator);
      writeOutputMessageBytes(outputGenerator, outputTeamMessageBytes);
      sendMessageBytes(outputGenerator, outputTeamMessageBytes);
    }
  };
}


void TeamMessageHandler2023b::generateOutputMessage(TeamMessage2023OutputGenerator& outputGenerator) const
{
  TeamMessage2023& teamMessage = outputGenerator.teamMessage;

  teamMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);

  // timeSyncInfo
  teamMessage.seqNumGC = theRawGameInfo.packetNumber;
  teamMessage.timeReceivedGC = theRawGameInfo.timeLastPacketReceived;

  // outputGenerator.teamMessage.seqNum = outputGenerator.sentMessages % 15; // TODO not used

  teamMessage.timestamp = Time::getCurrentSystemTime();

  teamMessage.isPenalized = (theRobotInfo.penalty != PENALTY_NONE);
  // teamMessage.isUpright was already filled out in update method - not sure why it's done this way

  // robotPose

  if (sendMirroredRobotPose)
  {
    Pose2f theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    teamMessage.robotPose.pose = theMirroredRobotPose;
  }
  else
    teamMessage.robotPose.pose = theRobotPose;

  teamMessage.robotPose.translationSD = Covariance::getMaxTranslationSD(theRobotPose.covariance);
  teamMessage.robotPose.rotationSD = Covariance::getRotationSD(theRobotPose.covariance);
  teamMessage.robotPose.quality = theRobotPose.quality;

  // ballModel
  teamMessage.ballModel.position = theBallModel.estimate.position;
  teamMessage.ballModel.velocity = theBallModel.estimate.velocity;
  teamMessage.ballModel.positionSD = Covariance::getMaxSD(theBallModel.estimate.covariance);
  teamMessage.ballModel.positionLastSeen = theBallModel.lastPerception;
  teamMessage.ballModel.frameTimeLastSeen = theBallModel.timeWhenLastSeen;

  // frameInfo
  teamMessage.frameTime = theFrameInfo.time;

  // obstacleModel
  // TODO

  // behaviourStatus
  teamMessage.behaviourStatus.passTarget = static_cast<TeamMessage2023::PlayerNum>(theBehaviorStatus.passTarget);

  // teamBehaviourStatus
  teamMessage.teamBehaviourStatus.timeWhenReachBallStriker = theTeamBehaviorStatus.timeToReachBall.timeWhenReachBallStriker;
  teamMessage.teamBehaviourStatus.timeWhenReachBall = theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall;

  teamMessage.teamBehaviourStatus.captain = static_cast<TeamMessage2023::PlayerNum>(theTeamBehaviorStatus.teammateRoles.captain);

  // copy from theTeamBehaviorStatus.teammateRoles.roles to teamMessage.teamBehaviourStatus.teammateRoles with casting
  teamMessage.teamBehaviourStatus.teammateRoles.clear();
  for (int role : theTeamBehaviorStatus.teammateRoles.roles)
    teamMessage.teamBehaviourStatus.teammateRoles.emplace_back(static_cast<PlayerRole::RoleType>(std::max(role, 0)));

  // std::transform(theTeamBehaviorStatus.teammateRoles.roles.begin(), 
  //                theTeamBehaviorStatus.teammateRoles.roles.end(),
  //                std::back_inserter(teamMessage.teamBehaviourStatus.teammateRoles),
  //                [](int value) { return static_cast<PlayerRole::RoleType>(std::max(value, 0)); });
  teamMessage.teamBehaviourStatus.playerRole = theTeamBehaviorStatus.role.role;

  teamMessage.teamBehaviourStatus.passKicker = static_cast<TeamMessage2023::PlayerNum>(theTeamBehaviorStatus.passStatus.passKicker);
  teamMessage.teamBehaviourStatus.passReceiver = static_cast<TeamMessage2023::PlayerNum>(theTeamBehaviorStatus.passStatus.passReceiver);
  teamMessage.teamBehaviourStatus.numPasses = static_cast<unsigned>(theTeamBehaviorStatus.passStatus.numPasses);

  // Whistle
  teamMessage.whistle.detectionActive = (theWhistle.channelsUsedForWhistleDetection > 0);
  teamMessage.whistle.frameTimeLastWhistleDetection = theWhistle.lastTimeWhistleDetected;
  
  if (std::isfinite(theWhistle.confidenceOfLastWhistleDetection) && (theWhistle.channelsUsedForWhistleDetection > 0))
  {
    teamMessage.whistle.confidenceLastWhistleDetection =
        theWhistle.confidenceOfLastWhistleDetection / theWhistle.channelsUsedForWhistleDetection;
  }
  else
    teamMessage.whistle.confidenceLastWhistleDetection = 0.f;
}


void TeamMessageHandler2023b::writeOutputMessageBytes(TeamMessage2023OutputGenerator &outputGenerator,
                                               TeamMessageBytes& teamMessageBytes) const
{
  TeamMessage2023& teamMessage = outputGenerator.teamMessage;

  teamMessageBytes.size = teamMessageBytes.capacity; // initial size = max

  BitStream::Writer headerWriter(teamMessageBytes.data, 2);
  BitStream::Writer writer(teamMessageBytes.data + 2, (unsigned) teamMessageBytes.size-2);


  codec.header.write(headerWriter, HEADER);
  codec.version.write(headerWriter, VERSION);

  // The next byte to be written to the header is the byte count for all 
  // remaining data. We don't write this immediately. Instead, once all the 
  // values are written we come back and update 
  // the byte count in the header
  // TODO: I don't think this byte count is really needed - delete it

  codec.teamNum.write(writer, Global::getSettings().teamNumber);
  codec.playerNum.write(writer, teamMessage.playerNum);

  // timeSyncInfo
  codec.seqNumGC.write(writer, teamMessage.seqNumGC);
  codec.timeReceivedGC.write(writer, teamMessage.timeReceivedGC);

  // writer.writeInteger(outputGenerator.sentMessages % 15, 0, 14, 1, 4); // sequence number if needed

  codec.timestamp.write(writer, teamMessage.timestamp);

  TLOGV(tlogger, "writeOutputMessageBytes: team {}, player {}, seqNumGC {}, timeReceivedGC {}, timestamp {}",
        Global::getSettings().teamNumber, teamMessage.playerNum, teamMessage.seqNumGC, teamMessage.timeReceivedGC,
        teamMessage.timestamp);

  codec.isPenalized.write(writer, teamMessage.isPenalized);
  codec.isUpright.write(writer, teamMessage.isUpright);
  // TODO: check if frameTimeWhenLastUpright is used/need

  // robotPose
  codec.robotPoseX.write(writer, teamMessage.robotPose.pose.translation.x());
  codec.robotPoseY.write(writer, teamMessage.robotPose.pose.translation.y());
  codec.robotPoseRotation.write(writer, teamMessage.robotPose.pose.rotation);
  codec.robotPosePosSD.write(writer, teamMessage.robotPose.translationSD);
  codec.robotPoseRotationSD.write(writer, teamMessage.robotPose.rotationSD);
  codec.robotPoseQuality.write(writer, teamMessage.robotPose.quality);

  // ballModel
  codec.ballPosX.write(writer, teamMessage.ballModel.position.x());
  codec.ballPosY.write(writer, teamMessage.ballModel.position.y());
  codec.ballVelocityX.write(writer, teamMessage.ballModel.velocity.x());
  codec.ballVelocityY.write(writer, teamMessage.ballModel.velocity.y());
  codec.ballPosSD.write(writer, teamMessage.ballModel.positionSD);
  codec.ballPosLastSeenX.write(writer, teamMessage.ballModel.positionLastSeen.x());
  codec.ballPosLastSeenY.write(writer, teamMessage.ballModel.positionLastSeen.y());

  codec.ballFrameTimeLastSeen.write(writer, teamMessage.ballModel.frameTimeLastSeen, teamMessage.timestamp);

  // frameInfo
  codec.frameTime.write(writer, teamMessage.frameTime, teamMessage.timestamp);

  // obstacleModel
  // TODO

  // behaviourStatus
  codec.behaviourStatusPassTarget.write(writer, teamMessage.behaviourStatus.passTarget);

  // teamBehaviourStatus
  codec.teamBehaviourStatusTimeWhenReachBallStriker.write(
      writer, teamMessage.teamBehaviourStatus.timeWhenReachBallStriker, teamMessage.timestamp);
  codec.teamBehaviourStatusTimeWhenReachBall.write(
      writer, teamMessage.teamBehaviourStatus.timeWhenReachBall, teamMessage.teamBehaviourStatus.timeWhenReachBallStriker);

  codec.teamBehaviourStatusCaptain.write(writer, teamMessage.teamBehaviourStatus.captain);

  
  if (teamMessage.teamBehaviourStatus.teammateRoles.empty())
    codec.teamBehaviourStatusTeammateRolesLen.write(writer, 0);
  else
  {
    codec.teamBehaviourStatusTeammateRolesLen.write(
        writer, static_cast<unsigned>(teamMessage.teamBehaviourStatus.teammateRoles.size()));

    for (const auto& role : teamMessage.teamBehaviourStatus.teammateRoles)
      codec.teamBehaviourStatusTeammateRole.write(writer, role);
  }
  // codec.mediumPast.write(writer, teamMessage.teamBehaviourStatus.teammateRoles.timestamp, teamMessage.timestamp); // Not Impl

  // playerRole
  codec.teamBehaviourStatusPlayerRole.write(writer, teamMessage.teamBehaviourStatus.playerRole);
  // codec.numPlayers.write(writer, teamMessage.teamBehaviourStatus.playerRole.numOfActiveSupporters); // Not impl

  // pass status
  codec.teamBehaviourStatusPassKicker.write(writer, teamMessage.teamBehaviourStatus.passKicker);
  codec.teamBehaviourStatusPassReceiver.write(writer, teamMessage.teamBehaviourStatus.passReceiver);
  codec.teamBehaviourStatusNumPasses.write(writer, teamMessage.teamBehaviourStatus.numPasses);

  // Whistle
  codec.whistleDetectionActive.write(writer, teamMessage.whistle.detectionActive);
  codec.whistleConfidenceLastWhistleDetection.write(writer, teamMessage.whistle.confidenceLastWhistleDetection, 0.f);
  codec.whistleFrameTimeLastWhistleDetection.write(writer, teamMessage.whistle.frameTimeLastWhistleDetection,
                                                   teamMessage.timestamp);

  // finalize the buffers and sizes - once we know the (encoded) message body size
  // we need to add in that size to the header at which point our entire message
  // has been encoded into teamMessageBytes
  writer.finalizeBuffer();
  int messageBodySize = writer.size();
  codec.sizeOfMessageBody.write(headerWriter, messageBodySize);
  headerWriter.finalizeBuffer();

  // the underlying data array, teamMessageBytes.data, is now up to date
  // so ensure the final size is correct
  teamMessageBytes.size = messageBodySize + 2; // add 2 for the header
}


void TeamMessageHandler2023b::sendMessageBytes(TeamMessage2023OutputGenerator &outputGenerator,
                                               const TeamMessageBytes &teamMessageBytes)
{
  if (!teamMessageSocketHandler.send(teamMessageBytes)) // it is possible that the port has not been set
    return;

  outputGenerator.sentMessages++;

  sentTeamMessage.teamMessage = outputGenerator.teamMessage;
  sentTeamMessage.frameTime = theFrameInfo.time;

  // TODO
  // PLOT("module:TeamMessageHandler2023:SPLStdMsgDataCore", offset);
  // PLOT("module:TeamMessageHandler2023:SPLStdMsgDataDesired", desiredTotalSize);
  // PLOT("module:TeamMessageHandler2023:SPLStdMsgDataActual", outputGenerator.theBSPLStandardMessage.numOfDataBytes);

  // TLOGV(tlogger, "SPL data core={}, desired={}, actual={}", offset, desiredTotalSize,
  //       outputGenerator.theBSPLStandardMessage.numOfDataBytes);

  TLOGI(tlogger, "sent from {}, sentMessages {}, timestamp {}, frameTimeSent {}", theRobotInfo.number,
        outputGenerator.sentMessages, outputGenerator.teamMessage.timestamp, theFrameInfo.time);
}

// ============================================================================
// ============================================================================
// RECEIVING TEAM MESSAGES
// ============================================================================
// ============================================================================


void TeamMessageHandler2023b::update(TeamData& teamData)
{
  if (teamData.receivedMessages == 0)
  {
    teamData.teammateMessageTimestamps.fill(0);
    teamData.teammateMessageCounts.fill(0);
  }

  messageDesirabilityUpdated = false;

  // update timeSync with the latest GC times we have before processing received team messages
  timeSyncInfo.updateGCInfo(theRawGameInfo.packetNumber, theRawGameInfo.timeLastPacketReceived, theFrameInfo.time);

  teamMessageSocketHandler.receive(inputTeamMessageBytesBuffer); // pushes received message onto back of buffer, so oldest message is the front of the buffer
  if (!inputTeamMessageBytesBuffer.empty())
    TLOGV(tlogger, "update(TeamData), after socket receive, inputTeamMessageBytesBuffer size {}",
        inputTeamMessageBytesBuffer.size());

  // decode and process each of the received messages
  // bool wasEmpty = inputTeamMessageBytesBuffer.empty();

  size_t numMessages = inputTeamMessageBytesBuffer.size();
  for (unsigned iMessage = 1; !inputTeamMessageBytesBuffer.empty(); iMessage++)
  {
    const ReceivedTeamMessageBytes& inputMessageBytes = inputTeamMessageBytesBuffer.front(); // first received message first

    TLOGV(tlogger, "readInputMessageBytes message {} of {} ({})", iMessage, numMessages, (void*)&inputMessageBytes);
    TLOGV(tlogger, "inputTeamMessageBytesBuffer: {}", inputTeamMessageBytesBuffer.debugString());


    if (readInputMessageBytes(inputMessageBytes, receivedTeamMessage))
    {
      // at this point we have successfully decoded the input messsage bytes into
      // the receivedTeamMessage structure

      TLOGV(tlogger, "readInputMessageBytes was successful");

      // we've received a valid message
      teamData.receivedMessages++;
      teamData.teammateMessageTimestamps[receivedTeamMessage.playerNum] = receivedTeamMessage.timestamp;
      teamData.teammateMessageCounts[receivedTeamMessage.playerNum]++;

      parseMessageIntoTeammate(receivedTeamMessage, getTeammate(teamData, receivedTeamMessage.playerNum));

      TLOGI(tlogger, "received from {}, timestamp {}, received teamate/total {}/{}", receivedTeamMessage.playerNum,
            receivedTeamMessage.timestamp, teamData.teammateMessageCounts[receivedTeamMessage.playerNum], teamData.receivedMessages);
    }
    else
    {
      if (receivedTeamMessage.lastErrorCode != ReceivedTeamMessage::myOwnMessage &&
          receivedTeamMessage.lastErrorCode != ReceivedTeamMessage::magicNumberDidNotMatch &&
          receivedTeamMessage.lastErrorCode != ReceivedTeamMessage::timeSyncFailure)
      {
        // the message had a parsing error => possible intruder on our comms port
        if (theFrameInfo.getTimeSince(timeLastAlert) > minTimeBetween2RejectSounds &&
            SystemCall::playSound("intruderAlert.wav"))
          timeLastAlert = theFrameInfo.time;

        ANNOTATION("intruder-alert", "error code: " << receivedTeamMessage.lastErrorCode);
      }
    }

    inputTeamMessageBytesBuffer.pop_front(); // we've finished processing this message
  }

  maintainTeammateList(teamData);
}


void TeamMessageHandler2023b::maintainTeammateList(TeamData& teamData) const
{
  //@author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
  {
    // Iterate over deprecated list of teammate information and update some convenience information
    // (new information has already been coming via handleMessages)
    for  (auto& teammate : teamData.teammates)
    {
      Teammate::Status newStatus = Teammate::PLAYING;
      if  (teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if  (!teammate.isUpright) // || !teammate.hasGroundContact)
        newStatus = Teammate::FALLEN;

      if  (newStatus != teammate.status)
      {
        teammate.status = newStatus;
        teammate.timeWhenStatusChanged = theFrameInfo.time;
      }

      teammate.isGoalkeeper = (teammate.number == 1);
    }

    //Remove Penalised robots
    teamData.numberOfActiveTeammates = 0;

    for (auto teammate = teamData.teammates.begin(); teammate != teamData.teammates.end(); )
    {
      if (teammate->status == Teammate::PENALIZED)
      {
        TLOGD(tlogger, "maintainTeammateList: remove teammate (player {}) because penalized", teammate->number);
        teammate = teamData.teammates.erase(teammate);
      }
      else if (theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
      {
        TLOGD(tlogger, "maintainTeammateList: remove teammate (player {}) because no packets received in {} ms",
              teammate->number, networkTimeout);
        teammate = teamData.teammates.erase(teammate);
      }
      else
      {
        teamData.numberOfActiveTeammates++;
        ++teammate;
      }
    }
  }
}

#define PARSING_ERROR(...)                                                                                             \
  {                                                                                                                    \
    std::string s = fmt::format("readInputMessageBytes parsing error: " __VA_ARGS__);                                  \
    OUTPUT_ERROR(s);                                                                                                   \
    receivedTeamMessage.lastErrorCode = ReceivedTeamMessage::parsingError;                                             \
    return false;                                                                                                      \
  }

bool TeamMessageHandler2023b::readInputMessageBytes(const ReceivedTeamMessageBytes &receivedMsgBytes,
                                                    ReceivedTeamMessage &inputTeamMessage)
{
  BitStream::Reader reader(receivedMsgBytes.data, receivedMsgBytes.size);

  unsigned header;
  codec.header.read(reader, header);
  if (header != HEADER)
    PARSING_ERROR("bad message header: {:#b}", header);

  unsigned version;
  codec.version.read(reader, version);
  if (version != VERSION)
    PARSING_ERROR("bad message version: {:#b}", version);

  int len;
  codec.sizeOfMessageBody.read(reader, len); // TODO is this really needed?

  // TLOGV(tlogger, "readInputMessageBytes header,version OK, sizeOfMessageBody = {}", len);

  // read team and player numbers

  int teamNumber;
  codec.teamNum.read(reader, teamNumber);
  if (teamNumber != Global::getSettings().teamNumber)
    PARSING_ERROR("Invalid team number received: {}, expected {}", teamNumber, Global::getSettings().teamNumber);

  codec.playerNum.read(reader, inputTeamMessage.playerNum);
  if (inputTeamMessage.playerNum < Settings::lowestValidPlayerNumber ||
      inputTeamMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received: {}", inputTeamMessage.playerNum);

  // TLOGV(tlogger, "readInputMessageBytes from team {}, robot {}", teamNumber, inputTeamMessage.playerNum);


  // timeSyncInfo
  codec.seqNumGC.read(reader, inputTeamMessage.seqNumGC);
  codec.timeReceivedGC.read(reader, inputTeamMessage.timeReceivedGC);

  codec.timestamp.read(reader, inputTeamMessage.timestamp);

#ifndef SELF_TEST
  if(inputTeamMessage.playerNum == theRobotInfo.number)
  {
    TLOGV(tlogger,
          "readInputMessageBytes: ***myOwnMessage, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
          inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
    inputTeamMessage.lastErrorCode = ReceivedTeamMessage::myOwnMessage;
    return false;
  }
  else
    TLOGV(tlogger,
          "readInputMessageBytes: player {}, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
          inputTeamMessage.playerNum, inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
#else
  TLOGV(tlogger,
        "readInputMessageBytes: player {}, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
        inputTeamMessage.playerNum, inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
#endif // !SELF_TEST


  // update timesync info based on the teammate's packet
  timeSyncInfo.updateTeammateInfo(inputTeamMessage.playerNum, inputTeamMessage.timestamp, inputTeamMessage.seqNumGC,
                                  inputTeamMessage.timeReceivedGC);

  if (!timeSyncInfo.getTeammateInfo(inputTeamMessage.playerNum).isValid())
  {
    OUTPUT_WARNING(
        fmt::format("TeamMessageHandler2023: rejecting Teammate Message from {} - unable to achieve timesync",
                    inputTeamMessage.playerNum));

    receivedTeamMessage.lastErrorCode = ReceivedTeamMessage::timeSyncFailure;
    return false;
  }

  // OK - we've achieved timesync so update our reference time accordingly

  inputTeamMessage.timestamp = timeSyncInfo.estimateLocalTime(inputTeamMessage.playerNum, inputTeamMessage.timestamp);

  // now all subsequent decodings use the estimated local time reference

  codec.isPenalized.read(reader, inputTeamMessage.isPenalized);
  codec.isUpright.read(reader, inputTeamMessage.isUpright);
  // TODO: check if frameTimeWhenLastUpright is used/need

  // robotPose
  codec.robotPoseX.read(reader, inputTeamMessage.robotPose.pose.translation.x());
  codec.robotPoseY.read(reader, inputTeamMessage.robotPose.pose.translation.y());
  codec.robotPoseRotation.read(reader, inputTeamMessage.robotPose.pose.rotation);
  codec.robotPosePosSD.read(reader, inputTeamMessage.robotPose.translationSD);
  codec.robotPoseRotationSD.read(reader, inputTeamMessage.robotPose.rotationSD);
  codec.robotPoseQuality.read(reader, inputTeamMessage.robotPose.quality);

  // ballModel
  codec.ballPosX.read(reader, inputTeamMessage.ballModel.position.x());
  codec.ballPosY.read(reader, inputTeamMessage.ballModel.position.y());
  codec.ballVelocityX.read(reader, inputTeamMessage.ballModel.velocity.x());
  codec.ballVelocityY.read(reader, inputTeamMessage.ballModel.velocity.y());
  codec.ballPosSD.read(reader, inputTeamMessage.ballModel.positionSD);
  codec.ballPosLastSeenX.read(reader, inputTeamMessage.ballModel.positionLastSeen.x());
  codec.ballPosLastSeenY.read(reader, inputTeamMessage.ballModel.positionLastSeen.y());

  codec.ballFrameTimeLastSeen.read(reader, inputTeamMessage.ballModel.frameTimeLastSeen, inputTeamMessage.timestamp);

  // frameInfo
  codec.frameTime.read(reader, inputTeamMessage.frameTime, inputTeamMessage.timestamp);

  // obstacleModel
  // TODO

  // behaviourStatus
  codec.behaviourStatusPassTarget.read(reader, inputTeamMessage.behaviourStatus.passTarget);

  // teamBehaviourStatus
  codec.teamBehaviourStatusTimeWhenReachBallStriker.read(
      reader, inputTeamMessage.teamBehaviourStatus.timeWhenReachBallStriker, inputTeamMessage.timestamp);
  codec.teamBehaviourStatusTimeWhenReachBall.read(reader, inputTeamMessage.teamBehaviourStatus.timeWhenReachBall,
                                                  inputTeamMessage.teamBehaviourStatus.timeWhenReachBallStriker);

  codec.teamBehaviourStatusCaptain.read(reader, inputTeamMessage.teamBehaviourStatus.captain);

  unsigned numTeammateRoles;
  codec.teamBehaviourStatusTeammateRolesLen.read(reader, numTeammateRoles);
  for (unsigned i=0; i < numTeammateRoles; i++)
  {
    if (inputTeamMessage.teamBehaviourStatus.teammateRoles.size() < numTeammateRoles)
      inputTeamMessage.teamBehaviourStatus.teammateRoles.emplace_back(PlayerRole::none);

    codec.teamBehaviourStatusTeammateRole.read(reader, inputTeamMessage.teamBehaviourStatus.teammateRoles[i]);
  }

  codec.teamBehaviourStatusCaptain.read(reader, inputTeamMessage.teamBehaviourStatus.captain);
  // codec.mediumPast.read(reader, inputTeamMessage.teamBehaviourStatus.teammateRoles.timestamp, inputTeamMessage.timestamp); // not impl

  // playerRole
  codec.teamBehaviourStatusPlayerRole.read(reader, inputTeamMessage.teamBehaviourStatus.playerRole);
  // codec.numPlayers.read(reader, inputTeamMessage.teamBehaviourStatus.playerRole.numOfActiveSupporters); TODO - do we need this?

  // pass status
  codec.teamBehaviourStatusPassKicker.read(reader, inputTeamMessage.teamBehaviourStatus.passKicker);
  codec.teamBehaviourStatusPassReceiver.read(reader, inputTeamMessage.teamBehaviourStatus.passReceiver);
  codec.teamBehaviourStatusNumPasses.read(reader, inputTeamMessage.teamBehaviourStatus.numPasses);

  // Whistle
  codec.whistleDetectionActive.read(reader, inputTeamMessage.whistle.detectionActive);
  codec.whistleConfidenceLastWhistleDetection.read(reader, inputTeamMessage.whistle.confidenceLastWhistleDetection);
  codec.whistleFrameTimeLastWhistleDetection.read(reader, inputTeamMessage.whistle.frameTimeLastWhistleDetection, inputTeamMessage.timestamp);

  inputTeamMessage.timeReceived = receivedMsgBytes.timeReceived;

  // ANNOTATION("TeamMessageHandler2023", "readSPLStandardMessage read OK");
  return true;
}

Teammate& TeamMessageHandler2023b::getTeammate(TeamData& teamData, int playerNum) const
{
  // find the teammate by playerNum in our existing list (from prev cycle)
  for (auto &teammate : teamData.teammates)
    if (teammate.number == playerNum)
      return teammate;

  // not found => this teammate has just come online or come back from penalty
  teamData.teammates.emplace_back();

  TLOGD(tlogger, "getTeammate: teammate added (player {})", playerNum);

  return teamData.teammates.back();
}


void TeamMessageHandler2023b::parseMessageIntoTeammate(const ReceivedTeamMessage& inputTeamMessage, Teammate& teammate)
{
  teammate.number = inputTeamMessage.playerNum;

  // time info
  teammate.timeWhenLastPacketSent = inputTeamMessage.timestamp; // already converted to local time
  teammate.timeWhenLastPacketReceived = inputTeamMessage.timeReceived;


  teammate.isPenalized = inputTeamMessage.isPenalized;
  teammate.isUpright = inputTeamMessage.isUpright;
  // teammate.hasGroundContact = robotStatus.hasGroundContact; // TODO
  // teammate.timeWhenLastUpright = robotStatus.timeWhenLastUpright;
  // teammate.timeOfLastGroundContact = robotStatus.timeOfLastGroundContact;

  // teammate.sequenceNumber = robotStatus.sequenceNumbers[teammate.number - Settings::lowestValidPlayerNumber];
  // teammate.returnSequenceNumber = robotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber];

  // RobotPose
  teammate.theRobotPose = inputTeamMessage.robotPose.pose;
  teammate.theRobotPose.quality = inputTeamMessage.robotPose.quality;
  teammate.theRobotPose.covariance(0,0) = sqr(inputTeamMessage.robotPose.translationSD);
  teammate.theRobotPose.covariance(1,1) = sqr(inputTeamMessage.robotPose.translationSD);
  teammate.theRobotPose.covariance(2,2) = sqr(inputTeamMessage.robotPose.rotationSD);

  // FrameInfo
  teammate.theFrameInfo.time = inputTeamMessage.frameTime;

  // BallModel
  teammate.theBallModel.estimate = BallState();
  teammate.theBallModel.estimate.position = inputTeamMessage.ballModel.position;
  teammate.theBallModel.estimate.velocity = inputTeamMessage.ballModel.velocity;
  teammate.theBallModel.estimate.covariance(0,0) = sqr(inputTeamMessage.ballModel.positionSD);
  teammate.theBallModel.estimate.covariance(1,1) = sqr(inputTeamMessage.ballModel.positionSD);

  teammate.theBallModel.lastPerception = inputTeamMessage.ballModel.positionLastSeen;
  teammate.theBallModel.timeWhenLastSeen = inputTeamMessage.ballModel.frameTimeLastSeen;
  // ballModel.timeWhenDisappeared not used anywhere
  // ballModel.seenPercentage not used anywhere

  // ObstacleModel
  // TODO: not included for now

  // BehaviorStatus
  teammate.theBehaviorStatus = BehaviorStatus(); // set defaults except where we override them
  teammate.theBehaviorStatus.passTarget = inputTeamMessage.behaviourStatus.passTarget;

  // TeamBehaviorStatus
  teammate.theTeamBehaviorStatus.teamActivity = TeamBehaviorStatus::noTeam; // we don't use this value anywhere
  teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBallStriker =
      inputTeamMessage.teamBehaviourStatus.timeWhenReachBallStriker;
  teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall = //std::numeric_limits<unsigned>::max(); // TODO - shall we calc this or communicate it
      inputTeamMessage.teamBehaviourStatus.timeWhenReachBall;

  teammate.theTeamBehaviorStatus.teammateRoles.captain = inputTeamMessage.teamBehaviourStatus.captain;

  // copy from receivedTeamMessage.teamBehaviourStatus.teammateRoles to theTeamBehaviorStatus.teammateRoles.roles with casting
  for (size_t i=0; i < inputTeamMessage.teamBehaviourStatus.teammateRoles.size(); i++)
    teammate.theTeamBehaviorStatus.teammateRoles[i] = inputTeamMessage.teamBehaviourStatus.teammateRoles[i];

  // std::transform(inputTeamMessage.teamBehaviourStatus.teammateRoles.begin(),
  //                inputTeamMessage.teamBehaviourStatus.teammateRoles.end(),
  //                std::back_inserter(teammate.theTeamBehaviorStatus.teammateRoles.roles),
  //                [](PlayerRole::RoleType role) { return static_cast<int>(role); });

  teammate.theTeamBehaviorStatus.role.role = inputTeamMessage.teamBehaviourStatus.playerRole;
  teammate.theTeamBehaviorStatus.role.numOfActiveSupporters = 0; // TODO maybe replace with this robot's count of activeSupporters - I don't think it is used for teammates

  teammate.theTeamBehaviorStatus.passStatus.passKicker = inputTeamMessage.teamBehaviourStatus.passKicker;
  teammate.theTeamBehaviorStatus.passStatus.passReceiver = inputTeamMessage.teamBehaviourStatus.passReceiver;
  teammate.theTeamBehaviorStatus.passStatus.numPasses = inputTeamMessage.teamBehaviourStatus.numPasses;

  // Whistle
  teammate.theWhistle.confidenceOfLastWhistleDetection = inputTeamMessage.whistle.confidenceLastWhistleDetection;
  teammate.theWhistle.channelsUsedForWhistleDetection = inputTeamMessage.whistle.detectionActive ? 1 : 0; // simplify by assuming all robots operate on just 1 channel
  teammate.theWhistle.lastTimeWhistleDetected = inputTeamMessage.whistle.frameTimeLastWhistleDetection;

  // ANNOTATION("TeamMessageHandler2023", "ParseMessageIntoTeammate seems OK");
}

// ============================================================================
// ============================================================================
// Message Desirability functionality
// ============================================================================
// ============================================================================

void TeamMessageHandler2023b::updateMessageDesirability()
{
  messageDesirabilityUpdated = true;

  messageDesirability.shouldSend = false;
  messageDesirability.reasonToSend = MessageDesirability::noReasonToSend;
  messageDesirability.reasonNotToSend = MessageDesirability::noReasonNotToSend;


  // check for reasons not to send (regardless of any other factors) first

  if (!checkMessageBudgetAllowsSending() || !checkStateAllowsSending() || !checkMinMessageIntervalAllowsSending())
  {
    // TLOGV(tlogger, "updateMessageDesirability - should not send because {}",
    //       TypeRegistry::getEnumName(messageDesirability.reasonNotToSend));
    return;
  }

  // now deal with suppressing messages in the ready and set state, but noting
  // that certain messages (e.g. ping, returnFromPenalty) must be sent even in READY or SET

  checkReturnFromPenalty();
  checkPingNeeded();
  checkReadySetPlay();

  if (messageDesirability.hasReasonNotToSend() && !messageDesirability.hasReasonToSend())
  {
    // TLOGV(tlogger, "updateMessageDesirability - should not send because {}",
    //       TypeRegistry::getEnumName(messageDesirability.reasonNotToSend));
    return;
  }

  // now check for other reasons to send - stop at the first one that is a reason
  // to send (if any)

  if (messageDesirability.hasReasonToSend() || checkBallMoved() || checkRoleChanged() || checkPlayerMoved())
  {
    messageDesirability.shouldSend = true;
    TLOGD(tlogger, "updateMessageDesirability shouldSend because {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend));
    ANNOTATION("MessageDesirability",
               fmt::format("shouldSend reason = {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend)));
  }

}


bool TeamMessageHandler2023b::checkStateAllowsSending()
{
#if defined(TARGET_ROBOT) && !defined(SITTING_TEST)
  bool playingDead = (theMotionRequest.motion == MotionRequest::playDead) || 
                     (theMotionInfo.executedPhase == MotionPhase::playDead);
#else
  bool playingDead = false;
#endif

  if (!wasPenalized && theRobotInfo.isPenalized())
    wasPenalized = true;

  bool gameSituationAllows = (theRawGameInfo.state != STATE_INITIAL) && (theRawGameInfo.state != STATE_FINISHED) &&
                             (!theRobotInfo.isPenalized()) && (theRawGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT);

  if (playingDead || !gameSituationAllows)
    messageDesirability.reasonNotToSend = MessageDesirability::statePreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}


bool TeamMessageHandler2023b::checkMessageBudgetAllowsSending()
{
  messageDesirability.messageBudgetThisHalf = theOwnTeamInfo.messageBudget;
  if (theRawGameInfo.firstHalf)
    messageDesirability.messageBudgetThisHalf -= totalMessageBudget / 2;

  if (messageDesirability.messageBudgetThisHalf <= 5)
    messageDesirability.reasonNotToSend = MessageDesirability::messageBudgetPreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}


bool TeamMessageHandler2023b::checkMinMessageIntervalAllowsSending()
{
  // FIXME: log playback at least seems to have frameTimeWrapAround every 20-40secs - why?
  bool frameTimeWrapAround = (theFrameInfo.time < sentTeamMessage.frameTime) && 
                             ((sentTeamMessage.frameTime - theFrameInfo.time) > 1000);

  if (!frameTimeWrapAround && (theFrameInfo.getTimeSince(sentTeamMessage.frameTime) < minSendInterval))
    messageDesirability.reasonNotToSend = MessageDesirability::minIntervalPreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}


bool TeamMessageHandler2023b::checkPingNeeded()
{
  // FIXME: log playback at least seems to have frameTimeWrapAround every 20-40secs - why?
  bool frameTimeWrapAround = (theFrameInfo.time < sentTeamMessage.frameTime) && 
                             ((sentTeamMessage.frameTime - theFrameInfo.time) > 1000);

  if (frameTimeWrapAround)
  {
    TLOGW(tlogger, "frameTimeWrapAround frameTimeNow {} < sentTeamMessage.frameTime {} by {} ms", 
          theFrameInfo.time, sentTeamMessage.frameTime, (sentTeamMessage.frameTime - theFrameInfo.time));

    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;
  }
  else if (theFrameInfo.getTimeSince(sentTeamMessage.frameTime) >= maxSendInterval)
    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;

  return messageDesirability.hasReasonToSend();
}

bool TeamMessageHandler2023b::checkReadySetPlay()
{
  // initial to ready (i.e. walk in from side at the start of a game)
  if (theGameInfo.state == STATE_READY) 
  {
    if ((theExtendedGameInfo.gameStateBeforeCurrent == STATE_INITIAL) && (theExtendedGameInfo.timeSinceReadyStarted == 0))
      messageDesirability.reasonToSend = MessageDesirability::pingNeeded;
    else
      messageDesirability.reasonNotToSend = MessageDesirability::readySetConstrainsSending;

    // in the case of transition from PLAYING to READY we don't do anything and rely on normal timeout based ping behaviour
  }
  else if (theGameInfo.state == STATE_SET)
  {
    messageDesirability.reasonNotToSend = MessageDesirability::readySetConstrainsSending;

    // The normal transition is from READY to SET - we don't need to do anything special for this.
    // The other possibility is from PLAYING to SET - this will occur if we mistakenly
    // heard a whistle and then found out that we should not be in PLAYING state.
    // Again, however, it doesn't seem that we would need to send any additional messages
  }
  else if (theGameInfo.state == STATE_PLAYING)
  {
    // we get here either via GC signalling the SET state or by guessing the
    // state if we think we heard a whistle.
    // Send a message if we have just transitioned to PLAYING and not sent one in the last few seconds (e.g. due
    // to a ping or hearing the whistle)
    if ((theExtendedGameInfo.gameStateLastFrame != STATE_PLAYING) &&
        (theFrameInfo.getTimeSince(sentTeamMessage.frameTime) > maxTimeWithoutMessageBeforePlaying))
      messageDesirability.reasonToSend = MessageDesirability::pingNeeded;
  }

  return messageDesirability.hasReasonToSend() && !messageDesirability.hasReasonNotToSend();
}



bool TeamMessageHandler2023b::checkBallMoved()
{
  // Is Our Local Ball Model Bad?
  if (theBallModel.seenPercentage <= 20 && (theBallModel.timeWhenDisappeared - theBallModel.timeWhenLastSeen > 2000))
  {
    return false;
  }
  float minDistanceBallSent = 0;
  Vector2f lastBallGiven = Vector2f::Zero();
  // Compare with what Teamdata says our ball model says
  for (auto teammate : theTeamData.teammates)
  {
    if (teammate.number == theRobotInfo.number)
    {
      lastBallGiven = teammate.theBallModel.estimate.position;
      minDistanceBallSent = (theBallModel.estimate.position - lastBallGiven).norm();
      break;
    }
  }

  // Do we agree with old Teamball if so send nothing keep as is
  if (minDistanceBallSent < minDistanceLastTeamBall)
  {
    return false;
  }
  else
  { // Does any Teammates Predict the ball there?
    for (auto teammate : theTeamData.teammates)
    {
      if ((teammate.theBallModel.estimate.position - theBallModel.estimate.position).norm() < minDistanceBallSent)
      {
        minDistanceBallSent = (teammate.theBallModel.estimate.position - theBallModel.estimate.position).norm();
      }
    }
    if (minDistanceBallSent > minDistanceLastTeamBall)
    {
      // if (theRobotInfo.number == 1 && theOwnTeamInfo.teamNumber == 2)
      // {
      //   TLOGD(tlogger, "We don't agree with TeamBallModel: Local: x = {}, y = {}, Team: x = {}, y = {}",
      //         theBallModel.estimate.position.x(), theBallModel.estimate.position.y(), theTeamBallModel.position.x(),
      //         theTeamBallModel.position.y());
      // }
      messageDesirability.reasonToSend = MessageDesirability::ballMoved;
      return true;
    }
  }
  return false;
}


bool TeamMessageHandler2023b::checkRoleChanged()
{
  //All assigned by captain but how does captain know who else is on field initially!?
  //Answer: When message Recieved added to Teammate list

 //As Role assignment centralised here if new captain, need send

  //Only other Teammate to note if changed is striker!
  if  (theTeamBehaviorStatus.teammateRoles.captain == theRobotInfo.number)
  {
    int i=0;
    for (auto teammate : theTeamBehaviorStatus.teammateRoles.roles)
    {
      if (teammate == PlayerRole::ballPlayer)
        if (i != previousStriker)
        {
          previousStriker = i;

          messageDesirability.reasonToSend = MessageDesirability::roleChanged;
          return true;
        }
      i++;
    }
  }
  return false;
}


bool TeamMessageHandler2023b::checkPlayerMoved()
{
  return false;
}


bool TeamMessageHandler2023b::checkReturnFromPenalty()
{
  // note: wasPenalized is updated in checkStateAllowsSending

  if (wasPenalized && (theExtendedGameInfo.timeSinceLastPenaltyEnded >= returnFromPenaltyDelay))
  {
    messageDesirability.reasonToSend = MessageDesirability::returnedFromPenalty;
    wasPenalized = false;

    TLOGV(tlogger, "checkReturnFromPenalty: wasPenalized before, timeSinceEnded {}",
          theExtendedGameInfo.timeSinceLastPenaltyEnded);

    return true;
  }

  return false;
}
