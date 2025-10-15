/**
 * @file TeamMessageHandler2025.cpp
 * 
 * The general architecture is based on the original TeamMessageHandler by B-Human,
 * but replacing the particle stuff and generally making the different parts of
 * communication more explicit.
 * 
 * The TeamMessage2025OutputGenerator representation controls the sending of
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
 * - updateWithReceivedMessage calls readSPLStandardMessage/readInputBytes to decode the bytes to a TeamMessage2025 struct,
 *   parseIntoBMate to copy relevant values from TeamMessage2025 to Teammate struct
 * 
 * The procedure for sending a message to teammates is:
 * - Cognition::afterModules calls TeamMessage2025OutputGenerator::sendIfNeeded.
 *   This happens at the end of each cycle
 * - sendIfNeeded checks shouldSendThisFrame and if this is true it calls
 *   generateOutputMessage to fill in the data to be sent, 
 *   writeOutputBytes to convert that data to encoded bytes (compressing numbers etc where possible)
 *   and TeamMessageCommunicator::send to actually send the data on the WLAN
 *
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#include "TeamMessageHandler2025.h"
#include "Tools/Communication/BitStreamCodec.h"

#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Tools/Math/Constants.h"
#include "Platform/File.h"
#include "Platform/Time.h"
#include "Platform/SystemCall.h"

#include "Tools/TextLogging.h"
#include "Tools/FmtCommonTypes.h"
#include "Tools/Debugging/Annotation.h"

#include "Tools/Math/Pose3f.h"
#include "Tools/Debugging/DebugDrawings3D.h"


DECL_TLOGGER(txLogger, "TeamMessageHandler2025.tx", TextLogging::WARNING);
DECL_TLOGGER(mdLogger, "TeamMessageHandler2025.desirability", TextLogging::WARNING);
DECL_TLOGGER(rxLogger, "TeamMessageHandler2025.rx", TextLogging::WARNING);

MAKE_MODULE(TeamMessageHandler2025, communication);


// this is a helper struct to collect Bitstream types to make coding/decoding a little bit more readable.
// Each member is a handler for a distinct type or range. If we want to encode
// multiple values with the same type and range we should usually reuse the same helper
// member.
namespace { // anonymous namespace so that TeamMessageCodec is only known in this file

struct TeamMessageCodec
{
  BitStream::Bits<unsigned> header {"header", 5}; // 5 bit header
  BitStream::Bits<unsigned> version {"version", 3}; // 3 bit version
  BitStream::Integer<int> sizeOfMessageBody {"sizeOfMessageBody", 0, 255};

  BitStream::Integer<int> playerNum {"playerNum", -1, Settings::highestValidPlayerNumber}; // sometimes we mark invalid players/slots using -1
  BitStream::Integer<int> teamNum {"teamNum", -1, 100};
  BitStream::Integer<uint8_t> seqNumGC {"seqNumGC", 0, 255};
  BitStream::Timestamp timeReceivedGC {"timeReceivedGC", 0, 0xFFFFFFFF, 1, 24}; // this is a truncated version of the real timestamp with ms precision that will wrap every 4.5 hours
  BitStream::Timestamp timestamp {"timestamp", 0, 0xFFFFFFFF, 1, 24}; // this is a truncated version of the real timestamp with ms precision that will wrap every 4.5 hours

  BitStream::Bool isPenalized { "isPenalized" };
  BitStream::Bool isUpright { "isUpgright" };
  BitStream::Bool kickedSinceRestart { "kickedSinceRestart" };

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
  BitStream::Floatf behaviourStatusWalkingToX { "behaviourStatusWalkingToX", -6400, 6400, 8};
  BitStream::Floatf behaviourStatusWalkingToY { "behaviourStatusWalkingToY", -6400, 6400, 8};
  BitStream::Floatf behaviourStatusSpeed { "behaviourStatusSpeed", -400, 400, 8};
  BitStream::Floatf behaviourStatusShootingToX { "behaviourStatusShootingToX", -6400, 6400, 8};
  BitStream::Floatf behaviourStatusShootingToY { "behaviourStatusShootingToY", -6400, 6400, 8};


  BitStream::ContiguousEnum<Formations::FormationId> activeTacticStatusFormationId
      { "activeTacticStatusFormationId", Formations::numFormationId };
  BitStream::ContiguousEnum<Formations::FormationRole> activeTacticStatusFormationRole
      { "activeTacticStatusFormationRole", Formations::numFormationRole };
  BitStream::ContiguousEnum<PlayerRole::Type> activeTacticStatusSituationRole
      { "activeTacticStatusSituationRole", PlayerRole::numType };


  BitStream::Bool whistleDetectionActive { "whistleDetectionActive" };
  BitStream::Floatf whistleConfidenceLastWhistleDetection { "whistleConfidenceLastWhistleDetection", 0, 2.55f, 8};
  BitStream::RelativeTimestamp whistleFrameTimeLastWhistleDetection {
      "whistleFrameTimeLastWhistleDetection", BitStream::RelativeTimestamp::beforeRef, 4000, 250, 0, 30000};

  BitStream::RelativeTimestamp standbyReadyGestureFrameTimeLastDetection {
      "standbyReadyGestureFrameTimeLastDetection", BitStream::RelativeTimestamp::beforeRef, 4000, 500, 0, 30000};


  BitStream::ContiguousEnum<SetPlayRefereeGesture::Gesture> setPlayRefereeGestureLastDetection
      { "setPlayRefereeGestureLastDetection", SetPlayRefereeGesture::numGesture };
  BitStream::RelativeTimestamp setPlayRefereeGestureFrameTimeLastDetection {
      "setPlayRefereeGestureFrameTimeLastDetection", BitStream::RelativeTimestamp::beforeRef, 4000, 500, 0, 30000};


  inline float replaceInfNan(float value, float replaceValue)
  {
    return std::isfinite(value) ? value : replaceValue;
  }
};

static TeamMessageCodec codec; // local to this file

const unsigned HEADER = 0b01110;
const unsigned VERSION = 3 & 0b111; // second part is the bitmask - 2025 version

} // end of anonymous namespace



TeamMessageHandler2025::TeamMessageHandler2025()
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

void TeamMessageHandler2025::update(TeamMessage2025OutputGenerator& outputGenerator)
{
  // DECLARE_PLOT("module:TeamMessageHandler2025:SPLStdMsgDataMin");
  // DECLARE_PLOT("module:TeamMessageHandler2025:SPLStdMsgDataDesired");
  // DECLARE_PLOT("module:TeamMessageHandler2025:SPLStdMsgDataActual");

  if (!messageDesirabilityUpdated)
    updateMessageDesirability();

  isUpright =
      (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering ||
      theFallDownState.state == FallDownState::squatting) &&
      (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp &&
      theMotionRequest.motion != MotionRequest::getUp);

  if (isUpright)
    frameTimeWhenLastUpright = theFrameInfo.time;

  // send will be called in Cognition after all modules have executed
  outputGenerator.sendIfNeeded = [this, &outputGenerator]()
  {
    // TLOGD(txLogger, "sendIfNeeded");
    if (messageDesirability.shouldSend)
    { 
      TLOGV(txLogger, "outputGenerator shouldSend so about to send");    
      generateOutputMessage(outputGenerator);
      writeOutputMessageBytes(outputGenerator, outputTeamMessageBytes);
      sendMessageBytes(outputGenerator, outputTeamMessageBytes);
    }
  };

  if (((outputGenerator.sentMessages == 1) && (sentMessagesLogTime == 0)) ||
      ((outputGenerator.sentMessages > sentMessagesLogged) &&
       (theFrameInfo.getTimeSince(sentMessagesLogTime) >= 15000)))
  {
    TLOGI(txLogger, "sentMessages {}, sender {}, last timestamp {} and frameTimeSent {}", outputGenerator.sentMessages,
          theGameInfo.playerNumber, sentTeamMessage.teamMessage.timestamp, sentTeamMessage.frameTime);

    sentMessagesLogged = outputGenerator.sentMessages;
    sentMessagesLogTime = theFrameInfo.time;
  }
  else if ((outputGenerator.sentMessages == sentMessagesLogged) &&
           (theFrameInfo.getTimeSince(sentMessagesLogTime) >= 15000))
  {
    TLOGI(txLogger, "sentMessages {}, no messages sent in last period", outputGenerator.sentMessages);
    sentMessagesLogTime = theFrameInfo.time;
  }

  DECLARE_DEBUG_DRAWING3D("representation:SentTeamMessage2025", "field");

  // drawing of sent messages (sent previous cycle)
  COMPLEX_DRAWING3D("representation:SentTeamMessage2025")
  {
    const int fadeMs = 400;
    if (theFrameInfo.getTimeSince(sentTeamMessage.frameTime) < fadeMs)
    {
      float alpha = (fadeMs - theFrameInfo.getTimeSince(sentTeamMessage.frameTime)) / static_cast<float>(fadeMs);

      // orient the x-axis of the pose out of the 3D field at the robot pose
      Pose3f origin = Pose3f(theRobotPose.translation.x(), theRobotPose.translation.y(), 0).rotateY(-90_deg);

      // CIRCLE3D("representation:SentTeamMessage2025", origin, 220, 1, ColorRGBA::white.alpha(alpha));
      PARTDISC3D("representation:SentTeamMessage2025", origin, 200, 250, -180_deg, 179_deg, ColorRGBA::violet.alpha(alpha));
    }
  }
}


void TeamMessageHandler2025::generateOutputMessage(TeamMessage2025OutputGenerator& outputGenerator) const
{
  TeamMessage2025& teamMessage = outputGenerator.teamMessage;

  // outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);

  teamMessage.playerNum = static_cast<uint8_t>(theGameInfo.playerNumber);

  // timeSyncInfo
  teamMessage.seqNumGC = theReceivedGameControlData.packetNumber;
  teamMessage.timeReceivedGC = theReceivedGameControlData.timeLastPacketReceived;

  // outputGenerator.teamMessage.seqNum = outputGenerator.sentMessages % 15; // TBD not used

  teamMessage.timestamp = Time::getCurrentSystemTime();

  teamMessage.isPenalized = (theGameInfo.isPenalized());
  teamMessage.isUpright = isUpright;

  prevFrameKickedSinceRestart = kickedSinceRestart;

  int timeSinceLastKick = theFrameInfo.getTimeSince(theMotionInfo.lastKickTimestamp);
  teamMessage.kickedSinceRestart = (timeSinceLastKick < theExtendedGameInfo.timeSincePlayingStarted) &&
                                   (timeSinceLastKick < theExtendedGameInfo.timeSinceFreeKickStarted);
  kickedSinceRestart = teamMessage.kickedSinceRestart;

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
  teamMessage.behaviourStatus.walkingTo = theBehaviorStatus.walkingTo;
  teamMessage.behaviourStatus.speed = theBehaviorStatus.speed;

  // ActiveTacticStatus
  teamMessage.activeTacticStatus = static_cast<const ActiveTacticStatus&>(theActiveTactic);

  // Whistle
  teamMessage.whistle.frameTimeLastWhistleDetection = theWhistle.lastTimeWhistleDetected;

  teamMessage.standbyReadyGesture.frameTimeLastDetection = theStandbyReadyGesture.frameTimeLastDetection;

  // TODO - check the logic here, but for now, only overwrite detected gesture when a new one is detected
  if (theSetPlayRefereeGesture.frameTimeLastDetection != teamMessage.setPlayRefereeGesture.frameTimeLastDetection)
  {
    teamMessage.setPlayRefereeGesture.lastDetection = theSetPlayRefereeGesture.lastDetection;
    teamMessage.setPlayRefereeGesture.frameTimeLastDetection = theSetPlayRefereeGesture.frameTimeLastDetection;
  }
}


void TeamMessageHandler2025::writeOutputMessageBytes(TeamMessage2025OutputGenerator &outputGenerator,
                                               TeamMessageBytes& teamMessageBytes) const
{
  TeamMessage2025& teamMessage = outputGenerator.teamMessage;

  teamMessageBytes.size = teamMessageBytes.capacity; // initial size = max

  BitStream::Writer headerWriter(teamMessageBytes.data, 2);
  BitStream::Writer writer(teamMessageBytes.data + 2, (unsigned) teamMessageBytes.size-2);


  codec.header.write(headerWriter, HEADER);
  codec.version.write(headerWriter, VERSION);

  // The next byte to be written to the header is the byte count for all 
  // remaining data. We don't write this immediately. Instead, once all the 
  // values are written we come back and update 
  // the byte count in the header

  codec.teamNum.write(writer, Global::getSettings().teamNumber);
  codec.playerNum.write(writer, teamMessage.playerNum);

  // timeSyncInfo
  codec.seqNumGC.write(writer, teamMessage.seqNumGC);
  codec.timeReceivedGC.write(writer, teamMessage.timeReceivedGC);

  // writer.writeInteger(outputGenerator.sentMessages % 15, 0, 14, 1, 4); // sequence number if needed

  codec.timestamp.write(writer, teamMessage.timestamp);

  TLOGV(txLogger, "writeOutputMessageBytes: team {}, player {}, seqNumGC {}, timeReceivedGC {}, timestamp {}",
        Global::getSettings().teamNumber, teamMessage.playerNum, teamMessage.seqNumGC, teamMessage.timeReceivedGC,
        teamMessage.timestamp);

  codec.isPenalized.write(writer, teamMessage.isPenalized);
  codec.isUpright.write(writer, teamMessage.isUpright);
  // TODO: check if frameTimeWhenLastUpright is used/need

  codec.kickedSinceRestart.write(writer, teamMessage.kickedSinceRestart);

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
  codec.behaviourStatusWalkingToX.write(writer, teamMessage.behaviourStatus.walkingTo.x());
  codec.behaviourStatusWalkingToY.write(writer, teamMessage.behaviourStatus.walkingTo.y());
  codec.behaviourStatusSpeed.write(writer, teamMessage.behaviourStatus.speed);
  codec.behaviourStatusShootingToX.write(writer, teamMessage.behaviourStatus.shootingTo.x());
  codec.behaviourStatusShootingToY.write(writer, teamMessage.behaviourStatus.shootingTo.y());

  // ActiveTacticStatus
  codec.activeTacticStatusFormationId.write(writer, teamMessage.activeTacticStatus.formationId);
  codec.activeTacticStatusFormationRole.write(writer, teamMessage.activeTacticStatus.formationRole);  
  codec.activeTacticStatusSituationRole.write(writer, teamMessage.activeTacticStatus.situationRole);
  
  // Whistle
  codec.whistleFrameTimeLastWhistleDetection.write(writer, teamMessage.whistle.frameTimeLastWhistleDetection,
                                                   teamMessage.timestamp);

  // Standby ready gesture
  codec.standbyReadyGestureFrameTimeLastDetection.write(writer, teamMessage.standbyReadyGesture.frameTimeLastDetection,
                                                        teamMessage.timestamp);

  // set play referee gesture                                                        
  // codec.setPlayRefereeGestureLastDetection.write(writer, teamMessage.setPlayRefereeGesture.lastDetection);
  // codec.setPlayRefereeGestureFrameTimeLastDetection.write(writer, teamMessage.setPlayRefereeGesture.frameTimeLastDetection,
  //                                                       teamMessage.timestamp);

  // finalize the buffers and sizes - once we know the (encoded) message body size
  // we need to add in that size to the header at which point our entire message
  // has been encoded into teamMessageBytes
  writer.finalizeBuffer();
  int messageBodySize = writer.size();
  codec.sizeOfMessageBody.write(headerWriter, messageBodySize);

  // TLOGI(txLogger, "MessageBody size = {}", messageBodySize);

  headerWriter.finalizeBuffer();

  // the underlying data array, teamMessageBytes.data, is now up to date
  // so ensure the final size is correct
  teamMessageBytes.size = messageBodySize + 2; // add 2 for the header

  // TODO tmp debugging code
  // checkOutputMessageBytes(teamMessage, teamMessageBytes);
}

void TeamMessageHandler2025::checkOutputMessageBytes(const TeamMessage2025 &originalMessage,
                                                      const TeamMessageBytes &teamMessageBytes) const
{
  BitStream::Reader reader(teamMessageBytes.data, teamMessageBytes.size);
  TeamMessage2025 decodedMessage;

  unsigned header;
  codec.header.read(reader, header);
  ASSERT(header == HEADER);

  unsigned version;
  codec.version.read(reader, version);
  ASSERT(version == VERSION);

  int len;
  codec.sizeOfMessageBody.read(reader, len); // TODO is this really needed?

  // read team and player numbers

  int teamNumber;
  codec.teamNum.read(reader, teamNumber);
  ASSERT(teamNumber == Global::getSettings().teamNumber);

  codec.playerNum.read(reader, decodedMessage.playerNum);
  ASSERT(decodedMessage.playerNum == originalMessage.playerNum);

  // timeSyncInfo
  codec.seqNumGC.read(reader, decodedMessage.seqNumGC);
  codec.timeReceivedGC.read(reader, decodedMessage.timeReceivedGC);

  ASSERT(decodedMessage.seqNumGC == originalMessage.seqNumGC);
  ASSERT(codec.timeReceivedGC.checkQuantEqual(decodedMessage.timeReceivedGC, originalMessage.timeReceivedGC));

  codec.timestamp.read(reader, decodedMessage.timestamp);
  ASSERT(codec.timestamp.checkQuantEqual(decodedMessage.timestamp, originalMessage.timestamp));

  codec.isPenalized.read(reader, decodedMessage.isPenalized);
  codec.isUpright.read(reader, decodedMessage.isUpright);
  // TODO: check if frameTimeWhenLastUpright is used/need

  ASSERT(decodedMessage.isPenalized == originalMessage.isPenalized);
  ASSERT(decodedMessage.isUpright == originalMessage.isUpright);

  // robotPose
  codec.robotPoseX.read(reader, decodedMessage.robotPose.pose.translation.x());
  codec.robotPoseY.read(reader, decodedMessage.robotPose.pose.translation.y());
  codec.robotPoseRotation.read(reader, decodedMessage.robotPose.pose.rotation);
  codec.robotPosePosSD.read(reader, decodedMessage.robotPose.translationSD);
  codec.robotPoseRotationSD.read(reader, decodedMessage.robotPose.rotationSD);
  codec.robotPoseQuality.read(reader, decodedMessage.robotPose.quality);

  ASSERT(codec.robotPoseX.checkQuantEqual(decodedMessage.robotPose.pose.translation.x(), originalMessage.robotPose.pose.translation.x()));
  ASSERT(codec.robotPoseY.checkQuantEqual(decodedMessage.robotPose.pose.translation.y(), originalMessage.robotPose.pose.translation.y()));
  ASSERT(codec.robotPoseRotation.checkQuantEqual(decodedMessage.robotPose.pose.rotation, originalMessage.robotPose.pose.rotation));
  ASSERT(codec.robotPosePosSD.checkQuantEqual(decodedMessage.robotPose.translationSD, originalMessage.robotPose.translationSD));
  ASSERT(codec.robotPoseRotationSD.checkQuantEqual(decodedMessage.robotPose.rotationSD, originalMessage.robotPose.rotationSD));
  ASSERT(decodedMessage.robotPose.quality == originalMessage.robotPose.quality);

  // ballModel
  codec.ballPosX.read(reader, decodedMessage.ballModel.position.x());
  codec.ballPosY.read(reader, decodedMessage.ballModel.position.y());
  codec.ballVelocityX.read(reader, decodedMessage.ballModel.velocity.x());
  codec.ballVelocityY.read(reader, decodedMessage.ballModel.velocity.y());
  codec.ballPosSD.read(reader, decodedMessage.ballModel.positionSD);
  codec.ballPosLastSeenX.read(reader, decodedMessage.ballModel.positionLastSeen.x());
  codec.ballPosLastSeenY.read(reader, decodedMessage.ballModel.positionLastSeen.y());

  codec.ballFrameTimeLastSeen.read(reader, decodedMessage.ballModel.frameTimeLastSeen, decodedMessage.timestamp);

  ASSERT(codec.ballPosX.checkQuantEqual(decodedMessage.ballModel.position.x(), originalMessage.ballModel.position.x()));
  ASSERT(codec.ballPosY.checkQuantEqual(decodedMessage.ballModel.position.y(), originalMessage.ballModel.position.y()));
  ASSERT(codec.ballVelocityX.checkQuantEqual(decodedMessage.ballModel.velocity.x(), originalMessage.ballModel.velocity.x()));
  ASSERT(codec.ballVelocityY.checkQuantEqual(decodedMessage.ballModel.velocity.y(), originalMessage.ballModel.velocity.y()));
  ASSERT(codec.ballPosSD.checkQuantEqual(decodedMessage.ballModel.positionSD, originalMessage.ballModel.positionSD));
  ASSERT(codec.ballPosLastSeenX.checkQuantEqual(decodedMessage.ballModel.positionLastSeen.x(), originalMessage.ballModel.positionLastSeen.x()));
  ASSERT(codec.ballPosLastSeenY.checkQuantEqual(decodedMessage.ballModel.positionLastSeen.y(), originalMessage.ballModel.positionLastSeen.y()));

  // frameInfo
  codec.frameTime.read(reader, decodedMessage.frameTime, decodedMessage.timestamp);
  ASSERT(codec.frameTime.checkQuantEqual(decodedMessage.frameTime, decodedMessage.timestamp, 
                                         originalMessage.frameTime, originalMessage.timestamp));

  // Whistle
  codec.whistleFrameTimeLastWhistleDetection.read(reader, decodedMessage.whistle.frameTimeLastWhistleDetection,
                                                  decodedMessage.timestamp);
  ASSERT(codec.whistleFrameTimeLastWhistleDetection.checkQuantEqual(
      decodedMessage.whistle.frameTimeLastWhistleDetection, decodedMessage.timestamp,
      originalMessage.whistle.frameTimeLastWhistleDetection, originalMessage.timestamp));

  // TODO - gestures
}


void TeamMessageHandler2025::sendMessageBytes(TeamMessage2025OutputGenerator &outputGenerator,
                                               const TeamMessageBytes &teamMessageBytes)
{
  if (!teamMessageSocketHandler.send(teamMessageBytes)) // it is possible that the port has not been set
    return;

  outputGenerator.sentMessages++;

  sentTeamMessage.teamMessage = outputGenerator.teamMessage;
  sentTeamMessage.frameTime = theFrameInfo.time;

  // TODO
  // PLOT("module:TeamMessageHandler2025:SPLStdMsgDataCore", offset);
  // PLOT("module:TeamMessageHandler2025:SPLStdMsgDataDesired", desiredTotalSize);
  // PLOT("module:TeamMessageHandler2025:SPLStdMsgDataActual", outputGenerator.theBSPLStandardMessage.numOfDataBytes);

  // TLOGV(txLogger, "SPL data core={}, desired={}, actual={}", offset, desiredTotalSize,
  //       outputGenerator.theBSPLStandardMessage.numOfDataBytes);

  // if ((outputGenerator.sentMessages == 1) || (outputGenerator.sentMessages % 15 == 0))
  //   TLOGI(txLogger, "sent msg from {}, sentMessages {}, timestamp {}, frameTimeSent {}",
  //         theGameInfo.playerNumber, outputGenerator.sentMessages, outputGenerator.teamMessage.timestamp, theFrameInfo.time);
  // else
    TLOGD(txLogger, "sent msg from {}, sentMessages {}, timestamp {}, frameTimeSent {}", theGameInfo.playerNumber,
          outputGenerator.sentMessages, outputGenerator.teamMessage.timestamp, theFrameInfo.time);
}

// ============================================================================
// ============================================================================
// RECEIVING TEAM MESSAGES
// ============================================================================
// ============================================================================


void TeamMessageHandler2025::update(TeamData& teamData)
{
  if (teamData.receivedMessages == 0)
  {
    teamData.teammateMessageTimestamps.fill(0);
    teamData.teammateMessageCounts.fill(0);
  }

  messageDesirabilityUpdated = false;

  // update timeSync with the latest GC times we have before processing received team messages
  timeSyncInfo.updateGCInfo(theReceivedGameControlData.packetNumber, theReceivedGameControlData.timeLastPacketReceived, theFrameInfo.time);

  teamMessageSocketHandler.receive(inputTeamMessageBytesBuffer); // pushes received message onto back of buffer, so oldest message is the front of the buffer
  if (!inputTeamMessageBytesBuffer.empty())
    TLOGV(rxLogger, "update(TeamData), after socket receive, inputTeamMessageBytesBuffer size {}",
        inputTeamMessageBytesBuffer.size());

  // decode and process each of the received messages
  // bool wasEmpty = inputTeamMessageBytesBuffer.empty();

  size_t numMessages = inputTeamMessageBytesBuffer.size();
  for (unsigned iMessage = 1; !inputTeamMessageBytesBuffer.empty(); iMessage++)
  {
    const ReceivedTeamMessageBytes& inputMessageBytes = inputTeamMessageBytesBuffer.front(); // first received message first

    TLOGV(rxLogger, "readInputMessageBytes message {} of {} ({})", iMessage, numMessages, (void*)&inputMessageBytes);
    TLOGV(rxLogger, "inputTeamMessageBytesBuffer: {}", inputTeamMessageBytesBuffer.debugString());


    if (readInputMessageBytes(inputMessageBytes, receivedTeamMessage))
    {
      // at this point we have successfully decoded the input messsage bytes into
      // the receivedTeamMessage structure

      TLOGV(rxLogger, "readInputMessageBytes was successful");

      // we've received a valid message
      teamData.receivedMessages++;
      teamData.teammateMessageTimestamps[receivedTeamMessage.playerNum] = receivedTeamMessage.timestamp;
      teamData.teammateMessageCounts[receivedTeamMessage.playerNum]++;

      receivedMessagePlayerNum = receivedTeamMessage.playerNum;

      parseMessageIntoTeammate(receivedTeamMessage, getTeammate(teamData, receivedTeamMessage.playerNum));

      // if ((teamData.receivedMessages == 1) || (teamData.receivedMessages % 15 == 0))
      //   TLOGI(rxLogger, "have now received {} messages, latest from {} (teammateMsgCount {}), timestamp {}",
      //         teamData.receivedMessages, receivedTeamMessage.playerNum,
      //         teamData.teammateMessageCounts[receivedTeamMessage.playerNum], receivedTeamMessage.timestamp);
      // else
        TLOGD(rxLogger, "received msg from {}, timestamp {}, msgCount teamate/total {}/{}",
              receivedTeamMessage.playerNum, receivedTeamMessage.timestamp,
              teamData.teammateMessageCounts[receivedTeamMessage.playerNum], teamData.receivedMessages);
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

  lastGCPacketReceivedTime = theGameInfo.timeLastPacketReceived;

  if (((teamData.receivedMessages == 1) && (receivedMessagesLogTime == 0)) ||
      ((teamData.receivedMessages > receivedMessagesLogged) &&
       (theFrameInfo.getTimeSince(receivedMessagesLogTime) >= 15000)))
  {
    ASSERT(receivedMessagePlayerNum != -1);

    TLOGI(rxLogger, "have now received {} messages:", teamData.receivedMessages);
    if (rxLogger.isEnabled(TextLogging::INFO))
    {
      for (int i = Settings::lowestValidPlayerNumber; i <= Settings::highestValidPlayerNumber; i++)
        if (teamData.teammateMessageCounts[i] > 0)
          TLOGI(rxLogger, "    player {}, received count {}, last timestamp {}", i,
                teamData.teammateMessageCounts[receivedMessagePlayerNum],
                teamData.teammateMessageTimestamps[receivedMessagePlayerNum]);
    }

    receivedMessagesLogged = teamData.receivedMessages;
    receivedMessagesLogTime = theFrameInfo.time;
  }
  else if ((teamData.receivedMessages == receivedMessagesLogged) &&
           (theFrameInfo.getTimeSince(receivedMessagesLogTime) >= 15000))
  {
    TLOGI(rxLogger, "have received {} messages to date - no messages in last period", teamData.receivedMessages);
    receivedMessagesLogTime = theFrameInfo.time;
  }
}

void TeamMessageHandler2025::maintainTeammateList(TeamData& teamData) const
{
  const GameInfo::TeamInfo& ourTeam = theGameInfo.ourTeam();

  // --------------------------------------------------------------------------
  // update teamData based on GC messages
  // first whether robots are penalized or not (and might need to be removed)
  // --------------------------------------------------------------------------

  for  (auto& teammate : teamData.teammates)
  {
    const GameInfo::RobotInfo& teammateRobotInfo = ourTeam.getPlayer(Settings::toPlayerIndex(teammate.number));
    Teammate::Status newStatus = Teammate::PLAYING;

    // ensure teammates update isPenalized - it won;t be updated by the robot
    // itself since it cannot send messages while penalized
    if (teammateRobotInfo.isPenalized())
      teammate.isPenalized = true;

    if  (teammate.isPenalized)
    {
      if (teammateRobotInfo.isPenalized(PENALTY_SPL_REQUEST_FOR_PICKUP) ||
          teammateRobotInfo.isPenalized(PENALTY_SUBSTITUTE))
        newStatus = Teammate::ABSENT;
      else
        newStatus = Teammate::PENALIZED;
    }
    else if  (!teammate.isUpright) // || !teammate.hasGroundContact)
    {
      newStatus = Teammate::FALLEN;
    }

    if  (newStatus != teammate.status)
    {
      teammate.status = newStatus;
      teammate.timeWhenStatusChanged = theFrameInfo.time;
    }

    if ((theExtendedGameInfo.timeSincePlayingStarted == 0) || (theExtendedGameInfo.timeSinceFreeKickStarted == 0))
      teammate.kickedSinceRestart = false;

    teammate.isGoalkeeper = Settings::isGoalkeeper(teammate.number);
  }

  // --------------------------------------------------------------------------
  // add robots if they are just now appearing in GC, but only in the INITIAL, STANDBY, or READY states.
  // In other states or if returning from penalty, robots have to announce themselves
  // --------------------------------------------------------------------------
  if ((theGameInfo.timeLastPacketReceived != lastGCPacketReceivedTime) // new GC packet received
      && ((theGameInfo.state == STATE_INITIAL) || (theGameInfo.state == STATE_STANDBY) ||
          (theGameInfo.state == STATE_READY)))
  {
    // first make it easy to lookup the existing set of known robots
    std::vector<Teammate *> teammatesByNumber(Settings::highestValidPlayerNumber + 1, nullptr);
    for (auto &teammate : teamData.teammates)
      teammatesByNumber[teammate.number] = &teammate;

    for (unsigned i = 0; i < theGameInfo.playersPerTeam; i++)
    {
      const GameInfo::RobotInfo &teammateRobotInfo = ourTeam.getPlayer(i);
      int teammateNumber = Settings::toPlayerNumber(i);

      // don't include self in teammates
      if (teammateNumber == theGameInfo.playerNumber)
        continue;

      // is player[i] in the game but not in TeamData
      if (teammatesByNumber[teammateNumber] == nullptr &&
          !teammateRobotInfo.isPenalized(PENALTY_SPL_REQUEST_FOR_PICKUP) &&
          !teammateRobotInfo.isPenalized(PENALTY_SUBSTITUTE))
      {
        TLOGI(rxLogger, "maintainTeammateList: adding teammate {} based on GC info", teammateNumber);

        // if we get this far, we have a new player

        teamData.teammates.emplace_back(); // default constructed
        Teammate& teammate = teamData.teammates.back();

        // initialise teammate from game controller info
        teammate.number = teammateNumber;
        teammate.isGoalkeeper = Settings::isGoalkeeper(teammate.number);
        teammate.isPenalized = (ourTeam.players[i].penalty != PENALTY_NONE);
        teammate.status = (teammate.isPenalized ? Teammate::PENALIZED : Teammate::PLAYING);
        teammate.timeWhenStatusChanged = theFrameInfo.time;
        teammate.timeWhenLastUpdated = theFrameInfo.time;

        const SetupPoses::SetupPose &p = theSetupPoses.getPoseOfRobot(teammate.number);
        teammate.theRobotPose.translation = p.position;
        teammate.theRobotPose.rotation = (p.turnedTowards - p.position).angle();
        teammate.theRobotPose.onRead();

        // default ball model is OK - nothing to add here

        teammate.theFrameInfo = theFrameInfo;

        // default ObstacleModel is OK - nothing to add here

        if (!theActiveTactic.formationPoses.empty())
          teammate.theBehaviorStatus.walkingTo = teammate.theRobotPose.toLocal(
              theActiveTactic.formationPoses[Settings::toPlayerIndex(teammate.number)].translation);
        else
          teammate.theBehaviorStatus.walkingTo = teammate.theRobotPose.toLocal(teammate.theRobotPose.translation);

        teammate.theBehaviorStatus.speed = theWalkingEngineOutput.maxSpeed.translation.x(); // we guess full speed
        
        // Whistle is OK - nothing to add here
        // we try the default TeamBehaviorStatus also - hopefully it is OK
      }
    }
  }

  // --------------------------------------------------------------------------
  // Remove absent robots
  // Penalized robots will not appear in the activeTeammates list after onRead is called
  // --------------------------------------------------------------------------

  for (auto teammate = teamData.teammates.begin(); teammate != teamData.teammates.end();)
  {
    if (teammate->status == Teammate::ABSENT)
    {
      TLOGV(rxLogger, "maintainTeammateList: remove teammate (player {}) because ABSENT", teammate->number);
      teammate = teamData.teammates.erase(teammate);
    }
    // FIXME - this will lead to immediate timeout after penalty ends
    else if ((theGameInfo.state == STATE_PLAYING) &&
              (theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout))
    {
      TLOGV(rxLogger, "maintainTeammateList: remove teammate (player {}) because no packets received in {} ms",
            teammate->number, networkTimeout);
      teammate = teamData.teammates.erase(teammate);
    }
    else
    {
      ++teammate;
    }
  }

  teamData.onRead(); // update the alternative access methods
}

#define PARSING_ERROR(...)                                                                                             \
  {                                                                                                                    \
    std::string s = fmt::format("readInputMessageBytes parsing error: " __VA_ARGS__);                                  \
    OUTPUT_ERROR(s);                                                                                                   \
    receivedTeamMessage.lastErrorCode = ReceivedTeamMessage::parsingError;                                             \
    return false;                                                                                                      \
  }

bool TeamMessageHandler2025::readInputMessageBytes(const ReceivedTeamMessageBytes &receivedMsgBytes,
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

  // TLOGV(rxLogger, "readInputMessageBytes header,version OK, sizeOfMessageBody = {}", len);

  // read team and player numbers

  int teamNumber;
  codec.teamNum.read(reader, teamNumber);
  if (teamNumber != Global::getSettings().teamNumber)
    PARSING_ERROR("Invalid team number received: {}, expected {}", teamNumber, Global::getSettings().teamNumber);

  codec.playerNum.read(reader, inputTeamMessage.playerNum);
  if (inputTeamMessage.playerNum < Settings::lowestValidPlayerNumber ||
      inputTeamMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received: {}", inputTeamMessage.playerNum);

  // TLOGV(rxLogger, "readInputMessageBytes from team {}, robot {}", teamNumber, inputTeamMessage.playerNum);


  // timeSyncInfo
  codec.seqNumGC.read(reader, inputTeamMessage.seqNumGC);
  codec.timeReceivedGC.read(reader, inputTeamMessage.timeReceivedGC);

  codec.timestamp.read(reader, inputTeamMessage.timestamp);

#ifndef SELF_TEST
  if(inputTeamMessage.playerNum == theGameInfo.playerNumber)
  {
    TLOGV(rxLogger,
          "readInputMessageBytes: ***myOwnMessage, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
          inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
    inputTeamMessage.lastErrorCode = ReceivedTeamMessage::myOwnMessage;
    return false;
  }
  else
    TLOGV(rxLogger,
          "readInputMessageBytes: player {}, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
          inputTeamMessage.playerNum, inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
#else
  TLOGV(rxLogger,
        "readInputMessageBytes: player {}, seqNumGC {}, timeReceivedGC {}, timestamp {} (all before timesync)",
        inputTeamMessage.playerNum, inputTeamMessage.seqNumGC, inputTeamMessage.timeReceivedGC, inputTeamMessage.timestamp);
#endif // !SELF_TEST


  // update timesync info based on the teammate's packet
  timeSyncInfo.updateTeammateInfo(inputTeamMessage.playerNum, inputTeamMessage.timestamp, inputTeamMessage.seqNumGC,
                                  inputTeamMessage.timeReceivedGC);

  if (!timeSyncInfo.getTeammateInfo(inputTeamMessage.playerNum).isValid())
  {
    OUTPUT_WARNING(
        fmt::format("TeamMessageHandler2025: rejecting Teammate Message from {} - unable to achieve timesync",
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

  codec.kickedSinceRestart.read(reader, inputTeamMessage.kickedSinceRestart);

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

  // behaviourStatus
  codec.behaviourStatusPassTarget.read(reader, inputTeamMessage.behaviourStatus.passTarget);
  codec.behaviourStatusWalkingToX.read(reader, inputTeamMessage.behaviourStatus.walkingTo.x());
  codec.behaviourStatusWalkingToY.read(reader, inputTeamMessage.behaviourStatus.walkingTo.y());
  codec.behaviourStatusSpeed.read(reader, inputTeamMessage.behaviourStatus.speed);
  codec.behaviourStatusShootingToX.read(reader, inputTeamMessage.behaviourStatus.shootingTo.x());
  codec.behaviourStatusShootingToY.read(reader, inputTeamMessage.behaviourStatus.shootingTo.y());

  // Active tactic
  codec.activeTacticStatusFormationId.read(reader, inputTeamMessage.activeTacticStatus.formationId);
  codec.activeTacticStatusFormationRole.read(reader, inputTeamMessage.activeTacticStatus.formationRole);
  codec.activeTacticStatusSituationRole.read(reader, inputTeamMessage.activeTacticStatus.situationRole);


  // Whistle
  // codec.whistleDetectionActive.read(reader, inputTeamMessage.whistle.detectionActive);
  // codec.whistleConfidenceLastWhistleDetection.read(reader, inputTeamMessage.whistle.confidenceLastWhistleDetection);
  codec.whistleFrameTimeLastWhistleDetection.read(reader, inputTeamMessage.whistle.frameTimeLastWhistleDetection,
                                                  inputTeamMessage.timestamp);

  // StandbyReadyGesture
  codec.standbyReadyGestureFrameTimeLastDetection.read(
      reader, inputTeamMessage.standbyReadyGesture.frameTimeLastDetection, inputTeamMessage.timestamp);

  // SetPlayRefereeGesture
  codec.setPlayRefereeGestureLastDetection.read(reader, inputTeamMessage.setPlayRefereeGesture.lastDetection);
  codec.setPlayRefereeGestureFrameTimeLastDetection.read(
      reader, inputTeamMessage.setPlayRefereeGesture.frameTimeLastDetection, inputTeamMessage.timestamp);


  inputTeamMessage.timeReceived = receivedMsgBytes.timeReceived;
  inputTeamMessage.frameTimeReceived = theFrameInfo.time;

  // ANNOTATION("TeamMessageHandler2025", "readSPLStandardMessage read OK");
  return true;
}

Teammate& TeamMessageHandler2025::getTeammate(TeamData& teamData, int playerNum) const
{
  // find the teammate by playerNum in our existing list (from prev cycle)
  for (auto &teammate : teamData.teammates)
    if (teammate.number == playerNum)
      return teammate;

  // not found => this teammate has just come online or come back from penalty
  teamData.teammates.emplace_back();

  TLOGV(rxLogger, "getTeammate: teammate added (player {})", playerNum);

  return teamData.teammates.back();
}


void TeamMessageHandler2025::parseMessageIntoTeammate(const ReceivedTeamMessage& inputTeamMessage, Teammate& teammate)
{
  teammate.number = inputTeamMessage.playerNum;

  // time info
  teammate.timeWhenLastPacketSent = inputTeamMessage.timestamp; // already converted to local time
  teammate.timeWhenLastPacketReceived = inputTeamMessage.frameTimeReceived;
  teammate.timeWhenLastUpdated = teammate.timeWhenLastPacketReceived;


  teammate.isPenalized = inputTeamMessage.isPenalized;
  teammate.isUpright = inputTeamMessage.isUpright;

  teammate.kickedSinceRestart = inputTeamMessage.kickedSinceRestart;

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

  // ObstacleModel
  // TODO: not included for now

  // BehaviorStatus
  teammate.theBehaviorStatus = BehaviorStatus(); // set defaults except where we override them
  // teammate.theBehaviorStatus.passTarget = inputTeamMessage.behaviourStatus.passTarget;
  teammate.theBehaviorStatus.walkingTo = inputTeamMessage.behaviourStatus.walkingTo;
  teammate.theBehaviorStatus.speed = inputTeamMessage.behaviourStatus.speed;

  // TeamBehaviorStatus - fill in values based on activeTacticStatus
  teammate.theTeamBehaviorStatus.teamActivity = TeamBehaviorStatus::noTeam; // we don't use this value anywhere
  teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBallStriker = std::numeric_limits<unsigned>::max();
  teammate.theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall = std::numeric_limits<unsigned>::max();

  teammate.theTeamBehaviorStatus.teammateRoles.captain = -1;

  teammate.theTeamBehaviorStatus.role.role = inputTeamMessage.activeTacticStatus.situationRole;
  teammate.theTeamBehaviorStatus.role.numOfActiveSupporters = 0;

  teammate.theTeamBehaviorStatus.passStatus.passKicker = -1;
  teammate.theTeamBehaviorStatus.passStatus.passReceiver = -1;
  teammate.theTeamBehaviorStatus.passStatus.numPasses = 0;

  // ActiveTacticStatus
  teammate.theActiveTacticStatus = inputTeamMessage.activeTacticStatus;

  // Whistle
  teammate.theWhistle.lastTimeWhistleDetected = inputTeamMessage.whistle.frameTimeLastWhistleDetection;
  teammate.theWhistle.confidenceOfLastWhistleDetection = (teammate.theWhistle.lastTimeWhistleDetected > 1) ? 0.9f : 0.f;
  teammate.theWhistle.channelsUsedForWhistleDetection = (teammate.theWhistle.lastTimeWhistleDetected > 1) ? 1 : 0; // simplify by assuming all robots operate on just 1 channel

  // StandbyReadyGesture  
  teammate.theStandbyReadyGesture.frameTimeLastDetection = inputTeamMessage.standbyReadyGesture.frameTimeLastDetection;
  bool newReadyDetection = (teammate.theStandbyReadyGesture.frameTimeLastDetection >= teammate.theFrameInfo.time);
  teammate.theStandbyReadyGesture.confidence = newReadyDetection ? 0.91f : 0.01f;
  teammate.theStandbyReadyGesture.isDetected = theFrameInfo.getTimeSince(teammate.theStandbyReadyGesture.frameTimeLastDetection) < 1000;

  // SetPlayRefereeGesture
  teammate.theSetPlayRefereeGesture.detectedGesture = SetPlayRefereeGesture::NONE; // we don't care about "current frame" detection for teammates
  teammate.theSetPlayRefereeGesture.confidence = 0.f;
  teammate.theSetPlayRefereeGesture.lastDetection = inputTeamMessage.setPlayRefereeGesture.lastDetection;
  teammate.theSetPlayRefereeGesture.frameTimeLastDetection = inputTeamMessage.setPlayRefereeGesture.frameTimeLastDetection;

  // ANNOTATION("TeamMessageHandler2025", "ParseMessageIntoTeammate seems OK");
}

// ============================================================================
// ============================================================================
// Message Desirability functionality
// ============================================================================
// ============================================================================

void TeamMessageHandler2025::updateMessageDesirability()
{
  messageDesirabilityUpdated = true;

  messageDesirability.shouldSend = false;
  messageDesirability.reasonToSend = MessageDesirability::noReasonToSend;
  messageDesirability.reasonNotToSend = MessageDesirability::noReasonNotToSend;

  updateLastTeamMessageTime();
  updateMessageRate(); // update MessageDesirability::remainingTeamMessageRateRatio before possible early return in next if statement

  // perform the old style message behaviour if appropriate
  if (sendAtMinPlayerInterval &&
      timeSinceTimeout(sentTeamMessage.frameTime, minPlayerSendInterval, "sentTeamMessage.frameTime"))
  {
    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;
    messageDesirability.shouldSend = true;
    TLOGD(mdLogger, "updateMessageDesirability shouldSend because sendAtMinPlayerInterval + pingNeeded");
    ANNOTATION("MessageDesirability",
               fmt::format("shouldSend reason = {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend)));
    return;
  }

  // check for reasons not to send (regardless of any other factors) first

  if (!checkStateAllowsSending() || !checkMessageBudgetAllowsSending() || !checkMessageRateAllowsSending())
  {
    // TLOGV(mdLogger, "updateMessageDesirability - should not send because {}",
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
    // TLOGV(mdLogger, "updateMessageDesirability - should not send because {}",
    //       TypeRegistry::getEnumName(messageDesirability.reasonNotToSend));
    return;
  }

  // now check for other reasons to send - stop at the first one that is a reason
  // to send (if any)

  if (messageDesirability.hasReasonToSend() || checkRoleChanged() || checkBallMoved() || checkPlayerMoved() ||
      checkUprightStatusChanged() || checkStandbyReadyGesture() || checkSetPlayRefereeGesture())
  {
    messageDesirability.shouldSend = true;
    TLOGD(mdLogger, "updateMessageDesirability shouldSend because {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend));
    ANNOTATION("MessageDesirability",
               fmt::format("shouldSend reason = {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend)));
  }

}

void TeamMessageHandler2025::updateLastTeamMessageTime()
{
  // did we send a message more recently than the last teammate?
  if (sentTeamMessage.frameTime > lastTeamMessageTime)
    lastTeamMessageTime = sentTeamMessage.frameTime;

  // did any teammate send a message more recently than us
  for (auto teammate : theTeamData.teammates)
  {
    // FIXME: this will currently fail to count packets sent by robots that have subsequently been penalized or become
    // inactive
    if (teammate.timeWhenLastPacketReceived > lastTeamMessageTime)
      lastTeamMessageTime = teammate.timeWhenLastPacketReceived;
  }
}


bool TeamMessageHandler2025::checkStateAllowsSending()
{
#if defined(TARGET_ROBOT) && !defined(SITTING_TEST)
  bool playingDead = (theMotionRequest.motion == MotionRequest::playDead) || 
                     (theMotionInfo.executedPhase == MotionPhase::playDead);
#else
  bool playingDead = false;
#endif

  if (!wasPenalized && theGameInfo.isPenalized())
    wasPenalized = true; // note: gets reseet to false in checkReturnFromPenalty

  bool gameSituationAllows = (theGameInfo.state != STATE_INITIAL) && (theGameInfo.state != STATE_FINISHED) &&
                             (!theGameInfo.isPenalized()) && (theGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT);

  if (playingDead || !gameSituationAllows)
    messageDesirability.reasonNotToSend = MessageDesirability::statePreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}


void TeamMessageHandler2025::updateMessageRate()
{
  // update message budget remaining
  messageDesirability.messageBudgetThisHalf = theGameInfo.ourTeam().messageBudget;
  if (theGameInfo.firstHalf)
    messageDesirability.messageBudgetThisHalf -= theGameConfig.messageBudget / 2;

  // corresponding actual message rate remaining
  float messageRateRemaining =
      theGameInfo.secsRemaining > 0
          ? static_cast<float>(messageDesirability.messageBudgetThisHalf) / theGameInfo.secsRemaining
          : 0.f;

  // use extraSecs so that we have the chance to send messages during the ready state if needed
  // (when secsRemaining is not going down). Interpolate extra down to zero as we get near the
  // end of the half
  int extraSecs = theGameInfo.secsRemaining > messageRateExtraSecs * 2
                      ? messageRateExtraSecs
                      : std::max(0, theGameInfo.secsRemaining - messageRateExtraSecs);

  float messageRateTarget = static_cast<float>(theGameConfig.messageBudget / 2) / (theGameConfig.secsInHalf + extraSecs);

  messageDesirability.remainingTeamMessageRateRatio = messageRateRemaining / messageRateTarget;
}


bool TeamMessageHandler2025::checkMessageBudgetAllowsSending()
{
  if (messageDesirability.messageBudgetThisHalf <= 10)
    messageDesirability.reasonNotToSend = MessageDesirability::messageBudgetPreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}


bool TeamMessageHandler2025::checkMessageRateAllowsSending()
{
  // const unsigned messageTime = addJitter(lastTeamMessageTime, intervalJitter); // was sentTeamMessage.frameTime, add jitter to prevent all robots sending at once
  const unsigned messageTime = addJitter(sentTeamMessage.frameTime, intervalJitter); // add jitter to prevent all robots sending at once

  // if (!timeSinceTimeout(messageTime, minTeamSendInterval, "messageTime") &&
  //     !timeSinceTimeout(messageTime, minPlayerSendInterval, "messageTime"))
  if (!timeSinceTimeout(messageTime, minPlayerSendInterval, "messageTime"))
    messageDesirability.reasonNotToSend = MessageDesirability::minIntervalPreventsSending;
  else if (messageDesirability.remainingTeamMessageRateRatio < 1.f)
    messageDesirability.reasonNotToSend = MessageDesirability::messageRatePreventsSending;

  return !messageDesirability.hasReasonNotToSend();
}

bool TeamMessageHandler2025::checkPingNeeded()
{
  if (timeSinceTimeout(sentTeamMessage.frameTime, addJitter(maxPlayerSendInterval, maxPlayerSendIntervalJitter),
                       "sentTime"))
    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;

  return messageDesirability.hasReasonToSend();
}

unsigned TeamMessageHandler2025::addJitter(unsigned time, unsigned jitterAmount)
{
  if ((time < jitterAmount) || (time > (0u - jitterAmount)))
    return time;
  else
    return (time - jitterAmount) + Random::uniformInt(jitterAmount * 2);
}


bool TeamMessageHandler2025::checkReadySetPlay()
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



bool TeamMessageHandler2025::checkBallMoved()
{
  // Is Our Local Ball Model Bad?
  if (theBallModel.seenPercentage <= minBallSeenPercentage &&
      (timeSinceTimeout(theBallModel.timeWhenLastSeen, ballLastSeenTimeout, "ballTimeWhenLastSeen") ||
       timeSinceTimeout(theBallModel.timeWhenDisappeared, ballDisappearedTimeout, "ballTimeWhenDisappeared")))
    return false;

  // OK our ball model is valid
  const Vector2f ballEndPosition = BallPhysics::getEndPosition(theBallModel.estimate.position,
                                                                theBallModel.estimate.velocity,
                                                                theBallSpecification.friction);  
  const Vector2f ballEndPositionOnField = theRobotPose.toFieldCoordinates(ballEndPosition);
  // const Vector2f ballOnField = theRobotPose.toFieldCoordinates(theBallModel.estimate.position);

  // if this robot is the ball player and time since last ball sent exceeds ball player timeout, send it
  if (theTeamBehaviorStatus.role.isBallPlayer())
  {
    if (timeSinceTimeout(lastTeamMessageTime, maxBallPlayerBallSendInterval, "lastTeamMessageTime"))
    {
      messageDesirability.reasonToSend = MessageDesirability::ballUpdateNeeded;
      return true;
    }
    else // not timed out, has it moved?
    {
      const Vector2f teamBallEndPositionOnField = BallPhysics::getEndPosition(
          theTeamBallModel.position, theTeamBallModel.velocity, theBallSpecification.friction);

      // we assume the team ball is the latest info all robots have, so compare the current end position to that
      // TODO: RV check - all robots have saame teamball, so if invalid they all have that too
      if (theTeamBallModel.isValid &&
          ((ballEndPositionOnField - teamBallEndPositionOnField).norm() > ballMovementThreshold))
      {
        TLOGV(mdLogger, "ballMoved: teamBall was {}, ourBallModel is {}", teamBallEndPositionOnField, ballEndPositionOnField);
        messageDesirability.reasonToSend = MessageDesirability::ballMoved;
        return true;
      }

    }
  }
  // not the ball player, but is this robot the closest to the ball end position based on assumed teammate positions?
  else if (timeSinceTimeout(addJitter(lastTeamMessageTime, intervalJitter), maxTeamBallSendInterval,
                            "lastTeamMessageTime"))
  {
//     float mySquaredNorm = ballEndPosition.squaredNorm(); // dist from this player to ball end position
//     bool playerIsClosest = true;
// 
//     for (const auto& location : theTeammatesLocationModel.locations)
//     {
//       if ((location.pose.translation - ballEndPositionOnField).squaredNorm() < mySquaredNorm)
//       {
//         playerIsClosest = false;
//         break;
//       }
//     }
// 
//     if (playerIsClosest)

    // Instead of checking if this robot is closest, if it has a decent ball model simply
    // send the message. We rely on the intervalJitter to generally ensure that all nearby
    // robots don't send at once and once one of them has sent, the others will wait again.
    // Also note that if a role change to ball player takes place, the new ball player will
    // send a message anyway, regardless of whether the ball moved or not.
    {
      messageDesirability.reasonToSend = MessageDesirability::ballUpdateNeeded;
      return true;
    }
  }

  // the ball could theoretically be much closer to this robot than the previous ball player
  // but we assume that in that case, this robot should become the ball player and
  // the case then gets dealt with above


//   float minDistanceBallSent = 0;
//   Vector2f lastBallGiven = Vector2f::Zero();
//   // Compare with what Teamdata says our ball model says
//   for (auto teammate : theTeamData.teammates)
//   {
//     if (teammate.number == theGameInfo.playerNumber)
//     {
//       lastBallGiven = teammate.theBallModel.estimate.position;
//       minDistanceBallSent = (theBallModel.estimate.position - lastBallGiven).norm();
//       break;
//     }
//   }
// 
//   // Do we agree with old Teamball if so send nothing keep as is (unless needed to keep team ball valid)
//   if (minDistanceBallSent < minDistanceLastTeamBall)
//   {
//     const int maxBallSendInterval = 4000; // FIXME - should be loadable param
//     const unsigned messageTime = lastTeamMessageTime;
// 
//     return timeSinceTimeout(messageTime, maxBallSendInterval, "messageTime");
//   }
//   else
//   { // Does any Teammates Predict the ball there?
//     for (auto teammate : theTeamData.teammates)
//     {
//       if ((teammate.theBallModel.estimate.position - theBallModel.estimate.position).norm() < minDistanceBallSent)
//       {
//         minDistanceBallSent = (teammate.theBallModel.estimate.position - theBallModel.estimate.position).norm();
//       }
//     }
//     if (minDistanceBallSent > minDistanceLastTeamBall)
//     {
//       messageDesirability.reasonToSend = MessageDesirability::ballMoved;
//       return true;
//     }
//   }

  return false;
}


bool TeamMessageHandler2025::checkRoleChanged()
{
  // for now, the most important thing is if this robot becomes ballPlayer
  if (prevSituationRole != theActiveTactic.situationRole)
  {
    switch (theActiveTactic.situationRole)
    {
    case PlayerRole::ballPlayer:
      messageDesirability.reasonToSend = MessageDesirability::roleChanged;
      break;
    default:
      break; // no reason to send
    }

    prevSituationRole = theActiveTactic.situationRole;
  }

  // TODO: for now we don't pay attention to change in formationRole because it
  // cannot yet change, but in future this may be important also
  static_cast<void>(prevFormationRole); // TODO: replace once really used

  return (messageDesirability.reasonToSend == MessageDesirability::roleChanged);
}


bool TeamMessageHandler2025::checkPlayerMoved()
{
  return false;
}


bool TeamMessageHandler2025::checkReturnFromPenalty()
{
  // note: wasPenalized is updated in checkStateAllowsSending

  if (wasPenalized && (theExtendedGameInfo.timeSinceLastPenaltyEnded >= returnFromPenaltyDelay))
  {
    messageDesirability.reasonToSend = MessageDesirability::returnedFromPenalty;
    wasPenalized = false;

    TLOGV(mdLogger, "checkReturnFromPenalty: wasPenalized before, timeSinceEnded {}",
          theExtendedGameInfo.timeSinceLastPenaltyEnded);

    return true;
  }

  return false;
}


bool TeamMessageHandler2025::checkUprightStatusChanged()
{
  if (isUpright != sentTeamMessage.teamMessage.isUpright)
  {
    messageDesirability.reasonToSend = MessageDesirability::uprightStatusChanged;
    return true;
  }

  return false;
}


bool TeamMessageHandler2025::checkStandbyReadyGesture()
{
  // have we receive a new signal at least specified time after previous signal
  unsigned frameTimeLastDetection = sentTeamMessage.teamMessage.standbyReadyGesture.frameTimeLastDetection;
  if (frameTimeLastDetection > 0)
    frameTimeLastDetection += 30000;

  if ((theGameInfo.state == STATE_STANDBY) && (theStandbyReadyGesture.frameTimeLastDetection > frameTimeLastDetection))
  {
    messageDesirability.reasonToSend = MessageDesirability::standbyReadyGestureDetected;
    return true;
  }

  return false;
}


bool TeamMessageHandler2025::checkSetPlayRefereeGesture()
{
  bool ret = false;

  // Are we in a set play with kicking team previously NONE and it has just changed
  if (theGameInfo.setPlay == SET_PLAY_NONE)
  {
    frameTimeLastKickingTeamUpdate = theFrameInfo.time;
    prevKickingTeam = KICKING_TEAM_NONE;
  }
  else // a set play is active
  {
    // has this robot detected a gesture since start of set play
    if (theSetPlayRefereeGesture.frameTimeLastDetection >= theExtendedGameInfo.timeWhenLastFreeKickStarted)
    {
      // during this set play, have we changed gestures since the last one we detected
      if ((theSetPlayRefereeGesture.lastDetection != SetPlayRefereeGesture::NONE) &&
          (theSetPlayRefereeGesture.lastDetection != sentTeamMessage.teamMessage.setPlayRefereeGesture.lastDetection) &&
          (theSetPlayRefereeGesture.frameTimeLastDetection >=
           sentTeamMessage.teamMessage.setPlayRefereeGesture.frameTimeLastDetection))
      {
        messageDesirability.reasonToSend = MessageDesirability::setPlayRefereeGestureDetected;
        ret = true;
      }
    }

    // // has this robot detected the gesture (and not sent it in a recent frame)?
    // if ((theSetPlayRefereeGesture.frameTimeLastDetection >= frameTimeLastKickingTeamUpdate) &&
    //     (theSetPlayRefereeGesture.lastDetection != sentTeamMessage.teamMessage.setPlayRefereeGesture.lastDetection) &&
    //     (theSetPlayRefereeGesture.lastDetection != SetPlayRefereeGesture::NONE))
    // {
    //   messageDesirability.reasonToSend = MessageDesirability::setPlayRefereeGestureDetected;
    //   ret = true;
    // }

    if (theGameInfo.kickingTeam != prevKickingTeam)
    {
      frameTimeLastKickingTeamUpdate = theFrameInfo.time;
      prevKickingTeam = theGameInfo.kickingTeam;
    }
  }

  return ret;
}


bool TeamMessageHandler2025::checkKickedSinceRestart()
{
  if (kickedSinceRestart && !prevFrameKickedSinceRestart)
  {
    messageDesirability.reasonToSend = MessageDesirability::kickedSinceRestart;
    return true;
  }
  
  return false;
}

bool TeamMessageHandler2025::timeSinceTimeout(unsigned timeToCheck, int interval, const char* var)
{
  // FIXME: log playback at least seems to have frameTimeWrapAround every 20-40secs - why?
  bool frameTimeWrapAround = (theFrameInfo.time < timeToCheck) && 
                            ((timeToCheck - theFrameInfo.time) > 1000);

  if (frameTimeWrapAround)
    TLOGW(mdLogger, "timeSinceTimeout timeToCheck {}, interval {} - frameTimeWrapAround frameTimeNow {} < {} by {} ms", 
          timeToCheck, interval, theFrameInfo.time, var ? var : "timeToCheck", (timeToCheck - theFrameInfo.time));

  return (frameTimeWrapAround || (theFrameInfo.getTimeSince(timeToCheck) >= interval));
}
