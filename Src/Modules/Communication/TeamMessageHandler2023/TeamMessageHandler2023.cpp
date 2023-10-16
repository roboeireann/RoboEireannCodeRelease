/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file TeamMessageHandler2023.cpp
 *
 * the module that provide manages sending and receiving of team messages
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Aidan Colgan
 * @author Rudi Villing
 */

#include "TeamMessageHandler2023.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Platform/Time.h"

#include "Tools/TextLogging.h"
#include "Tools/Debugging/Annotation.h"

#include <limits>

//#define SITTING_TEST
//#define SELF_TEST

#define USE_VARIABLE(var_) (void)var_

DECL_TLOGGER(tlogger, "TeamMessageHandler2023", TextLogging::WARNING);

MAKE_MODULE(TeamMessageHandler2023, communication);

struct TeamMessage
{};

void TeamMessageHandler2023::regTeamMessage()
{
  PUBLISH(regTeamMessage);
  const char* name = typeid(TeamMessage).name();
  TypeRegistry::addClass(name, nullptr);
#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) TypeRegistry::addAttribute(name, typeid(x).name(), "the" #x)

  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotStatus);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotPose);

  REGISTER_TEAM_MESSAGE_REPRESENTATION(FrameInfo);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(BallModelSimplified);
  // REGISTER_TEAM_MESSAGE_REPRESENTATION(ObstacleModel); // TODO: registered but not used
  REGISTER_TEAM_MESSAGE_REPRESENTATION(Whistle);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(BehaviorStatus);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(TeamBehaviorStatus);

}

TeamMessageHandler2023::TeamMessageHandler2023()
    // : messageDesirabilityHelper(theFrameInfo, theRawGameInfo, theOwnTeamInfo, theBallModel, theRobotInfo, theTeamData,
    //                             theTeamBehaviorStatus)
{
  File f("teamMessage.def", "r");
  ASSERT(f.exists());
  std::string source(f.getSize(), 0);
  f.read(source.data(), source.length());
  teamCommunicationTypeRegistry.addTypes(source);
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage");

#ifndef TARGET_ROBOT
  theSPLMessageHandler.startLocal(Global::getSettings().getPort(),
                                  static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  theSPLMessageHandler.start(Global::getSettings().getPort());
#endif
}

void TeamMessageHandler2023::update(BHumanMessageOutputGenerator& outputGenerator)
{
  DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataMin");
  DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataDesired");
  DECLARE_PLOT("module:TeamMessageHandler2023:SPLStdMsgDataActual");

  // messageDesirabilityHelper.updateParams(totalMessageBudget, maxSendInterval, sendIntervalReadySet, sendInterval,
  //                                        networkTimeout, minDistanceLastTeamBall);


  // FIXME: Not used for now but if needed, generateTCMPPluginClass needs updating
  // DEBUG_RESPONSE_ONCE("module:TeamMessageHandler2023:generateTCMPluginClass")
  //   teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanStandardMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  theRobotStatus.isUpright =
      (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering ||
       theFallDownState.state == FallDownState::squatting) &&
      (theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp &&
       theMotionRequest.motion != MotionRequest::getUp);

  if(theRobotStatus.isUpright)
    theRobotStatus.timeWhenLastUpright = theFrameInfo.time;

  outputGenerator.sendIfNeeded = [this, &outputGenerator]()
  {
    if (shouldSendThisFrame())
    {
      generateMessage(outputGenerator);
      writeMessage(outputGenerator, outTeamMessage);
      sendMessage(outputGenerator, outTeamMessage);
    }
  };
}


void TeamMessageHandler2023::generateMessage(BHumanMessageOutputGenerator& outputGenerator) const
{
  outputGenerator.theBSPLStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theBSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  outputGenerator.theBHumanStandardMessage.setMagicNumber(Global::getSettings().magicNumber);

  outputGenerator.theBHumanStandardMessage.timestamp = Time::getCurrentSystemTime();

  theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  // The other members of theRobotStatus are filled in the update method.
  theRobotStatus.sequenceNumbers.fill(-1);
  for(const Teammate& teammate : theTeamData.teammates)
    theRobotStatus.sequenceNumbers[teammate.number - Settings::lowestValidPlayerNumber] = teammate.sequenceNumber;
  theRobotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber] = outputGenerator.sentMessages % 15;

  outputGenerator.theBHumanStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theBHumanStandardMessage.compressedContainer,
                                        outputGenerator.theBHumanStandardMessage.timestamp, teamMessageType,
                                        !outputGenerator.sentMessages);
  outputGenerator.theBHumanStandardMessage.out = &stream;

  theTimeSyncInfo >> outputGenerator;
  theRobotStatus >> outputGenerator;

  if(sendMirroredRobotPose)
  {
    RobotPose theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    theMirroredRobotPose >> outputGenerator;
  }
  else
    theRobotPose >> outputGenerator;

  theFrameInfo >> outputGenerator;

  // SEND_PARTICLE(BallModel);
  BallModelSimplified ballModelSimplified;
  ballModelSimplified.fromBallModel(theBallModel);
  ballModelSimplified >> outputGenerator;

  /* SEND_PARTICLE(ObstacleModel); */
  theWhistle >> outputGenerator;
  theBehaviorStatus >> outputGenerator;
  theTeamBehaviorStatus >> outputGenerator;


  outputGenerator.theBHumanStandardMessage.out = nullptr;

  outputGenerator.theBSPLStandardMessage.numOfDataBytes =
    static_cast<uint8_t>(outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage());
}

void TeamMessageHandler2023::writeMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage& m) const
{
  // write payload data part
  outputGenerator.theBHumanStandardMessage.write(reinterpret_cast<void*>(m.data));

  const int sizeOfBHumanMessage = outputGenerator.theBHumanStandardMessage.sizeOfBHumanMessage();

  // update the size in our local copy
  outputGenerator.theBSPLStandardMessage.numOfDataBytes = static_cast<uint8_t>(sizeOfBHumanMessage);

  // write SPL standard message part from our local copy
  outputGenerator.theBSPLStandardMessage.write(reinterpret_cast<void*>(&m.playerNum));

  // PLOT("module:TeamMessageHandler2023:SPLStdMsgDataCore", offset);
  // PLOT("module:TeamMessageHandler2023:SPLStdMsgDataDesired", desiredTotalSize);
  PLOT("module:TeamMessageHandler2023:SPLStdMsgDataActual", outputGenerator.theBSPLStandardMessage.numOfDataBytes);

  const int sizeOfStandardPart = sizeof(RoboCup::SPLStandardMessage)-sizeof(m.data);

  TLOGV(tlogger, "SPL data standard={} customData={}, total={}", sizeOfStandardPart, sizeOfBHumanMessage,
    sizeOfStandardPart + sizeOfBHumanMessage);
}


void TeamMessageHandler2023::sendMessage(BHumanMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage& m)
{
  if (!theSPLMessageHandler.send(m))
    return;

  outputGenerator.sentMessages++;
  frameTimeLastSent = theFrameInfo.time;

  TLOGD(tlogger, "sent from {}, sentMessages = {}", theRobotInfo.number, outputGenerator.sentMessages);
  ANNOTATION("TeamMessage2023", "sendMessage sent");
}


void TeamMessageHandler2023::update(TeamData& teamData)
{
  theSPLMessageHandler.receive(inTeamMessages);

  // update timeSync with the latest GC times we have before processing received team messages
  theTimeSyncInfo.updateGCInfo(theRawGameInfo.packetNumber, theRawGameInfo.timeLastPacketReceived, theFrameInfo.time);

  // process each of the messages received
  while (!inTeamMessages.empty())
  {
    // if (saveForLogfile) // save the raw packets in TeamData so that they will be logged
    // {
    //   teamData.rawSPLStandardMessageBufferEntries.push_back(RawSPLStandardMessageBufferEntry());
    //   memcpy(teamData.rawSPLStandardMessageBufferEntries.back().data.data(), m, sizeof(SPLStandardMessageBufferEntry));
    // }

    const ReceivedSPLStandardMessage& inputMessage = inTeamMessages.front(); // first received message first
    inTeamMessages.pop_front();

    if (readSPLStandardMessage(inputMessage)) // extract from ReceivedSPLStandardMessage into ReceivedBHumanMessage
    {
      // at this point we have successfully decoded the input ReceivedSPLStandardMessage into
      // the ReceivedBHumanMessage structure (and verified that the time could be synchronised)

      TLOGV(tlogger, "readSPLStandardMessage was successful");

      // we've received a valid message
      teamData.receivedMessages++;

      parseMessageIntoBMate(getBMate(teamData));

      TLOGD(tlogger, "received from {}, receivedMessages = {}",
            receivedMessageContainer.theBSPLStandardMessage.playerNum, teamData.receivedMessages);
    }
    else
    {
      if (receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage ||
          receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch ||
          receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::timeSyncFailure)
        return;

      // the message had a parsing error => possible intruder on our comms port
      if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
        timeWhenLastMimimi = theFrameInfo.time;

      ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
    }
  }

  maintainBMateList(teamData);
}


void TeamMessageHandler2023::maintainBMateList(TeamData& teamData) const
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


    // Remove Penalised and timed-out robots
    teamData.numberOfActiveTeammates = 0;
    
    for (auto teammate = teamData.teammates.begin(); teammate != teamData.teammates.end(); )
    {
      // if  (theRobotInfo.number == 1 && theOwnTeamInfo.teamNumber == 2 && teammate->number == 5 && theOwnTeamInfo.players[teammate->number - 1].penalty)
      // {
      //   TLOGV(tlogger, "Have any Teammates been penalised Member: {}, Penalised: {}",teammate->number,theOwnTeamInfo.players[teammate->number - 1].penalty);
      // }
      if (theOwnTeamInfo.players[teammate->number - 1].penalty != PENALTY_NONE ||
          theFrameInfo.getTimeSince(teammate->timeWhenLastPacketReceived) > networkTimeout)
        teammate = teamData.teammates.erase(teammate);
      else
      {
        teamData.numberOfActiveTeammates++;
        ++teammate; // iterator next
      }
    }
  }
}

#define PARSING_ERROR(outputText)                                                                                      \
  {                                                                                                                    \
    OUTPUT_ERROR(outputText);                                                                                          \
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError;                                      \
    return false;                                                                                                      \
  }

bool TeamMessageHandler2023::readSPLStandardMessage(const ReceivedSPLStandardMessage& m)
{
  // if(!receivedMessageContainer.theBSPLStandardMessage.read(&m->message.header[0]))
  if(!receivedMessageContainer.theBSPLStandardMessage.read(&m.message.playerNum))
    PARSING_ERROR("readSPLStandardMessage:" " SPL standard part reading failed");

#ifndef SELF_TEST
  if(receivedMessageContainer.theBSPLStandardMessage.playerNum == theRobotInfo.number)
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage;
    return false;
  }
#endif // !SELF_TEST

  if(receivedMessageContainer.theBSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theBSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("readSPLStandardMessage: Invalid robot number received");

  if(receivedMessageContainer.theBSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("readSPLStandardMessage: Invalid team number received");

  // now read our custom data payload
  if(!receivedMessageContainer.theBHumanStandardMessage.read(m.message.data))
    PARSING_ERROR("readSPLStandardMessage:" " custom BHumandStandardMessage part reading failed");

  if (!receivedMessageContainer.theBHumanStandardMessage.checkMagicNumber(Global::getSettings().magicNumber))
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch;
    TLOGW(tlogger, "readSPLStandardMessage: magic number did not match");
    return false;
  }

  // try to synchronize time here. Note, because of handling it here we will NOT try to read
  // the TimeSyncInfo particle in parseMessageIntoBMate

  theTimeSyncInfo << receivedMessageContainer;
  if (!theTimeSyncInfo.getTeammateInfo(receivedMessageContainer.theBSPLStandardMessage.playerNum).isValid())
  {
    receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::timeSyncFailure;
    TLOGW(tlogger, "readSPLStandardMessage: could not sync time of received message");
    return false;
  }
  
  receivedMessageContainer.timestamp = m.timestamp;

  // ANNOTATION("TeamMessageHandler2023", "readSPLStandardMessage read OK");
  return true;
}

Teammate& TeamMessageHandler2023::getBMate(TeamData& teamData) const
{
  // find the teammate by playerNum in our existing list (from prev cycle)
  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theBSPLStandardMessage.playerNum)
      return teammate;

  // not found => this teammate has just come online or come back from penalty
  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}


void TeamMessageHandler2023::parseMessageIntoBMate(Teammate& teammate)
{
  teammate.number = receivedMessageContainer.theBSPLStandardMessage.playerNum;

  const TimeSyncInfo::TeammateInfo& teammateSync = theTimeSyncInfo.getTeammateInfo(teammate.number);

  teammate.timeWhenLastPacketSent = teammateSync.estimateLocalTime(receivedMessageContainer.theBHumanStandardMessage.timestamp);
  teammate.timeWhenLastPacketReceived = receivedMessageContainer.timestamp;


  CompressedTeamCommunicationIn stream(receivedMessageContainer.theBHumanStandardMessage.compressedContainer,
                                       teammate.timeWhenLastPacketSent, teamMessageType,
                                       [&teammateSync](unsigned u) { return teammateSync.estimateLocalTime(u); });
  receivedMessageContainer.theBHumanStandardMessage.in = &stream;

  RobotStatus robotStatus;
  robotStatus << receivedMessageContainer;

  teammate.isPenalized = robotStatus.isPenalized;
  teammate.isUpright = robotStatus.isUpright;
  teammate.sequenceNumber = robotStatus.sequenceNumbers[teammate.number - Settings::lowestValidPlayerNumber];
  teammate.returnSequenceNumber = robotStatus.sequenceNumbers[theRobotInfo.number - Settings::lowestValidPlayerNumber];

  teammate.theRobotPose << receivedMessageContainer;
  teammate.theFrameInfo << receivedMessageContainer;

  // RECEIVE_PARTICLE(BallModel);
  BallModelSimplified ballModelSimplified;
  ballModelSimplified << receivedMessageContainer;
  ballModelSimplified.toBallModel(teammate.theBallModel);
  
  /* RECEIVE_PARTICLE(ObstacleModel); */
  teammate.theWhistle << receivedMessageContainer;
  teammate.theBehaviorStatus << receivedMessageContainer;
  teammate.theTeamBehaviorStatus << receivedMessageContainer;

  receivedMessageContainer.theBHumanStandardMessage.in = nullptr;

  // ANNOTATION("TeamMessageHandler2023", "ParseMessageIntoBMate seems OK");
}

// ============================================================================
// ============================================================================
// Message Desirability functionality
// ============================================================================
// ============================================================================

bool TeamMessageHandler2023::shouldSendThisFrame()
{
  messageDesirability.shouldSend = false;
  messageDesirability.reasonToSend = MessageDesirability::noReasonToSend;
  messageDesirability.reasonNotToSend = MessageDesirability::noReasonNotToSend;

  // check reasons from highest priority to lowest priority
  // check highest priority reasons not to send first

  if (!checkMessageBudgetAllowsSending() || !checkStateAllowsSending() || !checkMinMessageIntervalAllowsSending())
  {
    TLOGV(tlogger, "updateMessageDesirability - should not send because {}",
          TypeRegistry::getEnumName(messageDesirability.reasonNotToSend));
    return false;
  }


  // check for messages which should be sent, regardless
  checkReturnFromPenalty();
  checkPingNeeded();

  // now deal with special handling in the ready and set state
  if (!messageDesirability.hasReasonToSend())
  {
    if (!checkReadySetPlay())
    {
      TLOGV(tlogger, "updateMessageDesirability - should not send because {}",
            TypeRegistry::getEnumName(messageDesirability.reasonNotToSend));
      return false;
    }
  }

  if (messageDesirability.hasReasonNotToSend() && !messageDesirability.hasReasonToSend())
  {
  }

  // now check for other reasons to send - stop at the first one that is a reason
  // to send (if any)

  if (messageDesirability.hasReasonToSend() || checkBallMoved() || checkRoleChanged())
  {
    messageDesirability.shouldSend = true;
    TLOGI(tlogger, "shouldSendThisFrame because {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend));
    ANNOTATION("MessageDesirability",
               fmt::format("Send message reason = {}", TypeRegistry::getEnumName(messageDesirability.reasonToSend)));
  }

  return messageDesirability.shouldSend;
}


bool TeamMessageHandler2023::checkStateAllowsSending()
{
#if defined(TARGET_ROBOT) && !defined(SITTING_TEST)
  bool playingDead = (theMotionRequest.motion == MotionRequest::playDead) || 
                     (theMotionInfo.executedPhase == MotionPhase::playDead);
#else
  bool playingDead = false;
#endif

  bool gameSituationAllows = (theRawGameInfo.state != STATE_INITIAL) && (theRawGameInfo.state != STATE_FINISHED) &&
                             (!theRobotInfo.isPenalized()) && (theRawGameInfo.gamePhase != GAME_PHASE_PENALTYSHOOT);

  if (playingDead || !gameSituationAllows)
    messageDesirability.reasonNotToSend = MessageDesirability::statePreventsSending;

  return messageDesirability.reasonNotToSend == MessageDesirability::noReasonNotToSend;
}


bool TeamMessageHandler2023::checkMessageBudgetAllowsSending()
{
  messageDesirability.messageBudgetThisHalf = theOwnTeamInfo.messageBudget;
  if (theRawGameInfo.firstHalf)
    messageDesirability.messageBudgetThisHalf -= totalMessageBudget / 2;

  if (messageDesirability.messageBudgetThisHalf <= 5)
    messageDesirability.reasonNotToSend = MessageDesirability::messageBudgetPreventsSending;

  return messageDesirability.reasonNotToSend == MessageDesirability::noReasonNotToSend;
}


bool TeamMessageHandler2023::checkMinMessageIntervalAllowsSending()
{
//   const int minSendInterval = int(sendInterval * 0.75f);
// 
//   int currentSendInterval = 20000; // ms - changed below
// 
//   // ball player and captain get to send more often
//   if (theTeamBehaviorStatus.role.playsTheBall() || (theTeamBehaviorStatus.teammateRoles.captain == theRobotInfo.number))
//     currentSendInterval = int(sendInterval * 0.75f);
//   else
//     currentSendInterval = int(sendInterval * 1.5f);
// 
// 
//   if (theOwnTeamInfo.messageBudget < 200)
//     currentSendInterval = int(currentSendInterval * 1.1f);
//   else if (theOwnTeamInfo.messageBudget < 100)
//     currentSendInterval = int(currentSendInterval * 1.25f);
//     
//     
//   if (theRawGameInfo.state == STATE_READY || theRawGameInfo.state == STATE_SET)
//     currentSendInterval = std::max(currentSendInterval, 15000);
// 
//   bool timeAllows = (theFrameInfo.getTimeSince(frameTimeLastSent) >= currentSendInterval) || frameTimeWrapAround;
//   bool minTimeAllows = (theFrameInfo.getTimeSince(frameTimeLastSent) >= minSendInterval) || frameTimeWrapAround;


  // FIXME: log playback at least seems to have frameTimeWrapAround every 20-40secs - why?
  bool frameTimeWrapAround = (theFrameInfo.time < frameTimeLastSent); // && ((frameTimeLastSent - theFrameInfo.time) > 1000);

  if (!frameTimeWrapAround && (theFrameInfo.getTimeSince(frameTimeLastSent) < minSendInterval))
    messageDesirability.reasonNotToSend = MessageDesirability::minIntervalPreventsSending;

  return messageDesirability.reasonNotToSend == MessageDesirability::noReasonNotToSend; // do we have a reason not to send?
}


bool TeamMessageHandler2023::checkPingNeeded()
{
  // FIXME: log playback at least seems to have frameTimeWrapAround every 20-40secs - why?
  bool frameTimeWrapAround = (theFrameInfo.time < frameTimeLastSent); // && ((frameTimeLastSent - theFrameInfo.time) > 1000);

  if (frameTimeWrapAround)
  {
    TLOGW(tlogger, "frameTimeWrapAround frameTimeNow {} < frameTimeLastSent {} by {} ms", 
          theFrameInfo.time, frameTimeLastSent, (frameTimeLastSent - theFrameInfo.time));

    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;
  }
  else if (theFrameInfo.getTimeSince(frameTimeLastSent) >= maxSendInterval)
    messageDesirability.reasonToSend = MessageDesirability::pingNeeded;

  return messageDesirability.hasReasonToSend();
}


bool TeamMessageHandler2023::checkReadySetPlay()
{
  bool frameTimeWrapAround = (theFrameInfo.time < frameTimeLastSent); // && ((frameTimeLastSent - theFrameInfo.time) > 1000);
  int intervalLastSent =
      frameTimeWrapAround ? std::numeric_limits<int>::max() : theFrameInfo.getTimeSince(frameTimeLastSent);

  if ((theGameInfo.state == STATE_READY) || (theGameInfo.state == STATE_SET))
  {
    // this function will only be called if we've already exceeded the minSendInterval in general,
    // so the following condition should only be true for one cycle (near the start of the ready state)
    if ((theGameInfo.state == STATE_READY) && (theExtendedGameInfo.timeSinceReadyStarted < (minSendInterval + 100)) &&
        (intervalLastSent > minSendInterval))
      messageDesirability.reasonToSend = MessageDesirability::readySetPingNeeded;
    else if (intervalLastSent > maxSendIntervalReadySet)
      messageDesirability.reasonToSend = MessageDesirability::readySetPingNeeded;
    else
      messageDesirability.reasonNotToSend = MessageDesirability::readySetConstrainsSending;
  }

  return messageDesirability.hasReasonToSend() && !messageDesirability.hasReasonNotToSend();
}


bool TeamMessageHandler2023::checkBallMoved()
{
  // Is Our Local Ball Model Bad?
  if (theBallModel.seenPercentage <= 20 && (theBallModel.timeWhenDisappeared - theBallModel.timeWhenLastSeen > 2000))
  {
    // if (theRobotInfo.number == 1 && theOwnTeamInfo.teamNumber == 2)
    // {
    //   TLOGI(tlogger, "Local ball model bad: seen percentage = {}, timeWhenLastSeen = {}",
    //   theBallModel.seenPercentage,theBallModel.timeWhenLastSeen);
    // }
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
    // if (theRobotInfo.number == 1 && theOwnTeamInfo.teamNumber == 2)
    // {
    //   TLOGI(tlogger, "We agree with TeamBallModel: Local: x = {}, y = {}, Team: x = {}, y = {} ", 
    //         theBallModel.estimate.position .x(), theBallModel.estimate.position.y(),
    //         theTeamBallModel.position.x(), theTeamBallModel.position.y());
    // }
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

bool TeamMessageHandler2023::checkRoleChanged()
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

bool TeamMessageHandler2023::checkReturnFromPenalty()
{
  bool returnFromPenalty = wasPenalized && !theRobotInfo.isPenalized();
  wasPenalized = theRobotInfo.isPenalized();

  if (returnFromPenalty)
  {
    messageDesirability.reasonToSend = MessageDesirability::returnedFromPenalty;

    TLOGV(tlogger, "checkReturnFromPenalty: wasPenalized before, unpenalized now");
    return true;
  }
  else
    return false;
}

