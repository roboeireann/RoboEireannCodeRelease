/**
 * @file ReceivedGameControlDataProvider.cpp
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#include "ReceivedGameControlDataProvider.h"
#include "Tools/Settings.h"
#include "Tools/TextLogging.h"

#ifdef TARGET_ROBOT
#include <arpa/inet.h>
#include <netinet/in.h>
#endif
#include <cstring>

MAKE_MODULE(ReceivedGameControlDataProvider, communication);

DECL_TLOGGER(tlogger, "ReceivedGameControlDataProvider", TextLogging::INFO);

ReceivedGameControlDataProvider::ReceivedGameControlDataProvider()
{
  // Initialize GameController packet
  resetReceivedGameControlData();

  // Initialize socket
  VERIFY(socket.setBlocking(false));
  VERIFY(socket.bind("0.0.0.0", GAMECONTROLLER_DATA_PORT));
}

void ReceivedGameControlDataProvider::update(ReceivedGameControlData& theReceivedGameControlData)
{
  receive(theGameInfo.playerMode == GameInfo::active);

  // send an alive message back to the GC if it is active and we're due
  if (theFrameInfo.getTimeSince(whenPacketWasReceived) < gameControllerTimeout) 
  {
    if ((theFrameInfo.getTimeSince(whenReturnPacketWasSent) >= aliveDelay) && sendAliveMessage())
      whenReturnPacketWasSent = theFrameInfo.time;
  }
  // else // No GC messages received in timeout, so consider GC inactive
  //   theReceivedGameControlData.gcActive = false;

  // finally update the representation from our internal copy
  theReceivedGameControlData = receivedGameControlData;
}

void ReceivedGameControlDataProvider::receive(bool updateFromReceivedData)
{
  int size;
  RoboCup::RoboCupGameControlData buffer;
  unsigned fromIp;
  while((size = socket.read(reinterpret_cast<char*>(&buffer), sizeof(buffer), fromIp)) > 0)
  {
    // only consume valid GameControlData packets
    if (size == sizeof(buffer) && !std::memcmp(&buffer, GAMECONTROLLER_STRUCT_HEADER, 4) &&
        buffer.version == GAMECONTROLLER_STRUCT_VERSION &&
        (buffer.teams[0].teamNumber == Global::getSettings().teamNumber ||
         buffer.teams[1].teamNumber == Global::getSettings().teamNumber))
    {
      whenPacketWasReceived = Time::getCurrentSystemTime(); // more accurate than theFrameInfo.time?;

      if (updateFromReceivedData)
      {
        std::memcpy(static_cast<RoboCup::RoboCupGameControlData*>(&receivedGameControlData), &buffer, sizeof(buffer));
        receivedGameControlData.timeLastPacketReceived = whenPacketWasReceived;
        // receivedGameControlData.gcActive = true;
        // receivedGameControlData.ourTeamIndex = (receivedGameControlData.teams[0].teamNumber == Global::getSettings().teamNumber) ? 0 : 1;


        TLOGD(tlogger, "GC packet receive time {} ({:#032b})", whenPacketWasReceived, whenPacketWasReceived);
      }
#ifdef TARGET_ROBOT
      unsigned ip = htonl(fromIp);
      socket.setTarget(inet_ntoa(reinterpret_cast<in_addr&>(ip)), GAMECONTROLLER_RETURN_PORT);
#endif
    }
  }
}

bool ReceivedGameControlDataProvider::sendAliveMessage()
{
  RoboCup::RoboCupGameControlReturnData returnPacket;
  
  returnPacket.teamNum = static_cast<std::uint8_t>(Global::getSettings().teamNumber);
  returnPacket.playerNum = static_cast<std::uint8_t>(Global::getSettings().playerNumber);

  bool fallen = theFallDownState.state != FallDownState::upright &&
                theFallDownState.state != FallDownState::staggering &&
                theFallDownState.state != FallDownState::squatting;
  returnPacket.fallen = static_cast<std::uint8_t>(fallen);

  returnPacket.pose[0] = theRobotPose.translation.x();
  returnPacket.pose[1] = theRobotPose.translation.y();
  returnPacket.pose[2] = theRobotPose.rotation;

  returnPacket.ballAge = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) / 1000.f;

  returnPacket.ball[0] = theBallModel.estimate.position.x();
  returnPacket.ball[1] = theBallModel.estimate.position.y();

  return socket.write(reinterpret_cast<char*>(&returnPacket), sizeof(returnPacket));
}

void ReceivedGameControlDataProvider::resetReceivedGameControlData()
{
  // FIXME - try to eliminate code duplication with GameInfoProvider here
  
  std::memset(&static_cast<RoboCup::RoboCupGameControlData&>(receivedGameControlData), 0, sizeof(receivedGameControlData));
  receivedGameControlData.teams[0].teamNumber = static_cast<uint8_t>(Global::getSettings().teamNumber);
  receivedGameControlData.teams[0].fieldPlayerColor = static_cast<uint8_t>(Global::getSettings().fieldPlayerColor);
  receivedGameControlData.teams[0].goalkeeperColor = static_cast<uint8_t>(Global::getSettings().goalkeeperColor);
  receivedGameControlData.teams[0].goalkeeper = 1; // default to player 1 for now (TODO: maybe in the settings later)

  receivedGameControlData.teams[1].fieldPlayerColor = receivedGameControlData.teams[0].fieldPlayerColor ^ 1; // we don't know better
  receivedGameControlData.teams[1].goalkeeperColor = receivedGameControlData.teams[0].goalkeeperColor ^ 1; // we don't know better
  
  receivedGameControlData.playersPerTeam = static_cast<uint8_t>(Global::getSettings().playerNumber); // we don't know better
  receivedGameControlData.firstHalf = 1;

  receivedGameControlData.timeLastPacketReceived = 0;
  // receivedGameControlData.gcActive = false;
  // receivedGameControlData.ourTeamIndex = 0;
  // receivedGameControlData.playerIndex = Global::getSettings().playerNumber - Settings::lowestValidPlayerNumber;
}
