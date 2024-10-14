/**
 * @file ReceivedGameControlDataProvider.h
 *
 * This file implements a module that provides the data received from the GameController.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "Representations/Modeling/BallModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/GameInfo.h"

#include "Representations/Infrastructure/FrameInfo.h"

#include "Platform/SystemCall.h"
#include "Tools/Communication/UdpComm.h"
#include "Tools/Module/Module.h"

MODULE(ReceivedGameControlDataProvider,
{,
  USES(BallModel), // for alive message
  USES(FallDownState), // for alive message
  USES(RobotPose), // for alive message
  USES(GameInfo), // in order to get the playerMode
  REQUIRES(FrameInfo),

  REQUIRES(ReceivedGameControlData),
  PROVIDES(ReceivedGameControlData),

  DEFINES_PARAMETERS(
  {,
    (int)(2000) gameControllerTimeout, /**< Connected to GameController when packet was received within this period of time (in ms). */
    (int)(1000) aliveDelay, /**< Send an alive signal in this interval of ms. */
  }),
});

class ReceivedGameControlDataProvider : public ReceivedGameControlDataProviderBase
{
public:
  /** Initialize data and open socket. */
  ReceivedGameControlDataProvider();

private:
  UdpComm socket; /**< The socket to communicate with the GameController. */
  ReceivedGameControlData receivedGameControlData; 
  unsigned whenPacketWasReceived = 0; /**< When the last GameController packet was received - regardless of whether we copied it or not */
  unsigned whenReturnPacketWasSent = 0; /**< When the last return packet was sent to the GameController. */
  // RobotInfo::Mode mode = SystemCall::getMode() == SystemCall::physicalRobot ? GameInfo::unstiff : GameInfo::active; /**< The current robot mode. */

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theReceivedGameControlData The representation updated.
   */
  void update(ReceivedGameControlData& theReceivedGameControlData);

  /**
   * Receives a packet from the GameController.
   * Packets are only accepted when the team number is known (nonzero) and
   * they are addressed to this team.
   * @param updateFromReceivedData Whether the received data shall overwrite the local state.
   */
  void receive(bool updateFromReceivedData);

  /** Sends the alive message to the GameController. */
  bool sendAliveMessage();

  /** Resets the internal game state. */
  void resetReceivedGameControlData();
};
