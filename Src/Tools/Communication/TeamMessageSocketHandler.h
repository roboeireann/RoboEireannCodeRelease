/**
 * @file TeamMessageSocketHandler.h
 * 
 * Manage the socket interface for communicating team messages.
 * This class only deals with raw message bytes. It does not interpret them
 * in any way.
 * 
 * It is based on SPLMessageHandler from BH2021
 * 
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Communication/UdpComm.h"
#include "Tools/RingBuffer.h"



#define TEAM_MESSAGE_MAX_DATA_SIZE 128

struct TeamMessageBytes
{
  const unsigned capacity = TEAM_MESSAGE_MAX_DATA_SIZE;
  unsigned size = 0; // data size actually used
  uint8_t data[TEAM_MESSAGE_MAX_DATA_SIZE];
};

struct ReceivedTeamMessageBytes : public TeamMessageBytes
{
  unsigned timeReceived; // clock time (ms) at which it was received - Note: not frame time
};

using ReceivedTeamMessageBytesBuffer = RingBuffer<ReceivedTeamMessageBytes, 11>;


/**
 * @class SPLMessageHandler
 * A class for team communication by broadcasting.
 */
class TeamMessageSocketHandler
{
public:
  /**
   * Constructor.
   * @param in buffer for received team messages.
   */
  TeamMessageSocketHandler() {}

  /**
   * The method starts the actual communication for local communication.
   * @param port The UDP port this handler is listening to.
   * @param localId An identifier for a local robot
   */
  void startLocal(int port, unsigned localId);

  /**
   * The method starts the actual communication on the given port.
   * @param port The UDP port this handler is listening to.
   * @param subnet The subnet the handler is broadcasting to.
   */
  void start(int port, const std::string& subnet = "");

  /**
   * The method sends the outgoing message if possible.
   */
  bool send(const TeamMessageBytes& outMessage);

  /**
   * The method receives packets if available.
   */
  void receive(ReceivedTeamMessageBytesBuffer& inBuffer);

private:
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  unsigned localId = 0; /**< The id of a local team communication participant or 0 for normal udp communication. */
  std::string requestedSubnet;
  std::string actualBroadcastAddr;
  int prevErrno = 0; ///< used to prevent spamming the log files with repeated errors

  bool setBroadcastTarget();
};
