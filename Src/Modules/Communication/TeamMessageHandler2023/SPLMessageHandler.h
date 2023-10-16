/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file Tools/Communication/SPLMessageHandler.h
 * The file declares a class for the team communication between robots.
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing
 *
 * based on TeamHandler authored by
 * @author Thomas Röfer
 */

#pragma once

#include "Tools/Communication/UdpComm.h"
#include "Tools/Communication/RoboCupGameControlData.h"
// #include "Tools/Communication/SPLStandardMessageBuffer.h"
#include "Tools/RingBuffer.h"



struct ReceivedSPLStandardMessage
{
  unsigned timestamp; /**< The timestamp when the message was received. */
  RoboCup::SPLStandardMessage message; /**< The received message. */
};


/**
 * @class SPLMessageHandler
 * A class for team communication by broadcasting.
 */
class SPLMessageHandler
{
public:
  using Buffer = RingBuffer<ReceivedSPLStandardMessage, 9>;

  /**
   * Constructor.
   * @param in Incoming spl standard messages.
   * @param out Outgoing spl standard message.
   */
  SPLMessageHandler() {}

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
  void start(int port, const char* subnet = nullptr);

  /**
   * The method sends the outgoing message if possible.
   */
  bool send(RoboCup::SPLStandardMessage& out);

  /**
   * The method receives packets if available.
   */
  void receive(Buffer& in);

private:
  int port = 0; /**< The UDP port this handler is listening to. */
  UdpComm socket; /**< The socket used to communicate. */
  unsigned localId = 0; /**< The id of a local team communication participant or 0 for normal udp communication. */
};
