/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file Tools/Communication/SPLMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing
 *
 * based on TeamHandler.cpp authored by
 * @author Thomas Röfer
 */

#include "SPLMessageHandler.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Annotation.h"

#include "Tools/TextLogging.h"

DECL_TLOGGER(tlogger, "SPLMessageHandler", TextLogging::INFO);

void SPLMessageHandler::startLocal(int port, unsigned localId)
{
  ASSERT(!this->port);
  this->port = port;
  this->localId = localId;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); //keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
  socket.setLoopback(true);
}

void SPLMessageHandler::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setLoopback(false);

  std::string bcastAddr = subnet ? std::string(subnet) : UdpComm::getWifiBroadcastAddress();
  if (!bcastAddr.empty())
    socket.setTarget(bcastAddr.c_str(), port);
}

bool SPLMessageHandler::send(RoboCup::SPLStandardMessage& out)
{
  if (!port)
    return false;

  socket.write(reinterpret_cast<char*>(&out), offsetof(RoboCup::SPLStandardMessage, data) + out.numOfDataBytes);

  // Plot usage of data buffer in percent:
  // const float usageInPercent = 100.f * out.numOfDataBytes / static_cast<float>(SPL_STANDARD_MESSAGE_DATA_SIZE);
  // PLOT("module:SPLMessageHandler:standardMessageDataBufferUsageInPercent", usageInPercent);

  return true;
}

void SPLMessageHandler::receive(Buffer& in)
{
  if (!port)
    return; // not started yet

  int size;
  unsigned remoteIp = 0;

  do
  {
    in.push_back();
    ReceivedSPLStandardMessage& entry = in.back();

    size = localId ? socket.readLocal(reinterpret_cast<char*>(&entry.message), sizeof(entry.message))
                   : socket.read(reinterpret_cast<char*>(&entry.message), sizeof(entry.message), remoteIp);
    if(size < static_cast<int>(offsetof(RoboCup::SPLStandardMessage, data)) || size > static_cast<int>(sizeof(RoboCup::SPLStandardMessage)))
    {
      if (size > 0)
        TLOGW(tlogger, "received an incomplete/invalid SPL (teammate) packet with {} bytes", size);
      in.pop_back();
    }
    else
    {
      entry.timestamp = Time::getCurrentSystemTime();
      TLOGD(tlogger, "received a complete SPL (teammate) packet");
      ANNOTATION("SPLMessageHandler", "received a complete SPL (teammate) packet");
    }
  }
  while(size > 0);
}
