/**
 * @file TeamMessageSocketHandler.cpp
 * 
 * Manage the socket interface for communicating team messages.
 * This class only deals with raw message bytes. It does not interpret them
 * in any way.
 * 
 * It is based on SPLMessageHandler from BH2021
 * 
 * @author Rudi Villing
 */

#include "TeamMessageSocketHandler.h"

#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/Annotation.h"

#include "Tools/TextLogging.h"

DECL_TLOGGER(tlogger, "TeamMessageSocketHandler", TextLogging::WARNING);

void TeamMessageSocketHandler::startLocal(int port, unsigned localId)
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

  actualBroadcastAddr = group;

  TLOGI(tlogger, "started - local multicast on {} : {}", actualBroadcastAddr, port);
}

void TeamMessageSocketHandler::start(int port, const std::string& subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setLoopback(false);

  requestedSubnet = subnet;
  if (!setBroadcastTarget())
    TLOGW(tlogger, "start: couldn't set broadcast address, requested {}, actual {}", requestedSubnet, actualBroadcastAddr);
  else
    TLOGI(tlogger, "start: started broadcast on {} : {}", actualBroadcastAddr, port);
}

bool TeamMessageSocketHandler::setBroadcastTarget()
{
  if (actualBroadcastAddr.empty())
    actualBroadcastAddr = !requestedSubnet.empty() ? requestedSubnet : UdpComm::getWifiBroadcastAddress();

  if (actualBroadcastAddr.empty())
    return false;

  if (!socket.setTarget(actualBroadcastAddr.c_str(), port))
  {
    actualBroadcastAddr = ""; // so that we try again next time
    return false;
  }
  else
    return true;
}

bool TeamMessageSocketHandler::send(const TeamMessageBytes& outMessage)
{
  if (!port)
    return false;
  else if (actualBroadcastAddr.empty())
  {
    if (!setBroadcastTarget())
    {
      TLOGW(tlogger, "send port {}: couldn't set broadcast address", port);
      return false;
    }
    else
      TLOGI(tlogger, "send port {}: set broadcast on {}", port, actualBroadcastAddr);
  }

  bool result = socket.write(reinterpret_cast<const char *>(outMessage.data), outMessage.size);
  if (!result)
  {
    if (errno != prevErrno)
    {
      TLOGW(tlogger, "send port {}: problem sending teammate packet: {}", port, ErrorStr(errno).str());
      prevErrno = errno;
    }
  }
  else
    TLOGD(tlogger, "send port {}: team message packet sent", port);

  return result;
}

void TeamMessageSocketHandler::receive(ReceivedTeamMessageBytesBuffer& inBuffer)
{
  if(!port)
    return; // not started yet

  int size;
  unsigned remoteIp = 0;

  do
  {
    inBuffer.push_back();
    ReceivedTeamMessageBytes& entry = inBuffer.back();

    size = localId ? socket.readLocal(reinterpret_cast<char*>(entry.data), sizeof(entry.data))
                   : socket.read(reinterpret_cast<char*>(&entry.data), sizeof(entry.data), remoteIp);

    if (size <= 0)
    {
      if ((size < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
        TLOGW(tlogger, "problem receiving teammate packet, {}", ErrorStr(errno).str());

      inBuffer.pop_back();
    }
    else
    {
      ASSERT(size <= static_cast<int>(sizeof(entry.data))); // should be impossible to fail

      entry.size = size;
      entry.timeReceived = Time::getCurrentSystemTime();
      TLOGD(tlogger, "received a team message packet, size {}, timeReceived {}", entry.size, entry.timeReceived);
      ANNOTATION("TeamMessageSocketHandler", "received a teammate packet");
    }
  }
  while(size > 0);
}
