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
  std::string hostAddr = SystemCall::getHostAddr();

  // Default to a fixed multicast address from the "Organization-Local Scope" range
  // This address is specifically for internal network use and doesn't rely on
  // an active physical network interface.
  std::string group = "239.255.0.1"; // default for loopback use

  // check if there is a suitable network
  // NOTE: for loopback to work in the absence of any other network, you may
  // need to explicitly enable multicast on the loopback interface.
  // You can call Make/Linux/enableLoopbackMulticast to do this temporarily.

  // if (!hostAddr.empty() && (hostAddr.substr(0,3) != "127"))
    group = "239" + hostAddr.substr(hostAddr.find('.'));

  TLOGI(tlogger, "Default host address is '{}', so multicast group is '{}'", hostAddr, group);

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
  {
    if (++consecutiveStartErrors < 20)
      TLOGW(tlogger, "start: couldn't set broadcast address, subnet '{}', broadcast '{}', port {}", requestedSubnet,
            actualBroadcastAddr, port);
  }
  else
  {
    consecutiveStartErrors = 0;
    TLOGI(tlogger, "start: started broadcast on {} : {}", actualBroadcastAddr, port);
  }
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
      if (++consecutiveSendErrors < 20)
        TLOGW(tlogger, "send port {}: couldn't set broadcast address", port);
      return false;
    }
    else
      TLOGI(tlogger, "send port {}: set broadcast on {}", port, actualBroadcastAddr);
  }

  bool result = socket.write(reinterpret_cast<const char *>(outMessage.data), outMessage.size);
  if (!result)
  {
    // if (errno != prevErrno)
    if (++consecutiveSendErrors < 20)
    {
      TLOGW(tlogger, "send port {}: problem sending teammate packet: {} ({})", port, ErrorStr(errno).str(), errno);
      // prevErrno = errno;
    }
  }
  else
  {
    consecutiveSendErrors = 0;
    TLOGD(tlogger, "send port {}: team message packet sent", port);
  }

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
        if (++consecutiveReceiveErrors < 20)
          TLOGW(tlogger, "problem receiving teammate packet, {} ({})", ErrorStr(errno).str(), errno);

      inBuffer.pop_back();
    }
    else
    {      
      ASSERT(size <= static_cast<int>(sizeof(entry.data))); // should be impossible to fail

      consecutiveReceiveErrors = 0;
      entry.size = size;
      entry.timeReceived = Time::getCurrentSystemTime();
      TLOGD(tlogger, "received a team message packet, size {}, timeReceived {}", entry.size, entry.timeReceived);
      ANNOTATION("TeamMessageSocketHandler", "received a teammate packet");
    }
  }
  while(size > 0);
}
