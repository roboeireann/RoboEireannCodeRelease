/**
 * @file Tools/Communication/TeamMessageChannel.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing [modifications]
 *
 * based on TeamHandler.cpp authored by
 * @author Thomas Röfer
 */

#include "TeamMessageChannel.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"

#include "TextLogging/TextLogging.h"


DECL_TLOGGER(tlogger, "TeamMessageChannel", TextLogging::INFO);


void TeamMessageChannel::startLocal(int port, unsigned localId)
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
  targetSet = true;
  // actualBroadcastAddr = group;
  TLOGI(tlogger, "startLocal: started local multicast on {} : {}", group, port);
}

void TeamMessageChannel::start(int port)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setLoopback(false);

  trySetBroadcastTarget();
}

bool TeamMessageChannel::send()
{
  if (!port)
    return false;

  if (!targetSet)
  {
    trySetBroadcastTarget();
    if (!targetSet)
      return false;
  }

  if (out.length == 0)
    TLOGW(tlogger, "send port {}: writing a zero length buffer", port);

  bool result = socket.write(reinterpret_cast<char *>(out.data), out.length);
  if (!result)
  {
    if (++consecutiveSendErrors < 20)
      TLOGW(tlogger, "send port {}: problem sending teammate packet: {} ({})", port, ErrorStr(errno).str(), errno);
  }
  else
  {
    consecutiveSendErrors = 0;
    TLOGD(tlogger, "send port {}: team message packet sent", port);
  }

  return result;
  
}

bool TeamMessageChannel::receive()
{
  if(!port)
    return false; // not started yet

  int size;
  unsigned remoteIp = 0;

  size = localId ? socket.readLocal(reinterpret_cast<char*>(in.data), sizeof(in.data) + 1)
                 : socket.read(reinterpret_cast<char*>(in.data), sizeof(in.data) + 1, remoteIp);

  if (size < 1 || static_cast<size_t>(size) > sizeof(in.data))
  {
    if ((size < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK))
      if (++consecutiveReceiveErrors < 20)
        TLOGW(tlogger, "receive: problem receiving teammate packet (size {}): {} ({})", size, ErrorStr(errno).str(), errno);
    
    return false;
  }
  else
  {
    in.length = static_cast<uint8_t>(size);

    consecutiveReceiveErrors = 0;
    TLOGD(tlogger, "receive: received a team message packet, size {}, timeReceived {}", size, Time::getCurrentSystemTime());
  }

  return true;
}

void TeamMessageChannel::trySetBroadcastTarget()
{
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();

  if (bcastAddr.empty())
  {
    if (++consecutiveStartErrors < 20)
      TLOGW(tlogger, "trySetBroadcastTarget: couldn't get broadcast address");
    return;
  }

  if (!socket.setTarget(bcastAddr.c_str(), port))
  {
    if (++consecutiveStartErrors < 20)
      TLOGW(tlogger, "trySetBroadcastTarget: couldn't set broadcast address {} : {}, problem: {} ({})", 
            bcastAddr, port, ErrorStr(errno).str(), errno);
    return;
  }

  // Success path
  targetSet = true;
  consecutiveStartErrors = 0;
  TLOGI(tlogger, "trySetBroadcastTarget: started broadcast on {} : {}", bcastAddr, port);
}
