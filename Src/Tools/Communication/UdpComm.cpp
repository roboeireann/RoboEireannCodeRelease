/**
 * @file Platform/Common/UdpComm.cpp
 * Wrapper for an udp socket.
 * @author Armin
 */

#include "UdpComm.h"

// #include <iostream>
#ifdef WINDOWS
#  include <ws2tcpip.h>
#else
#  include <cerrno>
#  include <unistd.h>
#  include <fcntl.h>
#  include <sys/socket.h>
#  include <sys/ioctl.h>
#  include <arpa/inet.h>
#  include <cstring>
#  include <net/if.h>
#  include <ifaddrs.h>
#endif

#include "Platform/BHAssert.h"
#include "Tools/TextLogging.h"

DECL_TLOGGER(tlogger, "UdpComm", TextLogging::INFO);

UdpComm::UdpComm()
{
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  target = reinterpret_cast<sockaddr*>(new sockaddr_in);

  ASSERT(-1 != sock);
}

UdpComm::~UdpComm()
{
#ifdef WINDOWS
  closesocket(sock);
#else
  close(sock);
#endif

  delete reinterpret_cast<sockaddr_in*>(target);
}

bool UdpComm::resolve(const char* addrStr, int port, sockaddr_in* addr)
{
  memset(addr, 0, sizeof(sockaddr_in));
  addr->sin_family = AF_INET;
  addr->sin_port = htons(static_cast<unsigned short>(port));
  if(1 != inet_pton(AF_INET, addrStr, &(addr->sin_addr.s_addr)))
  {
    TLOGE(tlogger, "resolve: {} is not a valid dotted IPv4 address", addrStr);
    return false;
  }

  return true;
}

bool UdpComm::setTarget(const char* addrStr, int port)
{
  sockaddr_in* addr = reinterpret_cast<sockaddr_in*>(target);
  return resolve(addrStr, port, addr);
}

bool UdpComm::setBlocking(bool block)
{
#ifdef WINDOWS
  int yes = block ? 0 : 1;
  if(ioctlsocket(sock, FIONBIO, reinterpret_cast<u_long*>(&yes)))
    return false;
  else
    return true;
#else
  bool result(false);
  if(block)
  {
    if(-1 != fcntl(sock, F_SETFL, 0))
      result = true;
  }
  else
  {
    if(-1 != fcntl(sock, F_SETFL, O_NONBLOCK))
      result = true;
  }
  return result;
#endif
}

bool UdpComm::setTTL(const char ttl)
{
  if(setsockopt(sock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(unsigned char)) < 0)
  {
    TLOGE(tlogger, "setTTL: could not set TTL to {}", ttl);
    return false;
  }
  return true;
}

bool UdpComm::setLoopback(bool yesno)
{
  const char val = yesno ? 1 : 0;
  if(setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &val, sizeof(char)) < 0)
  {
    TLOGE(tlogger, "could not set ip_multicast_loop to {}", val);
    return false;
  }
  return true;
}

bool UdpComm::joinMulticast(const char* addrStr)
{
  TLOGD(tlogger, "joinMulticast: addr {}", addrStr);

  sockaddr_in group;
  if(!resolve(addrStr, 0, &group))
    return false;

  //join multicast group for every interface
  if(IN_MULTICAST(ntohl(group.sin_addr.s_addr)))
  {
#ifndef WINDOWS
    ip_mreq mreq;
    ifconf ifc;
    ifreq* item;
    char buf[1024];

    ifc.ifc_len = sizeof(buf);
    ifc.ifc_buf = buf;
    if(ioctl(sock, SIOCGIFCONF, &ifc) < 0)
    {
      TLOGE(tlogger, "joinMulticast: cannot get interface list");
      return false;
    }
    else
    {
      bool could_join(false);
      for(unsigned int i = 0; i < ifc.ifc_len / sizeof(ifreq); i++)
      {
        item = &ifc.ifc_req[i];
        mreq.imr_multiaddr = group.sin_addr;
        mreq.imr_interface = reinterpret_cast<sockaddr_in*>(&item->ifr_addr)->sin_addr;
        if(0 == setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, static_cast<void*>(&mreq), sizeof(mreq)))
          could_join = true;
      }
      if(!could_join)
      {
        TLOGE(tlogger, "joinMulticast: group failed for interface");
        return false;
      }
    }
#else
    char host[128];
    hostent* pHost;
    if(gethostname(host, sizeof(host)) < 0 || !(pHost = static_cast<hostent*>(gethostbyname(host))))
    {
      TLOGE(tlogger, "joinMulticast: cannot get interface list");
      return false;
    }

    ip_mreq mreq;
    bool couldJoin(false);
    for(int i = 0; pHost->h_addr_list[i]; i++)
    {
      mreq.imr_multiaddr = group.sin_addr;
      mreq.imr_interface = *(reinterpret_cast<in_addr*>(pHost->h_addr_list[i]));
      if(setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<char*>(&mreq), sizeof(mreq)) == 0)
        couldJoin = true;
    }
    if(!couldJoin)
    {
      TLOGE(tlogger, "joinMulticast: group failed for interface");
      return false;
    }
#endif
    return true;
  }
  else
    TLOGE(tlogger, "joinMulticast: not a multicast address {}", addrStr);
  return false;
}

bool UdpComm::setBroadcast(bool enable)
{
  int yes = enable ? 1 : 0;
  if(0 == setsockopt(sock, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&yes), sizeof(yes)))
    return true;
  else
  {
    TLOGE(tlogger, "setBroadcast: setsockopt failed {}", ErrorStr(errno).str());
    return false;
  }
}

bool UdpComm::setRcvBufSize(unsigned int rcvbuf)
{
  if(0 == setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<char*>(&rcvbuf), sizeof(rcvbuf)))
  {
    TLOGE(tlogger, "setRcvBufSize: setsockopt for SO_RCVBUF failed: {}", ErrorStr(errno).str());
    return false;
  }

  int result;
  socklen_t result_len = sizeof(result);
  if(0 == getsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<char*>(&result), &result_len))
  {
    TLOGI(tlogger, "setRcvBufSize: getsockopt receive buffer set to {} bytes", result);
    return true;
  }
  else
  {
    TLOGI(tlogger, "setRcvBufSize: could not get sockopt SO_RCVbUF");
    return false;
  }
}

bool UdpComm::bind(const char* addr_str, int port)
{
  static const int yes = 1;
  sockaddr_in addr;
  addr.sin_addr.s_addr = INADDR_ANY; //addr.sin_addr;
  addr.sin_port = htons(static_cast<unsigned short>(port));
  addr.sin_family = AF_INET;

#ifdef BHUMAN_USE_INET_ADDR
  addr.sin_addr.s_addr = inet_addr(addr_str);
#else
  if(inet_pton(AF_INET, addr_str, &(addr.sin_addr)) <= 0)
  {
    TLOGE(tlogger, "bind: failed, invalid address {}", addr_str);
    return false;
  }
#endif

#ifdef SO_REUSEADDR
  if(-1 == setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes)))
    TLOGE(tlogger, "bind: could not set SO_REUSEADDR");
#endif
#ifdef SO_REUSEPORT
  if(-1 == setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, reinterpret_cast<const char*>(&yes), sizeof(yes)))
    TLOGE(tlogger, "bind: could not set SO_REUSEPORT");
#endif
  if(-1 == ::bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(sockaddr_in)))
  {
    TLOGE(tlogger, "bind: failed: {}", ErrorStr(errno).str());
    return false;
  }

  return true;
}

int UdpComm::read(char* data, int len, unsigned int& ip)
{
  sockaddr_in senderAddr;
#ifdef WINDOWS
  int size = sizeof(senderAddr);
#else
  unsigned size = sizeof(senderAddr);
#endif
  const int result = static_cast<int>(::recvfrom(sock, data, len, 0, reinterpret_cast<sockaddr*>(&senderAddr), &size));
  if(result > 0)
    ip = ntohl(senderAddr.sin_addr.s_addr);
  return result;
}

int UdpComm::read(char* data, int len)
{
  return static_cast<int>(::recv(sock, data, len, 0));
}

int UdpComm::readLocal(char* data, int len)
{
  sockaddr_in senderAddr;
#ifdef WINDOWS
  int size = sizeof(senderAddr);
#else
  unsigned size = sizeof(senderAddr);
  bool found = false;
#endif
  int result = static_cast<int>(::recvfrom(sock, data, len, 0, reinterpret_cast<sockaddr*>(&senderAddr), &size));
  if(result <= 0)
    return result;
  else
  {
#ifndef WINDOWS
    ifaddrs* addrs, *ifac;

    if(getifaddrs(&addrs) < 0)
      return -1;

    for(ifac = addrs; !found && ifac != nullptr; ifac = ifac->ifa_next)
    {
      if(ifac->ifa_flags & IFF_MULTICAST
         && ifac->ifa_addr
         && ifac->ifa_addr->sa_family == AF_INET)
      {
        if((reinterpret_cast<sockaddr_in*>(ifac->ifa_addr))->sin_addr.s_addr == senderAddr.sin_addr.s_addr)
          found = true;
      }
    }

    freeifaddrs(addrs);

    // no, packet comes from the outside -> ignore
    return found ? result : -1;
#else
    return result;
#endif
  }
}

bool UdpComm::write(const char* data, const int len)
{
  return ::sendto(sock, data, len, 0, target, sizeof(sockaddr_in)) == len;
}

std::string UdpComm::getWifiBroadcastAddress()
{
  std::string wifiAddress("255.255.255.255");
#ifdef TARGET_ROBOT
  char addressBuffer[INET_ADDRSTRLEN];

  ifaddrs* ifAddrStruct = nullptr;
  ifaddrs* ifa = nullptr;

  //determine ip address
  VERIFY(!getifaddrs(&ifAddrStruct));
  for(ifa = ifAddrStruct; ifa != nullptr; ifa = ifa->ifa_next)
  {
    // manpage getifaddrs    // check it is IP4
    if(ifa->ifa_addr != nullptr && ifa->ifa_addr->sa_family == AF_INET)
    {
      std::string interfaceName(ifa->ifa_name);
      if(interfaceName.find("wlan") != std::string::npos)
      {
        in_addr_t mask = reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask)->sin_addr.s_addr;
        in_addr_t addr = reinterpret_cast<sockaddr_in*>(ifa->ifa_addr)->sin_addr.s_addr;
        in_addr_t bcastAddr = ~mask | addr;

        in_addr bcast_addr;
        bcast_addr.s_addr = bcastAddr;

        VERIFY(inet_ntop(AF_INET, &bcast_addr, addressBuffer, INET_ADDRSTRLEN) == addressBuffer);
        wifiAddress = std::string(&addressBuffer[0]);
        break;
      }
    }
  }
  freeifaddrs(ifAddrStruct);
#endif
  return wifiAddress;
}
