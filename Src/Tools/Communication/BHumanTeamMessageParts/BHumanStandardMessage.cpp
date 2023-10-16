/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file BHumanStandardMessage.cpp
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BHumanStandardMessage.h"
#include "Platform/BHAssert.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"

#include "Tools/TextLogging.h"

#include <algorithm>


DECL_TLOGGER(tlogger, "BHumanStandardMessage", TextLogging::INFO);

template<typename T>
inline void writeVal(void*& data, T value)
{
  *reinterpret_cast<T*&>(data)++ = value;
}

template<typename T>
inline T readVal(const void*& data)
{
  return *reinterpret_cast<T*&>(data)++;
}

const uint8_t HEADER_PATTERN = 0b111;
const uint8_t HEADER_BITS  = 0b111;
const int HEADER_SHIFT = 5;
const uint8_t VERSION_BITS = 0b111;
const int VERSION_SHIFT = 2;
const uint8_t MAGIC_BITS = 0b11;

template <class T>
unsigned readBits(const T from, unsigned bits, unsigned shift)
{
  return (from >> shift) & static_cast<T>(bits);
}

template <class T>
bool equalBits(const T from, unsigned bits, unsigned shift, const T val)
{
  return readBits(from,bits,shift) == readBits(val,bits,0);
}


template <class T>
void writeBits(T& dest, unsigned bits, unsigned shift, const T val)
{
  dest = (dest & static_cast<T>(~(bits << shift))) | static_cast<T>((val & static_cast<T>(bits)) << shift);
}

// #define READ_BITS(_from,_bits,_shift) ((_from >>_shift) & _bits)
// #define WRITE_BITS(_dest,_bits,_shift,_val) _dest = (_dest & ~(_bits << _shift)) | ((_val & _bits) << _shift)

void BHumanStandardMessage::setMagicNumber(int magic)
{
  TLOGD(tlogger, "setMagicNumber({:#010b})", static_cast<uint8_t>(magic));
  writeBits(headerVersionMagic, MAGIC_BITS, 0, static_cast<uint8_t>(magic));
}

bool BHumanStandardMessage::checkMagicNumber(int magic) const
{
  return readBits(headerVersionMagic, MAGIC_BITS,0) == readBits(static_cast<uint8_t>(magic), MAGIC_BITS,0);
}

BHumanStandardMessage::BHumanStandardMessage() //:
  // version(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
{
  // const char* init = BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER;
  // for(unsigned int i = 0; i < sizeof(header); ++i)
  //   header[i] = init[i];

  // bits 7-5 is the header, all 1's
  // bits 4-2 is the 3 LSb's of the version
  // bits 1-0 is the 2 LSb's of the magic
  writeBits(headerVersionMagic,HEADER_BITS,HEADER_SHIFT,HEADER_PATTERN);
  writeBits(headerVersionMagic,VERSION_BITS,VERSION_SHIFT, static_cast<uint8_t>(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION));
}

int BHumanStandardMessage::sizeOfBHumanMessage() const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 23, "This method is not adjusted for the current message version");

  return 0
        //  + sizeof(header)
        //  + sizeof(version)
        //  + sizeof(magicNumber)
         + sizeof(headerVersionMagic)
         + sizeof(timestamp)
         + sizeof(seqNumGC)
         + sizeof(timeReceivedGC)
         + sizeof(uint8_t) // size of compressedContainer (8 bits) - ignore rest, requestsNTPMessage (1 bit), NTP reply bitset (6 bits)
        //  + static_cast<int>(ntpMessages.size()) * 5
         + static_cast<int>(compressedContainer.size());
}

void BHumanStandardMessage::write(void* data) const
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 23, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  // for(unsigned i = 0; i < sizeof(header); ++i)
  //   writeVal<char>(data, header[i]);
  // writeVal<uint8_t>(data, version);
  // writeVal<uint8_t>(data, magicNumber);

  TLOGD(tlogger, "write header={:#05b}, version={:#05b}, magic={:#04b}",
    readBits(headerVersionMagic,HEADER_BITS,HEADER_SHIFT),
    readBits(headerVersionMagic,VERSION_BITS,VERSION_SHIFT),
    readBits(headerVersionMagic,MAGIC_BITS,0));

  // note that the writeVal calls modify the ptr data, incrementing it as needed
  writeVal<uint8_t>(data, headerVersionMagic);
  writeVal<uint32_t>(data, timestamp);
  writeVal<uint8_t>(data, seqNumGC);
  writeVal<uint32_t>(data, timeReceivedGC);

  // strict dependency on 6 players per team due to the way bit shifts are used below
  // static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team.");
  // std::sort(const_cast<std::vector<BNTPMessage>&>(ntpMessages).begin(), const_cast<std::vector<BNTPMessage>&>(ntpMessages).end(), [&](const BNTPMessage& a, const BNTPMessage& b) {return a.receiver < b.receiver; });
  // uint16_t ntpReceivers = 0;
  // if(!ntpMessages.empty())
  // {
  //   auto ntpMessagesItr = ntpMessages.cbegin();
  //   for(unsigned int i = 0; i < BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, ntpReceivers <<= 1)
  //   {
  //     if(ntpMessagesItr == ntpMessages.cend())
  //       continue;
  //     else if(ntpMessagesItr->receiver == i + 1)
  //     {
  //       ntpReceivers |= 1;
  //       ++ntpMessagesItr;
  //       ASSERT(ntpMessagesItr == ntpMessages.cend() || ntpMessagesItr->receiver > i + 1);
  //     }
  //   }
  // }
  ASSERT(compressedContainer.size() < (1ull << 8));
  // writeVal<uint16_t>(data, static_cast<uint16_t>((compressedContainer.size() << 7) | (requestsNTPMessage ? (1u << 6) : 0u) | (ntpReceivers >> 1)));
  writeVal<uint8_t>(data, static_cast<uint8_t>(compressedContainer.size()));

  // for(const BNTPMessage& ntpMessage : ntpMessages)
  // {
  //   const uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
  //   writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00) << 20));
  //   writeVal<uint8_t>(data, requestReceiptDiffCutted & 0xFF);
  // }

  std::memcpy(data, compressedContainer.data(), compressedContainer.size());
  reinterpret_cast<char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());
}

bool BHumanStandardMessage::read(const void* data)
{
  static_assert(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION == 23, "This method is not adjusted for the current message version");

  const void* const begin = data; //just for length check

  // ntpMessages.clear();

  // for(unsigned i = 0; i < sizeof(header); ++i)
  //   if(header[i] != readVal<const char>(data))
  //     return false;

  // version = readVal<const uint8_t>(data);
  // if(version != BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)
  //   return false;

  // magicNumber = readVal<const uint8_t>(data);

  headerVersionMagic = readVal<const uint8_t>(data);

  if (!equalBits(headerVersionMagic,HEADER_BITS,HEADER_SHIFT, HEADER_PATTERN))
  {
    TLOGD(tlogger, "read header mismatch: header={:#05b}, version={:#05b}, magic={:#04b}",
      readBits(headerVersionMagic,HEADER_BITS,HEADER_SHIFT),
      readBits(headerVersionMagic,VERSION_BITS,VERSION_SHIFT),
      readBits(headerVersionMagic,MAGIC_BITS,0));
    return false; // illegal header
  }

  if (!equalBits(headerVersionMagic, VERSION_BITS,VERSION_SHIFT, static_cast<uint8_t>(BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION)))
  {
    TLOGD(tlogger, "read version mismatch");
    return false; // version mismatch
  }

  timestamp = readVal<const uint32_t>(data);
  seqNumGC = readVal<const uint8_t>(data);
  timeReceivedGC = readVal<const uint32_t>(data);


  // static_assert(BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 6, "This code only works for exactly six robots per team (but can be easily adjusted).");
  // const uint16_t ntpAndSizeContainer = readVal<const uint16_t>(data);
  // compressedContainer.resize(ntpAndSizeContainer >> 7);
  const uint8_t containerSize = readVal<const uint8_t>(data);
  compressedContainer.resize(containerSize);

//   requestsNTPMessage = (ntpAndSizeContainer & (1u << 6)) != 0;
//   uint16_t runner = 1u << 6;
//   for(uint8_t i = 1; runner != 0; ++i)
//   {
//     if(ntpAndSizeContainer & (runner >>= 1))
//     {
//       ntpMessages.emplace_back();
//       BNTPMessage& message = ntpMessages.back();
//       message.receiver = i;
// 
//       const uint32_t timeStruct32 = readVal<const uint32_t>(data);
//       const uint8_t timeStruct8 = readVal<const uint8_t>(data);
// 
//       message.requestOrigination = timeStruct32 & 0xFFFFFFF;
//       message.requestReceipt = timestamp - (((timeStruct32 >> 20) & 0xF00) | static_cast<uint32_t>(timeStruct8));
//     }
//   }

  std::memcpy(compressedContainer.data(), data, compressedContainer.size());
  reinterpret_cast<const char*&>(data) += compressedContainer.size();

  ASSERT((reinterpret_cast<const char*>(data) - reinterpret_cast<const char* const>(begin)) == sizeOfBHumanMessage());

  TLOGD(tlogger, "read ok");
  return true;
}
