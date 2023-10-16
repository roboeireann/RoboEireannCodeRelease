/////////////////////////////////////////
//        D E P R E C A T E D          //
/////////////////////////////////////////

/**
 * @file BHumanStandardMessage.h
 *
 * The file declares the B-Human standard message.
 *
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/InOut.h"
#include <cstdint>
#include <vector>

#define BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER  "RE  "
#define BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION 23      /**< This should be incremented with each change. */
#define BHUMAN_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS 6   /**< The maximum number of players per team. */

/** The definintion of an NTP message we send - in response to a previously received request. */
// STREAMABLE(BNTPMessage,
// {,
//   (uint32_t) requestOrigination,  /**<                        The timestamp of the generation of the request. */
//   (uint32_t) requestReceipt,      /**< [delta 0..-4095]       The timestamp of the receipt of the request. */
//   (uint8_t) receiver,             /**< [#_MAX_NUM_OF_PLAYERS] The robot to which this message should be sent. */
// });

STREAMABLE(BHumanStandardMessage,
{
  In* in = nullptr; /**< A stream where data can be read from the compressed part. */
  Out* out = nullptr; /**< A stream where data can be written to the compressed part. */

  /**
   * Returns the size of this struct when it is written.
   * @return The size of ...
   */
  int sizeOfBHumanMessage() const;

  /**
   * Converts this struct for communication usage.
   * @param data Pointer to dataspace,
   *        THIS SHOULD BE AT LEAST AS BIG AS this->sizeOfBHumanMessage()
   * -asserts: writing sizeOfBHumanMessage() bytes
   */
  void write(void* data) const;

  /**
   * Reads the message from data.
   * @param data The message.
   * @return Whether the header and the versions are convertible.
   */
  bool read(const void* data);

  void setMagicNumber(int magic);
  bool checkMagicNumber(int magic) const;


  /** Constructor. */
  BHumanStandardMessage();
  
  /***** separate header from streamable members, note comma at end *****/,

  // (char[4]) header,         /**< BHUMAN_STANDARD_MESSAGE_STRUCT_HEADER */
  // (uint8_t) version,        /**< BHUMAN_STANDARD_MESSAGE_STRUCT_VERSION */
  // (uint8_t)(0) magicNumber, /**< The magic number. */
  (uint8_t)(0) headerVersionMagic, /**< Combined bitfield for header 3 bits, version (3 lsbs of version), magic (2 lsbs of magic). */
  (unsigned)(0) timestamp,  /**< Timestamp when this message has been sent (relative to the clock frame of the sending robot). */

  // (bool)(false) requestsNTPMessage,       /**< Whether this robot requests NTP replies from the others. */
  // (std::vector<BNTPMessage>) ntpMessages, /**< The NTP replies of this robot to other robots. */

  (uint8_t)(0) seqNumGC, //< sequence number of "reference" game controller packet
  (unsigned)(0) timeReceivedGC, //< time at which the above GC packet was received

  (std::vector<uint8_t>) compressedContainer, /**< The container for the compressed data. */
});
