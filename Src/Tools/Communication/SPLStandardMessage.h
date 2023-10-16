// WARNING WARNING WARNING
// This is not a standard SPL message any more and indeed there is no standard
// SPL message for 2023.
// This has been modified from the 2022 version to work for RoboCup 2023.
// We've adjusted the version to 23 to indicate this modification.
// Only "necessary" fields have ben retained - anything that could be sacrificed
// to save space, was sacrificed

#ifndef SPLSTANDARDMESSAGE_H
#define SPLSTANDARDMESSAGE_H

#include <stdint.h>

// #define SPL_STANDARD_MESSAGE_STRUCT_HEADER  "SPL "
#define SPL_STANDARD_MESSAGE_STRUCT_VERSION 23

/*
 * Minimal MTU a network can set is 576 byte.
 * We have to subtract the IP header of 60 bytes and the UDP data 8 bytes.
 * So we have 576 - 60 - 8 = 508 safe size. From this we have to subtract the prefix to the data - 34 bytes.
 * So we have in the end 508 - 34 = 474 bytes free payload.
 *
 * See also https://stackoverflow.com/a/23915324
 */
// 128 less 3 bytes for the numeric members in SPLStandardMessage
#define SPL_STANDARD_MESSAGE_DATA_SIZE      125

/*
 * Important remarks about units:
 *
 * For each parameter, the respective comments describe its unit.
 * The following units are used:
 *
 * - Distances:  Millimeters (mm)
 * - Angles:     Radian
 * - Time:       Seconds (s)
 */
struct SPLStandardMessage
{
  uint8_t playerNum;     // [MANDATORY FIELD] 1-7
  uint8_t teamNum;       // [MANDATORY FIELD] the number of the team (as provided by the organizers)
  uint8_t numOfDataBytes;

  // buffer for arbitrary data, teams do not need to send more than specified in numOfDataBytes
  uint8_t data[SPL_STANDARD_MESSAGE_DATA_SIZE];

#ifdef __cplusplus
  SPLStandardMessage() : playerNum(0), teamNum(0), numOfDataBytes(0) {}
#endif
};

#endif // SPLSTANDARDMESSAGE_H
