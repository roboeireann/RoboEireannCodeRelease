/**
 * @file FieldCoverage.cpp
 *
 * Implementation to send information about the field coverage.
 *
 * @author Nicole Schrader
 */

#include "FieldCoverage.h"

/**
 * Encodes the time difference into two bits.
 *
 * @param diff the time difference
 * @return two bits representing the time difference
 */
int encodeTimeDifference(unsigned diff);

/**
 * Decodes two bits into the time difference.
 *
 * @param code two bits representing the time difference
 * @return the time difference
 */
unsigned decodeTimeDifference(int code);

unsigned decodeTimeDifference(int code)
{
  if(code == 3)
    return 60000;
  if(code == 2)
    return 20000;
  if(code == 1)
    return 5000;
  return 1000;
}

int encodeTimeDifference(unsigned diff)
{
  if(diff > 20000)
    return 3;
  if(diff > 5000)
    return 2;
  if(diff > 1000)
    return 1;
  return 0;
}
