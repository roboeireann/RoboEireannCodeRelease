/**
 * @file TimeSyncInfo.cpp
 * 
 * Based on the general idea of using the Game Controller as a time
 * reference suggested by the B-Human 2022 code release but aiming to be a 
 * little simpler in implementation
 *
 * @author Rudi Villing
 */


#include "TimeSyncInfo.h"

#include "Platform/Time.h"
#include "Tools/TextLogging.h"


DECL_TLOGGER(tlogger, "TimeSyncInfo", TextLogging::DEBUG);


void TimeSyncInfo::updateGCInfo(uint8_t seqNumGC, unsigned timeReceivedGC, unsigned referenceTime)
{
  // ignore duplicate packets (or when no new packet has been received)
  if (!gameControllerInfos.empty() && ((gameControllerInfos.front().seqNumGC == seqNumGC) ||
                                       (gameControllerInfos.front().timeReceivedGC == timeReceivedGC)))
    return;

  // delete any timestamps that are now too old to be useful
  if (oldestTimeReceivedGC && (Time::getTimeSince(oldestTimeReceivedGC, referenceTime) > GC_TIMEOUT))
  {
    while (!gameControllerInfos.empty() &&
           (Time::getTimeSince(gameControllerInfos.back().timeReceivedGC, referenceTime) > GC_TIMEOUT))
      gameControllerInfos.pop_back();

    oldestTimeReceivedGC = gameControllerInfos.empty() ? 0 : gameControllerInfos.back().timeReceivedGC;
  }

  // add in the new GC packet info
  gameControllerInfos.push_front(GCInfo(seqNumGC, timeReceivedGC));

  // if (tlogger.isEnabled(TextLogging::VERBOSE))
  unsigned timeNow = Time::getCurrentSystemTime();
  TLOGV(tlogger, "updateGCInfo: time {} ({:#032b})", timeNow, timeNow);

  TLOGV(tlogger, "updateGCInfo: gameControllerInfos = {}", fmt::join(gameControllerInfos.begin(), gameControllerInfos.end(), ","));
}


void TimeSyncInfo::updateTeammateInfo(int teammatePlayerNumber, unsigned teammateTimestamp, uint8_t teammateSeqNumGC,
                                      unsigned teammateTimeReceivedGC)
{
  const int teammateIndex = teammatePlayerNumber - Settings::lowestValidPlayerNumber;
  TeammateInfo& teammateInfo = teammateInfos[teammateIndex];

  // General algorithm:
  // if the teammateInfo is valid and the GC reference is unchanged or cannot be found
  //   validate the teammateInfo
  // else update the GC reference

  // find corresponding seqNumGC in gameControllerInfos so that we can check
  // when the local robot received the same GC packet
  unsigned localTimeReceivedGC = 0;
  for (const auto &gameControllerInfo : gameControllerInfos)
  {
    if (gameControllerInfo.seqNumGC == teammateSeqNumGC)
    {
      localTimeReceivedGC = gameControllerInfo.timeReceivedGC;
      break;
    }
  }

  // localTime + timeOffset = teammateTime 
  // => localTime = teammateTime - timeOffset
  // => timeOffset = teammateTime - localTime 

  unsigned timeNow = Time::getCurrentSystemTime();

  // do we need to invalidate the teammate?
  if (teammateInfo.isValid())
  {
    // still working from an old GC seqNum or no matching GC seqNum at this robot?
    if ((teammateInfo.seqNumGC == teammateSeqNumGC) || (!localTimeReceivedGC))
    {
      // compare estimated local time of teammate's message with the actual
      // time (i.e. timeNow) and check that they are within a margin of
      // each other
      int estLocalTimeTeammateTimestamp = getTimeRelative(teammateTimestamp, teammateInfo.timeOffset);
      int estLocalTimeDifference = getTimeRelative(estLocalTimeTeammateTimestamp, timeNow);

      if (abs(estLocalTimeDifference) > MAX_VALID_TIME_DIFF)
      {
        TLOGD(tlogger, "updateTeammateInfo: *** could not sync teammate {}, estLocalTimeDifference {}",
              teammatePlayerNumber, estLocalTimeDifference);

        teammateInfo.invalidate();
      }

      return;
    }
  }

  // if we get this far we need to update the teammateInfo
  
  if (localTimeReceivedGC) // double check that we have the required info to update
  {
    // we found a packet number and hence can update the offset
    int timeSyncOffset = getTimeRelative(teammateTimeReceivedGC, localTimeReceivedGC);

    teammateInfo.update(timeSyncOffset, timeNow, teammateSeqNumGC);
  }

  if (!teammateInfo.isValid())
    TLOGD(tlogger, "updateTeammateInfo: *** could not sync teammate {}, gameControllerInfos = {}", teammatePlayerNumber,
          fmt::join(gameControllerInfos.begin(), gameControllerInfos.end(), ","));
}