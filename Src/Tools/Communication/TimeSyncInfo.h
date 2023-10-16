/**
 * @file TimestampSyncInfo.h
 * 
 * Based on the general idea of using the Game Controller as a time
 * reference (B-Human 2022 code release) but simplified a little
 * 
 * Each time a new GC packet arrives at this robot (see updateGCInfo)
 * we note the sequence number and time and add these to a ring buffer 
 * (gameControllerInfos)
 * 
 * Each time a teammate packet arrives (see updateTeammateInfo),
 *  
 *
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Settings.h"

#include "fmt/ostream.h"



class TimeSyncInfo
{
public:
  struct GCInfo
  {
    uint8_t seqNumGC;
    unsigned timeReceivedGC; /// local time at which packet was received

    GCInfo(uint8_t seqNumGC, unsigned timeReceivedGC)
        : seqNumGC(seqNumGC), timeReceivedGC(timeReceivedGC)
    {
    }
  };

  struct TeammateInfo
  {
    uint8_t seqNumGC; // the GC packet on which the current timeOffset is based
    int timeOffset; // localTime + timeOffset = teammateTime ==> localTime = teammateTime - timeOffset
    unsigned timeUpdated = 0; // local time when timeoffset was updated

    bool isValid() const { return timeUpdated != 0; }
    void invalidate() { timeUpdated = 0; }

    /**
     * Estimate local time in this robot corresponding to teammateTime on the teammate robot
     * represented by this TeammateInfo
     */
    unsigned estimateLocalTime(unsigned teammateTime) const
    {
#ifdef TARGET_ROBOT
      // force est local time to be unsigned and >=0
      return teammateTime ? static_cast<unsigned>(std::max(0, static_cast<int>(teammateTime) - timeOffset)) : 0u;
#else
      return teammateTime;
#endif
    }

    void update(int newTimeOffset, unsigned newTime, uint8_t newSeqNumGC)
    {
      timeOffset = newTimeOffset;
      timeUpdated = newTime;
      seqNumGC = newSeqNumGC;
    }
  };


  TimeSyncInfo()  {}

  /**
   * update when a new GameControl packet arrives at this robot
   * 
   * Called every cycle (and checks if a new GC packet has arrived)
   * 
   * @param seqNumGC       the most recent GC packet number received by this robot
   * @param timeReceivedGC the local time at which the GC packet arrived
   * @param referenceTime  the reference time at which the update is being performed
   *                       (used for pruning old values)
   */
  void updateGCInfo(uint8_t seqNumGC, unsigned timeReceivedGC, unsigned referenceTime);

  /**
   * update when a new teammate packet arrives
   * 
   * Only called on cycles when a packet arrives - i.e. not every cycle
   * 
   * @param playerNumber the teammate's player number
   * @param timestamp    the teammate's message timestamp
   * @param seqNumGC     the game controller packet number last received by
   *                     the teammate
   * @param timeReceivedGC the teammate's timestamp when they last received the
   *                     GC packet
  */
  void updateTeammateInfo(int playerNumber, unsigned timestamp,uint8_t seqNumGC, unsigned timeReceivedGC);

  const TeammateInfo& getTeammateInfo(int playerNumber) const
  {
    ASSERT(Settings::lowestValidPlayerNumber <= playerNumber && playerNumber <= Settings::highestValidPlayerNumber);
    return teammateInfos[playerNumber - Settings::lowestValidPlayerNumber];
  }

  /**
   * convenience function to call estimateLocalTime on appropriate TeammateInfo object
   */
  unsigned estimateLocalTime(int teammatePlayerNumber, unsigned teammateTime) const
  {
    const TeammateInfo& teammateInfo = getTeammateInfo(teammatePlayerNumber);
    return teammateInfo.estimateLocalTime(teammateTime);
  }

  // utility functions

  /// get time relative to reference (this is >0 if time was later than reference)
  inline int getTimeRelative(unsigned time, unsigned reference) const { return static_cast<int>(time - reference); }

protected:
  static constexpr int GC_TIMEOUT = 10000;
  static constexpr int MAX_VALID_TIME_DIFF = 3000;

  TeammateInfo teammateInfos[Settings::numPlayerNumbers];
  RingBuffer<GCInfo, 10> gameControllerInfos;
  unsigned oldestTimeReceivedGC = 0;

  // unsigned getTimestampFor(uint8_t packetNumber);
};


template <>
class fmt::formatter<TimeSyncInfo::GCInfo> {
public:
  constexpr auto parse (format_parse_context& ctx) { return ctx.begin(); }
  template <typename Context>
  constexpr auto format (TimeSyncInfo::GCInfo const& info, Context& ctx) const {
      return format_to(ctx.out(), "{}", info.seqNumGC);
  }
};

