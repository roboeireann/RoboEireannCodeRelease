/**
 * File:   TimingManager.cpp
 * Author: arne
 *
 * Created on May 9, 2013, 1:37 PM
 */

#include "TimingManager.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"
#include "Streaming/Output.h"
#include "Streaming/MessageQueue.h"
#include <unordered_map>
#include <vector>

struct TimingManager::Pimpl
{
  struct Entry
  {
    unsigned long long startMicros = 0; ///< when the timer started, or 0 if it is not running
    unsigned cumulativeMicros = 0; ///< when a timer is stopped, the current timing is added to this value
    unsigned prevCumulativeMicros = 0; ///< needed when timers span calls to getData
    unsigned short id = 0; ///< Key: name of the stopwatch. Value: the id that is used when sending timing data over the network
  };

  /**
   * NOTE: the hash maps only work because the compiler uses a string table and
   *       allocates only one address for all const string literals with the same value.
   *       If this ever changes you have to replace the key with std::string
   */

  /**
   * Key: name of the timer
   * value: If timer has been started but not stopped, yet: the start time.
   *        Else: the time between start and stop.
   */
  std::unordered_map<const char*, Entry> timings;
  // std::unordered_map<const char*, unsigned long long> timing;
  // std::unordered_map<const char*, unsigned short> idTable; /**< Key: name of the stopwatch. Value: the id that is used when sending timing data over the network */
  unsigned currentThreadStartTime = 0; /**< Timestamp of the current thread iteration */
  unsigned frameNo = 0; /**<  Number of the current frame*/
  std::vector<const char*> watchNames; /**< Contains the names of the stopwatches */
  MessageQueue data; /**< Contains the timing data in streamable format in between frames */
  bool dataPrepared = false; /**< True if data hs already been prepared this frame */
  int watchNameIndex = 0; /**< Every frame a few watch names are transmitted. This is the index of the watchname that is to be transmitted next */
};

TimingManager::TimingManager() : prvt(new TimingManager::Pimpl)
{
  prvt->data.reserve(500000);
}

TimingManager::~TimingManager()
{
  delete prvt;
}

void TimingManager::startTiming(const char* identifier)
{
  auto timing = prvt->timings.find(identifier);
  if (timing == prvt->timings.end())
  {
    //create new entry
    Pimpl::Entry entry;
    entry.id = static_cast<unsigned short>(prvt->timings.size()); //NOTE: this assumes that an unsigned short will always be big big enough to count the timers...
    timing = prvt->timings.emplace(identifier, entry).first;
    prvt->watchNames.push_back(identifier);
  }
  prvt->dataPrepared = false;
  timing->second.startMicros = Time::getCurrentThreadTime();// - timing->second; // accumulate measurements
}

unsigned TimingManager::stopTiming(const char* identifier)
{
  const unsigned long long stopTime = Time::getCurrentThreadTime();
  auto timing = prvt->timings.find(identifier);
  if (timing == prvt->timings.end()) // programming error really
    return 999999;

  timing->second.cumulativeMicros += (stopTime - timing->second.startMicros);
  timing->second.startMicros = 0; // mark it stopped
  // const unsigned diff = unsigned(stopTime - timing->second);
  // timing->second = diff;
  return timing->second.cumulativeMicros;
}

void TimingManager::signalThreadStart()
{
  prvt->currentThreadStartTime = Time::getCurrentSystemTime();
  prvt->frameNo++;
  prvt->data.clear();
  prvt->dataPrepared = false;

  for (auto& [key, entry] : prvt->timings)
  {
    entry.startMicros = 0;
    entry.prevCumulativeMicros = entry.cumulativeMicros;
    entry.cumulativeMicros = 0;
  }
  // for(std::pair<const char* const, unsigned long long>& it : prvt->timing)
  //   it.second = 0;
}

MessageQueue& TimingManager::getData()
{
  if(!prvt->dataPrepared)
  {
    prepareData();
    prvt->dataPrepared = true;
  }
  return prvt->data;
}

void TimingManager::prepareData()
{
  /** Protocol:
   * unsigned short : number of stopwatch names (usually 3)
   * for each stopwatch name:
   *  unsigned short : id of the stopwatch
   *  string         : name of the stopwatch
   *
   * unsigned short : Total number of stopwatches
   * for each stopwatch:
   *  short : id of the stopwatch  (depending on where you start in the stream you might not know the name of this watch, yet)
   *  unsigned : time in microseconds
   *
   * unsigned : timestamp at which the last iteration started.
   * unsigned : frame number of the current frame
   */
  MessageQueue::OutBinary out = prvt->data.bin(idStopwatch);

  // every frame we send up to 3 watch names
  // Guard against fewer than 3 registered timers.
  const int numNames = static_cast<int>(prvt->watchNames.size());
  const int namesToSend = std::min(3, numNames);

  out << static_cast<unsigned short>(namesToSend); //number of names to follow
  for(int i = 0; i < namesToSend; ++i, prvt->watchNameIndex = (prvt->watchNameIndex + 1) % numNames)
  {
    const char* watchName = prvt->watchNames[prvt->watchNameIndex];
    out << prvt->timings[watchName].id << watchName;
  }

  // now write the data of all watches
  out << static_cast<unsigned short>(prvt->timings.size());
  // for(const std::pair<const char* const, unsigned long long>& it : prvt->timing)
  for(const auto& [key, entry] : prvt->timings)
  {
    out << entry.id;
    // output cumulativeMicros from the previous frame if the timer is currently running (startMicros != 0)
    out << (entry.startMicros ? entry.prevCumulativeMicros : entry.cumulativeMicros);
  }

  out << prvt->currentThreadStartTime;
  out << prvt->frameNo;
  if(out.failed())
    OUTPUT_WARNING("TimingManager: queue is full!!!");
}
