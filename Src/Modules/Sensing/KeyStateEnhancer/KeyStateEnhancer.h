/**
 * KeyStateEnhancer.h
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/FrameInfoEx.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Math/RingBuffer.h"
#include "Math/Range.h"
#include "Framework/Module.h"

#include <array>

MODULE(KeyStateEnhancer,
{,
  REQUIRES(KeyStates),
  REQUIRES(FrameInfo),
  REQUIRES(FrameInfoEx),

  PROVIDES(EnhancedKeyStates),
  DEFINES_PARAMETERS(
  {,
    // default values used for EnhancedKeyStates::hitStreak
    (unsigned)(1000u) releaseTimeOut,   ///< any long press longer than this is rejected in a hit streak
    (unsigned)(200u) successiveTimeOut, ///< allowed time between two hits to be considered as streak e.g. a double-click
  }),
});

class KeyStateEnhancer : public KeyStateEnhancerBase
{
  void update(EnhancedKeyStates& enhancedKeyStates) override;

  unsigned getHitStreakOf(KeyStates::Key key, bool keyState, unsigned maxPressDuration, unsigned allowedTimeBetweenHitStreak);

  using StreakDurationsBuffer = RingBuffer<Range<unsigned>, 10>;
  /// a ring buffer (to hold press durations within a hit streak) for each key
  std::array<StreakDurationsBuffer, KeyStates::numKey> hitStreakDurations;
};
