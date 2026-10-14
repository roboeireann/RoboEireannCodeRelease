/**
 * KeyStateEnhancer.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "KeyStateEnhancer.h"
#include "TextLogging/TextLogging.h"

MAKE_MODULE(KeyStateEnhancer);

DECL_TLOGGER(tlogger, "KeyStateEnhancer", TextLogging::INFO);

void KeyStateEnhancer::update(EnhancedKeyStates& enhancedKeyStates)
{
  enhancedKeyStates.getHitStreakFor = [&enhancedKeyStates, this](KeyStates::Key key, unsigned timeOut, unsigned allowedTimeBetweenHitStreak) -> unsigned
  {
    return getHitStreakOf(key, enhancedKeyStates.pressed[key], timeOut, allowedTimeBetweenHitStreak);
  };

  // clear lambda before resetting to eliminate stale captures etc.
  enhancedKeyStates.isPressedFor = FunctionImpl::Function<bool(KeyStates::Key, unsigned)>();
  enhancedKeyStates.isPressedFor = [&enhancedKeyStates, lastEnhancedKeyStates = enhancedKeyStates](KeyStates::Key key, unsigned duration) -> bool
  {
    return enhancedKeyStates.pressedDuration[key] >= duration && lastEnhancedKeyStates.pressedDuration[key] < duration;
  };

  unsigned prevHitStreak[KeyStates::numKey];
  bool pressedOrReleased[KeyStates::numKey] = {};

  FOREACH_ENUM(KeyStates::Key, i)
  {
    if (theKeyStates.pressed[i])
    {
      if (enhancedKeyStates.pressed[i] != theKeyStates.pressed[i] || hitStreakDurations[i].empty())
      {
        hitStreakDurations[i].push_front(Range<unsigned>(theFrameInfo.time));
        pressedOrReleased[i] = true;
      }
      else
        hitStreakDurations[i].front().add(theFrameInfo.time);

      enhancedKeyStates.pressedDuration[i] = hitStreakDurations[i].front().getSize(); // non-zero duration while pressed
      enhancedKeyStates.clickedDuration[i] = 0; // no click while pressed
    }
    else // not pressed
    {
      if (enhancedKeyStates.pressedDuration[i]) // was it just released now?
      {
        pressedOrReleased[i] = true;
        enhancedKeyStates.clickedDuration[i] = hitStreakDurations[i].front().getSize();
      }
      else
      {
        enhancedKeyStates.clickedDuration[i] = 0; // no click after the cycle where it was released
      }

      enhancedKeyStates.pressedDuration[i] = 0; // zero duration once release
    }

    prevHitStreak[i] = enhancedKeyStates.hitStreak[i];
    enhancedKeyStates.hitStreak[i] = getHitStreakOf(i, theKeyStates.pressed[i], releaseTimeOut, successiveTimeOut);
  }

  // Booster K1 specifics
  if ((pressedOrReleased[KeyStates::k1_f1]) || (pressedOrReleased[KeyStates::k1_stand]) ||
      (pressedOrReleased[KeyStates::k1_walk]))
  {
    auto keyStr = [&, enhancedKeyStates](KeyStates::Key k) -> std::string
    {
      if (pressedOrReleased[k])
        return theKeyStates.pressed[k] ? "PRESS" : fmt::format("REL({}ms)", enhancedKeyStates.clickedDuration[k]); //hitStreakDurations[k].front().getSize());
      if (theKeyStates.pressed[k])
        return fmt::format("HOLD({})", enhancedKeyStates.pressedDuration[k]);
      else
        return "idle";
      
    };

    TLOGI(tlogger, "pressed/released: F1={}, Stand={}, Walk={}", keyStr(KeyStates::k1_f1), keyStr(KeyStates::k1_stand),
          keyStr(KeyStates::k1_walk));

    if (tlogger.isEnabled(TextLogging::DEBUG))
    {
      TLOGD(tlogger, "hitStreakDurations [(T_now-timeSincePress ms, duration ms), ...]:");
      for (auto key : {KeyStates::k1_f1, KeyStates::k1_stand, KeyStates::k1_walk})
      {
        fmt::memory_buffer buf;
        const char *keyName = TypeRegistry::getEnumName(key);
        fmt::format_to(std::back_inserter(buf), "  {:8s}: [", keyName);

        bool first = true;
        for (const auto &r : hitStreakDurations[key])
        {
          const int ageSince = theFrameInfo.getTimeSince(r.min);
          if (ageSince > 10000)
            break; // ring buffer is newest-first, so everything beyond here is older

          if (!first)
            fmt::format_to(std::back_inserter(buf), ", ");
          first = false;

          fmt::format_to(std::back_inserter(buf), "(T-{}, {})", theFrameInfo.getTimeSince(r.min), r.getSize());
        }
        fmt::format_to(std::back_inserter(buf), "]");
        TLOGD(tlogger, "{}", fmt::string_view(buf.data(), buf.size()));
      }
    }
  }

  if ((enhancedKeyStates.hitStreak[KeyStates::k1_f1] != prevHitStreak[KeyStates::k1_f1]) ||
      (enhancedKeyStates.hitStreak[KeyStates::k1_stand] != prevHitStreak[KeyStates::k1_stand]) ||
      (enhancedKeyStates.hitStreak[KeyStates::k1_walk] != prevHitStreak[KeyStates::k1_walk]))
  {
    TLOGI(tlogger, "hitStreak change: F1={}, Stand={}, Walk={}", enhancedKeyStates.hitStreak[KeyStates::k1_f1],
          enhancedKeyStates.hitStreak[KeyStates::k1_stand], enhancedKeyStates.hitStreak[KeyStates::k1_walk]);
  }


  static_cast<KeyStates&>(enhancedKeyStates) = theKeyStates;
}

unsigned KeyStateEnhancer::getHitStreakOf(KeyStates::Key key, bool keyState, unsigned maxPressDuration, unsigned allowedTimeBetweenHits)
{
  const int timeBetweenHits = static_cast<int>(allowedTimeBetweenHits);

  // if the key is pressed the streak cannot be finished
  // if the hitStreakDurations buffer is empty, there cannot be a streak
  // if the interval from the most recent hit to currentFrame is less than or equal max allowed, the streak might not be over
  if (keyState || hitStreakDurations[key].empty() ||
      (theFrameInfo.getTimeSince(hitStreakDurations[key].front().max) <= timeBetweenHits))
    return 0;

  // if the interval from the most recent hit to prevFrame is more than max allowed, the streak should have been
  // announced in the prevFrame already and should be suppressed now
  if (Time::getTimeBetween(hitStreakDurations[key].front().max, theFrameInfoEx.prevFrameTime) > timeBetweenHits)
  {
    // if (Time::getTimeBetween(hitStreakDurations[key].front().max, theFrameInfoEx.prevFrameTime - 33) <= timeBetweenHits)
    //   TLOGD(tlogger, "timeSincePrevFrame={}, but timeSincePrevFrame-33 = {}, lessThanOrEqual to {}",
    //         Time::getTimeBetween(theFrameInfoEx.prevFrameTime, hitStreakDurations[key].front().max),
    //         Time::getTimeBetween(theFrameInfoEx.prevFrameTime + 33, hitStreakDurations[key].front().max),
    //         timeBetweenHits);
    return 0;
  }


  // count the hits in the streak
  unsigned hitStreak = 0;
  for (auto iter = hitStreakDurations[key].begin(); iter != hitStreakDurations[key].end(); ++iter)
  {
    if (iter->getSize() >= maxPressDuration) // press duration too long, streak ends
      break;
    else if ((iter != hitStreakDurations[key].begin()) &&
        (((iter - 1)->min - iter->max) > allowedTimeBetweenHits)) // hit interval too large, streak ends
      break;

    ++hitStreak; // only increment when this element is valid
  }

  return hitStreak;
}
