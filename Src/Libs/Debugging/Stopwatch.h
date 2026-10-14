/**
 * @file Stopwatch.h
 * The file declares the stopwatch macros.
 * @author <a href="mailto:juengel@informatik.hu-berlin.de">Matthias Jüngel</a>
 * @author Thomas Röfer
 * @author <a href="mailto:arneboe@tzi.de">Arne Böckmann</a>
 * @author Rudi Villing (modifications)
 */

#pragma once

#include "Debugging/TimingManager.h"
#include "Debugging/Debugging.h"

/** A stopwatch that measures the time an instance of it lives and plots it. */
class Stopwatch_
{
  const char* const name; /**< The name of the plot. */

public:
  /**
   * Start the stopwatch.
   * @param name The name of the plot.
   */
  Stopwatch_(const char *name)
      : name(name)
  {
    startTiming(name);
  }

  /** Stop the stopwatch.*/
  ~Stopwatch_() { stopTiming(name); }

  // "plot:stopwatch:" + name... so name at offset 15
  // always call from macro
  // Separating like this allows a timer that spans different scopes, which is necessary
  // to span different callbacks
  static void startTiming(const char* name) { Global::getTimingManager().startTiming(name + 15); }

  static void stopTiming(const char *name)
  {
    [[maybe_unused]] const unsigned time = Global::getTimingManager().stopTiming(name + 15);
    DEBUG_RESPONSE(name)
      OUTPUT(idPlot, bin, (name + 5) << static_cast<float>(time) * 0.001f);
  }
};

/**
 * Allows the measurement the execution time of the following block and plot the measurements.
 * @param name The name of the stopwatch.
 */
#define STOPWATCH(name) if (Stopwatch_ stopwatch_("plot:stopwatch:" name); true)
// stopwatch only goes out of scope at end of if-statement block

/// use this version without any following scope - it literally starts the timer
/// explicitly call STOPWATCH_STOP to stop it
#define STOPWATCH_START(name) Stopwatch_::startTiming("plot:stopwatch:" name)

#define STOPWATCH_STOP(name) Stopwatch_::stopTiming("plot:stopwatch:" name)
