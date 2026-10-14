/**
 * @file OnceFlag.h
 *
 * Define a class for triggering just once (though it can be reset).
 * It is not designed for streaming.
 *
 * @author Rudi Villing
 */

#pragma once

/**
 * @brief A lightweight guard to ensure an action is performed at most once until reset.
 *
 * OnceFlag acts as a single-use latch. It is designed to cleanly wrap the pattern of 
 * checking a boolean, executing a block, and setting the boolean to true.
 *
 * ### Example Usage
 * @code
 * OnceFlag flag;
 *
 * // First check succeeds
 * if (flag.once()) {
 *     // This block runs exactly once
 *     initializeSystem();
 * }
 *
 * // Subsequent checks fail
 * if (flag.once()) {
 *     // This will not execute
 * }
 *
 * // Resetting allows the action to fire one more time
 * flag.reset();
 * if (flag.once()) {
 *     // This runs again
 * }
 * @endcode
 * 
 * @note This class is not thread-safe. If multiple threads access the same OnceFlag 
 * concurrently, external synchronization (like a mutex) or a `std::once_flag` is required.
 */
struct OnceFlag
{
  /** @brief Tracks whether the flag has been triggered since construction or the last reset. */
  bool triggered = false;

  /**
   * @brief Resets the flag to its un-triggered state.
   * 
   * Subsequent calls to once() will return true exactly once.
   */
  void reset() { triggered = false; }

  /**
   * @brief Evaluates the flag and marks it as triggered.
   * 
   * @return true The first time called after construction or after a call to reset().
   *         false On all subsequent calls until reset() is invoked.
   */
  bool once()
  {
    if (triggered)
      return false;
    triggered = true;
    return true;
  }
};
