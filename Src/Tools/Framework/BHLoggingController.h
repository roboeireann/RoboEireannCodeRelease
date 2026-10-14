/**
 * @file BHLoggingController.h
 *
 * This file declares the class \c BHLoggingController.
 *
 * @author Arne Hasselbring
 * @author Rudi Villing (modifications)
 */

#include "Framework/Logger.h"
#include "Streaming/AutoStreamable.h"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct GameState;
struct RobotSafetyState;

class BHLoggingController : public LoggingController
{
public:
  /**
   * Constructor.
   * - Reads the file \c teamList.cfg that defines a mapping from team number to team name.
   * - Allocates a \c GameState in the \c Blackboard.
   */
  BHLoggingController();

private:
  /** A team number and the corresponding team name. */
  STREAMABLE(Team,
  {,
    (std::uint8_t)(0) number,
    (std::string) name,
  });

  /** The map from team numbers to team names. */
  STREAMABLE(TeamList,
  {,
    (std::vector<Team>) teams,
  });

  /// config params for the logging controller
  ENUM(LoggingCondition,
  {,
    undefined, // don't use this in configuration files
    softwareStart,
    safetyStateOperational,
    safetyStateSafetyTripped,
    playerStateUnstiff,
    playerStatePrepare,
    playerStateActive,
    stateInitial,
    stateReady,
    stateSet,
    statePlaying,
    stateFinished,
    penaltyShootoutInitial,
    penaltyShootoutShotSet,
    penaltyShootoutShotPlaying,
    penaltyShootoutShotFinished,
  });

  /// Note: it is up to the config file writer to ensure that the stop condition and start conditions do not overlap, i.e.
  /// that a startCondition cannot already be present when a stopCondition fires
  STREAMABLE(Params,
  {,
    (std::vector<LoggingCondition>)({stateReady}) startConditions, ///< any of the listed conditions will start logging
    (std::vector<LoggingCondition>)({stateFinished}) stopConditions, ///< any of the listed conditions will stop logging (after timeout)
    (int)(10000) stopConditionTimeout, ///< stop condition must be present for this time before logging stops
  });

  /**
   * Destructor.
   * - Frees the \c GameState in the \c Blackboard.
   */
  ~BHLoggingController() override;

  /**
   * Returns if the current game state is neither INITIAL nor FINISHED(*).
   * (*) See the implementation for an exception.
   * @param wasLogging Whether the logger was logging in the previous frame.
   * @return If the current game state is ...
   */
  bool shouldLog(bool wasLogging) const override;

  /**
   * Builds a log file description matching the current game state.
   * - If a GameController is running and the opponent team is known:
   *   <opponent team name> "_" ( "1stHalf" | "2ndHalf" | "ShootOut" )
   * - Otherwise: "Testing"
   * @return The description for the log file.
   */
  std::string getDescription() const override;

  std::unordered_map<std::uint8_t, std::string> teams; /**< The map from team numbers to team names for naming the log file after the opponent. */
  const GameState& gameState; /**< The game state in the current thread's blackboard. */
  const RobotSafetyState& robotSafetyState; ///< from the blackboard
  Params params;
  mutable bool isSoftwareStartup = true;
};
