/**
 * @file ControlDeclarations.h
 *
 * declares the Control tasks that will be included in the behaviour
 * and possibly called from other tasks. The behaviour impl/definitions are found
 * in the .cpp files.
 *
 * This file should only be included inside the container/executor class for
 * the tasks. 
 */


// ============================================================================
// NoTeamCoordination
// ============================================================================

CORO_DECLARE(noTeamCoordinationTask);
HBS_REGISTER_TASK(noTeamCoordinationTask);

// ============================================================================
// TestGameStateTask
// ============================================================================

HSM_DECLARE(testGameStateTask);
HBS_REGISTER_TASK(testGameStateTask);

// ============================================================================
// TestKicksTask
// ============================================================================

HSM_DECLARE(testKicksTask);
HBS_REGISTER_TASK(testKicksTask);

CORO_DECLARE(testKicksSearchTask);
CORO_DECLARE(testKicksGoToBallAndKickTask);
