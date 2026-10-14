/**
 * @file CommonControl.h
 *
 * declares the Common Control tasks that will be included in the behaviour
 * and possibly called from other tasks. The behaviour impl/definitions are found
 * in the .cpp files.
 *
 * This file should only be included inside the container/executor class for
 * the tasks. 
 */

 HSM_DECLARE(handlePhysicalRobotTask);
 HBS_REGISTER_TASK(handlePhysicalRobotTask);

 HSM_DECLARE(handlePlayerStateTask);
 HBS_REGISTER_TASK(handlePlayerStateTask);