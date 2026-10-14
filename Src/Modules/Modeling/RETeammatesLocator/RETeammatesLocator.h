/**
 * @file RETeammatesLocator.h
 *
 * Locate teammates based on communicated information and expected positions
 * according to behaviour
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Modeling/RETeammatesLocationModel.h"

#include "Framework/Module.h"

#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/REActiveTactic.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/RETeamData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Infrastructure/GameState.h"

#include <vector>

MODULE(RETeammatesLocator, 
{, 
  REQUIRES(SetupPoses), 
  REQUIRES(FrameInfo), 
  REQUIRES(RobotPose), 
  REQUIRES(RETeamData),
  REQUIRES(GameState),

  USES(REActiveTactic), // must be uses because it is produced by TeamBehaviour which depends on TeammatesLocationModel
  USES(FieldBall), 
  USES(WalkingEngineOutput),

  REQUIRES(RETeammatesLocationModel), 
  PROVIDES(RETeammatesLocationModel),

  DEFINES_PARAMETERS(
  {,
    (float)(500.f)rotateToTargetDistance, ///< at this distance from the target, the robot is assumed to rotate to
                                          ///< face the ball or targetPose rotation
  }),
});

/**
 * @class RETeammatesLocator
 * 
 * locate teammates based on commmunicated information and behaviour based expected locations
 */
class RETeammatesLocator : public RETeammatesLocatorBase
{
public:
  RETeammatesLocator();

private:
  unsigned prevFrameTime = 0;;

  /**
   * Provides the world representation of teammate locations
   */
  void update(RETeammatesLocationModel& teammatesLocationModel) override;

  /**
   * get the estimated pose of a robot based on the target formation pose
   * and its previous actual or estimated position
   */
  Pose2f getEstimatedPose(const Pose2f& prevPose, const Pose2f& targetPose);

  void draw(const std::vector<RETeammatesLocationModel::Location>& locations);
};
