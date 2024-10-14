/**
 * @file TeammatesLocator.h
 *
 * Locate teammates based on communicated information and expected positions
 * according to behaviour
 *
 * @author Rudi Villing
 */

#pragma once

#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/BehaviorControl/ActiveTactic.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"

#include "Representations/Modeling/TeammatesLocationModel.h"

#include "Tools/Module/Module.h"

#include <vector>

MODULE(TeammatesLocator, 
{, 
  REQUIRES(SetupPoses), 
  REQUIRES(FrameInfo), 
  REQUIRES(RobotPose), 
  REQUIRES(TeamData),
  REQUIRES(GameInfo),

  USES(ActiveTactic), // must be uses because it is produced by TeamBehaviour which depends on TeammatesLocationModel
  USES(FieldBall), 
  USES(WalkingEngineOutput),

  REQUIRES(TeammatesLocationModel), 
  PROVIDES(TeammatesLocationModel),

  DEFINES_PARAMETERS(
  {,
    (float)(500.f)rotateToTargetDistance, ///< at this distance from the target, the robot is assumed to rotate to
                                          ///< face the ball or targetPose rotation
  }),
});

/**
 * @class TeammatesLocator
 * 
 * locate teammates based on commmunicated information and behaviour based expected locations
 */
class TeammatesLocator : public TeammatesLocatorBase
{
public:
  TeammatesLocator();

private:
  unsigned prevFrameTime = 0;;

  /**
   * Provides the world representation of teammate locations
   */
  void update(TeammatesLocationModel& teammatesLocationModel) override;

  /**
   * get the estimated pose of a robot based on the target formation pose
   * and its previous actual or estimated position
   */
  Pose2f getEstimatedPose(const Pose2f& prevPose, const Pose2f& targetPose);

  void draw(const std::vector<TeammatesLocationModel::Location>& locations);
};
