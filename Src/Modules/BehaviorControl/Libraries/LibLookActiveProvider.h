/**
 * @file LibLookActiveProvider.h
 * @author Andreas Stolpmann
 * @author Florian Scholz
 */

#include "Framework/Module.h"
#include "Debugging/Debugging.h"
#include "Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/AgentStates.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/Sensing/TorsoMatrix.h"
#include "Math/RingBufferWithSum.h"

MODULE(LibLookActiveProvider,
{,
  REQUIRES(AgentStates),
  REQUIRES(BallModel),
  REQUIRES(BallSpecification),
  USES(CameraCalibration),
  REQUIRES(CameraInfo),
  REQUIRES(FrameInfo),
  REQUIRES(HeadLimits),
  REQUIRES(GameState),
  REQUIRES(JointAngles),
  REQUIRES(JointRequest),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamBallModel),
  REQUIRES(MotionInfo),
  REQUIRES(TorsoMatrix),
  PROVIDES(LibLookActive),
  LOADS_PARAMETERS(
  {,
    (Angle) defaultSpeed, // deg/s
    (Angle) minSpeed,
    (Angle) maxSpeed,
    (Angle) largeDefaultPan,
    (Angle) smallDefaultPan,
    (Angle) minPanMoving,
    (Angle) defaultTilt,
    (Angle) minTilt,
    (int) ballPositionUnknownTimeout,
    (int) ballPositionUnknownTimeoutLowerCamera,
    (float) translationSpeedMinValue,
    (float) translationSpeedMaxValue,
    (Angle) rotationSpeedMinValue,
    (Angle) rotationSpeedMaxValue,
    (bool) lookAtCloseObstacleWhenFollowingBall,
    (float) maxObstacleDistanceToBeLookedAt,
    (int) maxObstacleAgeToBeLookedAt,
    (Angle) cameraChoiceHysteresis,
    (float) teamBallModelError, /**< estimation of the worst-case TeamBallModel error */
  }),
});

class LibLookActiveProvider : public LibLookActiveProviderBase
{
private:
  Vector2f theBallPositionRelative;
  Vector2f theBallSpeedRelative;

  bool teamBallIsUsed = false;

  float translationSpeedFactor = 1.f;
  float rotationSpeedFactor = 1.f;

  int basePanAngleIndex = 0;

  Angle calculatePan(const bool forceBall);
  Angle calculateTilt(const bool forceBall, const bool onlyOwnBall) const;
  Angle calculateSpeed(const bool forceBall, const float targetPan, const float slowdownFactor) const;

  bool shouldLookAtBall() const;

  bool panReached(const Angle pan) const;
  bool ballAnglesReachable() const;
  bool ballInSight() const;
  bool ballPositionUnknown(const bool onlyOwnBall) const;

  Angle clipPanToBall(const Angle pan) const;

  Angle clipPanToNearObstacle(const Angle pan) const;

  float getTranslationOffset(float x) const;

  RingBufferWithSum<float, 180> translationSpeedBuffer;
  RingBufferWithSum<Angle, 120> rotationSpeedBuffer;
  void calculateSpeedFactors();

  /**
  * Computes head pan, tilt and speed for active ball-aware gaze control.
  *
  * Falls back to TeamBallModel if own ball has timed out, unless onlyOwnBall is set.
  *
  * @param withBall       Force ball tracking, bypassing the internal heuristic.
  * @param ignoreBall     Suppress all ball tracking; pure environment scan only.
  *                       OVERRIDES @p withBall if both are true.
  * @param onlyOwnBall    Use only own ball Model; disables team ball fallback.
  * @param fixTilt        Lock tilt to a fixed default angle; skips dynamic tilt.
  * @param slowdownFactor [0-1] Lower values scales head speed down.
  *
  * @return HeadTarget with pan, tilt, speed and camera mode set.
  */
  HeadTarget calculateHeadTarget(const bool withBall, const bool ignoreBall, const bool withOwnBall, bool fixTilt, float slowdownFactor);
  void update(LibLookActive& libLookActive) override;
};
