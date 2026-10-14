/**
 * @file OdometryDataPreviewProvider
 * This file implements the provider for the preview of the odometryData
 */

#pragma once

#include "Framework/Module.h"
#include "Representations/Configuration/MassCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/FootSupport.h"
#include "Representations/Sensing/InertialData.h"
#include "Representations/Sensing/JointPlay.h"
#include "Representations/Sensing/RobotModel.h"
#include "Representations/Sensing/TorsoMatrix.h"

MODULE(OdometryDataPreviewProvider,
{,
  USES(OdometryData),
  USES(MotionInfo),
  REQUIRES(FallDownState),
  REQUIRES(FootSupport),
  REQUIRES(FrameInfo),
  REQUIRES(InertialData),
  REQUIRES(JointPlay),
  REQUIRES(MassCalibration),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotModel),
  REQUIRES(TorsoMatrix),
  PROVIDES(OdometryDataPreview),
  REQUIRES(OdometryDataPreview),
  PROVIDES(OdometryTranslationRequest),
  LOADS_PARAMETERS(
  {,
    (float) useMeasuredSwingAfterStepStartedTime, /**< Use the measured swing foot and not the planned one when a new walk phase started for this time. */
    (unsigned int) usePlannedIsLeftPhaseAfterThisSwitchTime, /**< If last support foot switch took more than this time, use the planned left phase flag. */
    (Vector2f) odometryWalkScaling, /**< While walking scale the odometry with these factors. */
    (bool) filterSupportSwitch, /**< When switching the support foot, filter which sole to use as supporting one. */
    (bool) useAnalyticalSupportDetection, /**< If true, pick the support sole by lowest sole-edge height instead of FootSupport/MotionInfo. */
    (float) ambiguousSupportHeightThreshold, /**< [mm] If |minZ_left - minZ_right| < threshold, assume zero odometry this frame. */
  }),
});

class OdometryDataPreviewProvider : public OdometryDataPreviewProviderBase
{
private:
  /**
   * Updates the OdometryDataPreview representation.
   * @param odometryDataPreview The preview of the odometry
   */
  void update(OdometryDataPreview& odometryDataPreview) override;

  void update(OdometryTranslationRequest& theOdometryTranslationRequest) override { theOdometryTranslationRequest = internalOdometryTranslationRequest; };

  Pose2f getOdometryOffset(const RobotModel& theRobotModel, const Pose2f& lastOdometry, const bool overrideRotation, Pose2f& lastLeftSole, Pose2f& lastRightSole);

  /**
   * Decide which sole is the support sole based on the minimum height of its four edge corners (in ground frame).
   * @param robotModel        The robot model to use (measured or requested).
   * @param leftIsSupport     [out] True iff left sole is the support foot.
   * @param ambiguous         [out] True iff |minZ_left - minZ_right| < ambiguousSupportHeightThreshold.
   */
  void detectSupportSoleAnalytically(const RobotModel& robotModel, bool& leftIsSupport, bool& ambiguous) const;

  Pose2f lastSoleLeft; /**< Last frame left sole in last frame torso. */
  Pose2f lastSoleRight; /**< Last frame right sole in last frame torso. */

  Pose2f lastRequestedSoleLeft; /**< Last frame left sole in last frame torso. */
  Pose2f lastRequestedSoleRight; /**< Last frame right sole in last frame torso. */

  OdometryTranslationRequest internalOdometryTranslationRequest;
};
