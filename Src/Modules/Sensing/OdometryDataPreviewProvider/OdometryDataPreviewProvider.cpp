/**
 * @file OdometryDataPreviewProvider
 * This file implements the provider for the preview of the odometryData
 */

#include "OdometryDataPreviewProvider.h"
#include "Math/Rotation.h"

MAKE_MODULE(OdometryDataPreviewProvider);

void OdometryDataPreviewProvider::update(OdometryDataPreview& odometryDataPreview)
{
  static_cast<OdometryData&>(odometryDataPreview) = theOdometryData;

  const Pose2f odometryOffset = getOdometryOffset(theRobotModel, theOdometryData, true, lastSoleLeft, lastSoleRight);
  odometryDataPreview += odometryOffset;
  odometryDataPreview.odometryChange = odometryOffset;

  JointAngles executedRequest;
  FOREACH_ENUM(Joints::Joint, joint)
    executedRequest.angles[joint] = theJointPlay.jointState[joint].lastExecutedRequest;
  const RobotModel robotModelRequest(executedRequest, theRobotDimensions, theMassCalibration);
  const Pose2f odometryOffsetRequest = getOdometryOffset(robotModelRequest, internalOdometryTranslationRequest, false, lastRequestedSoleLeft, lastRequestedSoleRight);

  internalOdometryTranslationRequest += odometryOffsetRequest;
}

Pose2f OdometryDataPreviewProvider::getOdometryOffset(const RobotModel& theRobotModel, const Pose2f& lastOdometry, const bool overrideRotation, Pose2f& lastLeftSole, Pose2f& lastRightSole)
{
  Pose2f odometryOffset;

  // Sole poses in a gravityaligned frame
  const Pose3f gravityAlignedTorso(theTorsoMatrix.rotation);
  const Pose3f currentLeftSole3D = gravityAlignedTorso * theRobotModel.soleLeft;
  const Pose3f currentRightSole3D = gravityAlignedTorso * theRobotModel.soleRight;
  const Pose2f currentSoleLeft(currentLeftSole3D.rotation.getZAngle(), currentLeftSole3D.translation.head<2>());
  const Pose2f currentSoleRight(currentRightSole3D.rotation.getZAngle(), currentRightSole3D.translation.head<2>());

  if(!theMotionInfo.isMotion(MotionPhase::walk))
  {
    lastLeftSole = currentSoleLeft;
    lastRightSole = currentSoleRight;
  }
  else
  {
    bool leftIsSupport;
    bool ambiguous = false;

    if(useAnalyticalSupportDetection)
    {
      detectSupportSoleAnalytically(theRobotModel, leftIsSupport, ambiguous);
    }
    else
    {
      bool useIsLeftPhase = theMotionInfo.walkPhaseIsLeftPhase;
      if(filterSupportSwitch && theFrameInfo.getTimeSince(theMotionInfo.lastMotionPhaseStarted) < useMeasuredSwingAfterStepStartedTime
         && theFootSupport.lastSwitch < usePlannedIsLeftPhaseAfterThisSwitchTime)
        useIsLeftPhase = theFootSupport.support < 0.f;
      // walkPhaseIsLeftPhase == true means the LEFT foot is swinging, so the RIGHT foot is the support.
      leftIsSupport = !useIsLeftPhase;
    }

    // On ambiguous frames both soles are (nearly) flat on the ground
    if(!ambiguous)
    {
      const Pose2f& lastSupportSole = leftIsSupport ? lastLeftSole : lastRightSole;
      const Pose2f& currentSupportSole = leftIsSupport ? currentSoleLeft : currentSoleRight;

      odometryOffset = lastSupportSole * currentSupportSole.inverse();

      odometryOffset.translation.x() *= odometryWalkScaling.x();
      odometryOffset.translation.y() *= odometryWalkScaling.y();

      lastLeftSole = currentSoleLeft;
      lastRightSole = currentSoleRight;
    }
  }
  // Construct odometry update for this frame.
  if(theFallDownState.state == FallDownState::falling || theFallDownState.state == FallDownState::fallen)
    odometryOffset.rotation = 0_deg; // Postpone rotation change until being upright again to avoid huge variances in the self locator.
  else if(overrideRotation)
    odometryOffset.rotation = Angle::normalize(Rotation::Euler::getZAngle(theInertialData.orientation3D) - lastOdometry.rotation);
  return odometryOffset;
}

void OdometryDataPreviewProvider::detectSupportSoleAnalytically(const RobotModel& robotModel, bool& leftIsSupport, bool& ambiguous) const
{
  // Sole-edge corner offsets in sole-local frame. y-sign mirrors per foot.
  const float xF =  theRobotDimensions.soleToFrontEdgeLength;
  const float xB = -theRobotDimensions.soleToBackEdgeLength;
  const float yInL = -theRobotDimensions.soleToInnerEdgeLength;
  const float yOuL =  theRobotDimensions.soleToOuterEdgeLength;
  const float yInR =  theRobotDimensions.soleToInnerEdgeLength;
  const float yOuR = -theRobotDimensions.soleToOuterEdgeLength;

  auto minCornerZ = [&](const Pose3f& sole, float yInner, float yOuter) -> float
  {
    const Pose3f soleInGround = theTorsoMatrix * sole;
    const float z0 = (soleInGround * Vector3f(xF, yOuter, 0.f)).z();
    const float z1 = (soleInGround * Vector3f(xF, yInner, 0.f)).z();
    const float z2 = (soleInGround * Vector3f(xB, yOuter, 0.f)).z();
    const float z3 = (soleInGround * Vector3f(xB, yInner, 0.f)).z();
    return std::min(std::min(z0, z1), std::min(z2, z3));
  };

  const float minZLeft  = minCornerZ(robotModel.soleLeft,  yInL, yOuL);
  const float minZRight = minCornerZ(robotModel.soleRight, yInR, yOuR);

  ambiguous = std::abs(minZLeft - minZRight) < ambiguousSupportHeightThreshold;
  leftIsSupport = minZLeft < minZRight;
}
