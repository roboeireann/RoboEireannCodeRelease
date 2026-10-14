/**
 * @file PrepareEngine.cpp
 *
 * @author Rudi Villing
 * @author Philip Reichmann (original outline)
 */

#include "PrepareEngine.h"
#include "Representations/MotionControl/MotionInfo.h"

#include "Platform/SystemCall.h"

MAKE_MODULE(PrepareEngine);

PrepareEngine::PrepareEngine()
{
}

void PrepareEngine::update(PrepareGenerator& thePrepareGenerator)
{
  thePrepareGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    return std::make_unique<PreparePhase>(*this);
  };
}


// ==========================================================================


PreparePhase::PreparePhase(PrepareEngine &engine)
    : MotionPhase(MotionPhase::prepare),
      engine(engine)
{
  engine.theEnergySaving.shutDown();
}

void PreparePhase::update()
{
}

bool PreparePhase::isDone(const MotionRequest& motionRequest) const
{
  return motionRequest.motion != MotionRequest::prepare &&
         std::abs(engine.theInertialData.angle.x()) < engine.torsoAngleForLeaving &&
         std::abs(engine.theInertialData.angle.y()) < engine.torsoAngleForLeaving &&
         std::abs(engine.theInertialData.angle.z()) < engine.torsoAngleForLeaving &&
         std::abs(engine.theIMUValueState.gyroValues.mean.x()) < engine.gyroAnglesForLeaving &&
         std::abs(engine.theIMUValueState.gyroValues.mean.y()) < engine.gyroAnglesForLeaving &&
         std::abs(engine.theIMUValueState.gyroValues.mean.z()) < engine.gyroAnglesForLeaving;

  // TODO - is it possible that the above checks prevent us transitioning out of
  // prepare if the robot is lying down?
  // TODO - should we also check that we have fully stood up and that feet are (likely) on
  // the ground.
}

void PreparePhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f&, MotionInfo& motionInfo)
{
  // code from playDead - just keep up with the currently reported joint angles
  jointRequest.angles = engine.theJointAngles.angles;
  jointRequest.stiffnessData.stiffnesses.fill(0);
  motionInfo.isMotionStable = false; // TODO - check whether true or false is correct here
}

std::unique_ptr<MotionPhase> PreparePhase::createNextPhase(const MotionPhase& nextPhase) const
{
  if ((nextPhase.type != MotionPhase::playDead) && (SystemCall::getMode() == SystemCall::physicalRobot))
    SystemCall::say("Walk", true);
    
  return std::unique_ptr<MotionPhase>();
}