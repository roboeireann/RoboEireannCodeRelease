/**
 * @file TypeRegistryExtensions.cpp
 *
 * Add/modify TypeRegistry based on robot specifics
 *
 * @author Rudi Villing
 */

#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"

#include "Framework/Settings.h"

#include "Streaming/FunctionList.h"

namespace
{

void extendTypeRegistry(const Settings &settings)
{
  if (settings.robotType != Settings::nao)
  {
    const char *_type = typeid(CameraInfo::Camera).name();
    TypeRegistry::addEnum(_type);
    REG_CONSTANT(upper);
    REG_CONSTANT(color = upper);
    REG_CONSTANT(lower);
    REG_CONSTANT(depth = lower);

    _type = typeid(HeadMotionRequest::CameraControlMode).name();
    TypeRegistry::addEnum(_type);
    REG_CONSTANT(autoCamera);
    REG_CONSTANT(lowerCamera);
    REG_CONSTANT(depthCamera = lowerCamera);
    REG_CONSTANT(upperCamera);
    REG_CONSTANT(colorCamera = upperCamera);
  }
}

void registerExtension()
{
  PUBLISH(registerExtension);
  TypeRegistry::extensionHook = &extendTypeRegistry;
}

}
