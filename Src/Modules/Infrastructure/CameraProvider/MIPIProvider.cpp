/**
 * @file MIPIProvider.cpp
 *
 * This file implements a common base class for MIPI camera and depth providers.
 *
 * @author Thomas Röfer
 */

#include "MIPIProvider.h"
#include "Platform/Time.h"
#include "TextLogging/TextLogging.h"

thread_local MIPIProvider* MIPIProvider::theInstance = nullptr;

DECL_TLOGGER(tlogger, "MIPIProvider", TextLogging::INFO);

MIPIProvider::MIPIProvider(CameraInfo::Camera camera, [[maybe_unused]] const std::string &imageTopic,
                           [[maybe_unused]] const std::string &cameraInfoTopic, [[maybe_unused]] const bool useSystemTime)
    :
#ifdef TARGET_BOOSTER
      imageSubscriber(imageTopic, [this](const void *msg)
                      { imageHandler(*reinterpret_cast<const sensor_msgs::msg::Image *>(msg)); }),
      cameraInfoSubscriber(cameraInfoTopic, [this](const void *msg)
                           { cameraInfoHandler(*reinterpret_cast<const sensor_msgs::msg::CameraInfo *>(msg)); }),
      useSystemTime(useSystemTime),
#endif
      cameraInfo(camera)
{
  theInstance = this;
  VERIFY(readCameraIntrinsics());
  VERIFY(readCameraResolution());
  ASSERT(cameraResolutionRequest.resolutions[cameraInfo.camera] == CameraResolutionRequest::w544h448f30);
  cameraResolutionRequest.apply(cameraInfo.camera, cameraInfo);
  updateCameraInfo();

#ifdef TARGET_BOOSTER
  imageSubscriber.InitChannel();
  cameraInfoSubscriber.InitChannel();
#endif
}

MIPIProvider::~MIPIProvider()
{
#ifdef TARGET_BOOSTER
  imageSubscriber.CloseChannel();
  cameraInfoSubscriber.CloseChannel();
#endif
  theInstance = nullptr;
}

bool MIPIProvider::readCameraIntrinsics()
{
  InMapFile stream("cameraIntrinsics.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraIntrinsics;
  return exist;
}

bool MIPIProvider::readCameraResolution()
{
  InMapFile stream("cameraResolution.cfg");
  bool exist = stream.exists();
  if(exist)
    stream >> cameraResolutionRequest;
  return exist;
}

void MIPIProvider::updateCameraInfo()
{
  // set resolution
  const CameraIntrinsics::Camera& camera = cameraIntrinsics.cameras[cameraInfo.camera];

  // set opening angle
  cameraInfo.openingAngleWidth = camera.openingAngleWidth;
  cameraInfo.openingAngleHeight = camera.openingAngleHeight;

  // set optical center
  cameraInfo.opticalCenter = {camera.opticalCenter.x() * cameraInfo.width,
    camera.opticalCenter.y() * cameraInfo.height};

  // update focal length
  cameraInfo.updateFocalLength();
}

#ifdef TARGET_BOOSTER
void MIPIProvider::imageHandler(const sensor_msgs::msg::Image& image)
{
  SYNC;
  if(useSystemTime)
    lastImageTimestamp = Time::getRealSystemTime();
  else
    lastImageTimestamp = static_cast<unsigned>(image.header().stamp().sec() * 1000ll + image.header().stamp().nanosec() / 1000000 - Time::getSystemTimeBase());
  imageBuffer = image.data();

  TLOGV(tlogger, "imageHandler: new image received, lastImageTimestamp={}", lastImageTimestamp);

  imageSignal.post();
}

void MIPIProvider::cameraInfoHandler(const sensor_msgs::msg::CameraInfo& cameraInfo)
{
  SYNC;

  CameraIntrinsics::Camera& camera = cameraIntrinsics.cameras[this->cameraInfo.camera];
  camera.openingAngleWidth = 2.f * std::atan(static_cast<float>(cameraInfo.width() / cameraInfo.p()[0]) / 2.f);
  camera.openingAngleHeight = 2.f * std::atan(static_cast<float>(cameraInfo.height() / cameraInfo.p()[5]) / 2.f);
  camera.opticalCenter = {static_cast<float>(cameraInfo.p()[2]) / cameraInfo.width(),
    static_cast<float>(cameraInfo.p()[6]) / cameraInfo.height()};

  this->cameraInfo.width = cameraInfo.width();
  this->cameraInfo.height = cameraInfo.height();

  updateCameraInfo();

  cameraInfoSignal.post();
}
#endif

bool MIPIProvider::waitForFrameData(int timeout)
{
  const bool success = imageSignal.wait(timeout);
  while(success && imageSignal.tryWait());
  return success;
}

bool MIPIProvider::isFrameDataComplete()
{
#ifdef TARGET_BOOSTER
  if(theInstance)
    return theInstance->isFrameDataComplete2();
  else
#endif
    return true;
}

void MIPIProvider::waitForFrameData()
{
#ifdef TARGET_BOOSTER
  if(theInstance)
    theInstance->waitForFrameData2();
#endif
}
