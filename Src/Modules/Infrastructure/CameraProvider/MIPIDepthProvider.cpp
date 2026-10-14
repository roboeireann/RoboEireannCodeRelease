/**
 * @file MIPIDepthProvider.cpp
 *
 * This file declares a module that grabs depth images from the MIPI camera sensor.
 *
 * @author Thomas Röfer
 */

#include "MIPIDepthProvider.h"
#include "Platform/Time.h"

MAKE_MODULE(MIPIDepthProvider);

MIPIDepthProvider::MIPIDepthProvider()
  : MIPIProvider(CameraInfo::depth, imageTopic, cameraInfoTopic, useSystemTimeFlag) {}

void MIPIDepthProvider::update(DepthImage& theDepthImage)
{
#ifdef TARGET_BOOSTER
  SYNC;
  theDepthImage.setResolution(cameraInfo.width, cameraInfo.height);
  std::memcpy(theDepthImage[0], imageBuffer.data(), imageBuffer.size());
  theDepthImage.timestamp = lastImageTimestamp;
#else
  theDepthImage.setResolution(cameraInfo.width, cameraInfo.height);
  theDepthImage.timestamp = Time::getCurrentSystemTime();
#endif
}

void MIPIDepthProvider::update(JPEGDepthImage& theJPEGDepthImage)
{
  theJPEGDepthImage.fromDepthImage(theDepthImage, jpegQuality, depthRange);
}

bool MIPIDepthProvider::isFrameDataComplete2() const
{
  return theDepthImage.timestamp != lastImageTimestamp;
}

void MIPIDepthProvider::waitForFrameData2()
{
  const bool noImageYet = !theDepthImage.timestamp;
  waitForFrameData(noImageYet ? maxWaitForImage : minWaitForImage);
}
