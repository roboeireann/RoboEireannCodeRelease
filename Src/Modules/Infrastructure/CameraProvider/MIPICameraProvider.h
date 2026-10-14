/**
 * @file MIPICameraProvider.h
 *
 * This file declares a module that grabs images from the MIPI camera sensor.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "MIPIProvider.h"
#include "Framework/Module.h"
#include "Representations/Infrastructure/CameraImage.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/JPEGImage.h"

MODULE(MIPICameraProvider,
{,
  REQUIRES(CameraImage),
  PROVIDES_WITHOUT_MODIFY(CameraImage),
  PROVIDES(FrameInfo),
  PROVIDES(CameraInfo),
  PROVIDES(CameraIntrinsics),
  PROVIDES_WITHOUT_MODIFY(JPEGImage),
  LOADS_PARAMETERS(
  {,
    (std::string) imageTopic, /**< The full DDS topic name of the image. */
    (std::string) cameraInfoTopic, /**< The full DDS topic name of the camera info for that image. */
    (int) maxWaitForImage, /**< Timeout in ms for waiting for new images after the camera was just set up. */
    (int) minWaitForImage, /**< Timeout in ms for waiting for new images when the camera was previously working. */
    (int) jpegQuality, /**< The quality of the JPEG compressing (0 = bad ... 100 = very good). */
    (bool) useSystemTimeFlag, /**< Use the system time for camera time stamp. */
  }),
});

class MIPICameraProvider : public MIPICameraProviderBase, public MIPIProvider
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraImage The representation updated.
   */
  void update(CameraImage& theCameraImage) override;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraInfo The representation updated.
   */
  void update(CameraInfo& theCameraInfo) override {SYNC; theCameraInfo = cameraInfo;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theCameraIntrinsics The representation updated.
   */
  void update(CameraIntrinsics& theCameraIntrinsics) override {SYNC; theCameraIntrinsics = cameraIntrinsics;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theFrameInfo The representation updated.
   */
  void update(FrameInfo& theFrameInfo) override {theFrameInfo.time = theCameraImage.timestamp;}

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theJPEGImage The representation updated.
   */
  void update(JPEGImage& theJPEGImage) override;

  /** The method reports whether a new image is available. */
  bool isFrameDataComplete2() const override;

  /** The method waits for a new image. */
  void waitForFrameData2() override;

public:
  MIPICameraProvider();
};
