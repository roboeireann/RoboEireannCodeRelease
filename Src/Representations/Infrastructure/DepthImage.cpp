/**
 * @file DepthImage.cpp
 *
 * This file implements a representation for a 2D depth image.
 *
 * @author Thomas Röfer
 */

#include "DepthImage.h"
#include "Debugging/DebugDrawings3D.h"
#include "Debugging/Modify.h"
#include "ImageProcessing/ColorModelConversions.h"
#include "Math/BHMath.h"
#include "Representations/Infrastructure/CameraInfo.h"

void DepthImage::draw() const
{
  int stepSize = 4;
  MODIFY("representation:DepthImage:stepSize", stepSize);
  DEBUG_DRAWING3D("representation:DepthImage", "camera")
  {
    if(Blackboard::getInstance().exists("CameraInfo"))
    {
      static ColorRGBA colors[256];
      static bool computed = false;
      if(!computed)
      {
        computed = true;
        for(int hue = 0; hue < 256; ++hue)
        {
          ColorRGBA& color = colors[hue];
          ColorModelConversions::fromHSIToRGB(static_cast<unsigned char>(hue), 255, 255, color.r, color.g, color.b);
        }
      }

      const CameraInfo& cameraInfo = static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]);
      for(unsigned y = 0; y < height; y += stepSize)
      {
        const unsigned short* depth = (*this)[y];
        const float y2 = cameraInfo.focalLengthHeightInv * (static_cast<float>(y) - cameraInfo.opticalCenter.y());
        for(unsigned x = 0; x < width; x += stepSize)
        {
          const float z = depth[x];
          if(z < 5900.f)
          {
            const float x2 = -cameraInfo.focalLengthInv * (x - cameraInfo.opticalCenter.x()) * z;
            const float y3 = -y2 * z;
            POINT3D("representation:DepthImage", z, x2, y3, 2, colors[mapToRange(static_cast<int>(y3), -1000, 2000, 0, 255)]);
          }
        }
      }
    }
  }
}
