/**
 * @file DepthImage.h
 *
 * This file defines a representation for a 2D depth image.
 *
 * @author Thomas Röfer
 */

#pragma once

#include "ImageProcessing/Image.h"

STREAMABLE_WITH_BASE(DepthImage, Image<PixelTypes::DepthPixel>,
{
  /** Inherit constructors. */
  using Image<PixelTypes::DepthPixel>::Image;

  /** Draw this representation. */
  void draw() const,

  (unsigned)(0) timestamp, /**< The time when the depth image was taken. */
});
