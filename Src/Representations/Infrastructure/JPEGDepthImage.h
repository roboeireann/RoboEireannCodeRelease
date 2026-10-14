/**
 * @file JPEGDepthImage.h
 *
 * Declaration of a representation that contains JPEG-compressed
 * depth images. The current version is based on
 * https://dev.realsenseai.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras ,
 * i.e. converting depth data to RGB and then using JPEG to compress them.
 */

#pragma once

#include "DepthImage.h"
#include "Math/Range.h"

struct JPEGDepthImage : public Streamable
{
private:
  unsigned size; /**< The size of the compressed image. */
  int width; /**< The width of the image in pixel */
  int height; /**< The height of the image in pixel */
  Rangei depthRange; /**< The minimum and maximum depth store in the image. */
  std::vector<unsigned char> allocator; /**< The data storage */

public:
  JPEGDepthImage() = default;

  /**
   * Constructs a JPEG depth image from a depth image.
   * @param src The image used as template.
   */
  JPEGDepthImage(const DepthImage& src);

  /**
   * Assignment operator.
   * @param src The depth image used as template.
   * @return The resulting JPEG depth image.
   */
  JPEGDepthImage& operator=(const DepthImage& src);

  /**
   * Compress depth image and store result in this object.
   * @param src The depth image to compress.
   * @param quality The quality of the resulting image (0 = bad ... 100 = very good).
   * @param depthRange Valid depth range considered. Measurements are clamped to this range.
   */
  void fromDepthImage(const DepthImage& src, int quality = 75, const Rangei& depthRange = {200, 6000});

  /**
   * Uncompress image.
   * @param dest Will receive the uncompressed image.
   */
  void toDepthImage(DepthImage& dest) const;

  unsigned timestamp = 0; /**< The timestamp of this image. */

protected:
  /**
   * Read this object from a stream.
   * @param stream The stream from which the object is read.
   */
  void read(In& stream) override;

  /**
   * Write this object to a stream.
   * @param stream The stream to which the object is written.
   */
  void write(Out& stream) const override;

private:
  static void reg();
};
