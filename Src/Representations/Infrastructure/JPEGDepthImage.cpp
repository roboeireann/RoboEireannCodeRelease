/**
 * @file JPEGDepthImage.cpp
 *
 * Implementation of a representation that contains JPEG-compressed
 * depth images. The current version is based on
 * https://dev.realsenseai.com/docs/depth-image-compression-by-colorization-for-intel-realsense-depth-cameras ,
 * i.e. converting depth data to RGB and then using JPEG to compress them.
 */

#include "JPEGDepthImage.h"
#include "ImageProcessing/SIMD.h"
#include "Math/BHMath.h"
#include "Platform/BHAssert.h"
#include "Platform/Memory.h"
#include <cstddef>
#include <jpeglib.h>
#include <limits>

struct RGB {unsigned char r, g, b;}; /**< Type for RGB pixel. */
static const Rangei valueRange(100, 1219); /**< Value range actually used. Unused space should prevent wraparounds due to JPEG artifacts. */

static boolean onDestEmpty(j_compress_ptr)
{
  FAIL("Unsupported operation.");
  return false;
}

static void onDestIgnore(j_compress_ptr) {}

static void onSrcSkip(j_decompress_ptr, long) {}

static boolean onSrcEmpty(j_decompress_ptr)
{
  FAIL("Unsupported operation.");
  return false;
}

static void onSrcIgnore(j_decompress_ptr) {}

JPEGDepthImage::JPEGDepthImage(const DepthImage& src)
{
  fromDepthImage(src);
}

JPEGDepthImage& JPEGDepthImage::operator=(const DepthImage& src)
{
  fromDepthImage(src);
  return *this;
}

void JPEGDepthImage::fromDepthImage(const DepthImage& src, int quality, const Rangei& depthRange)
{
  allocator.resize(src.width * src.height * sizeof(RGB));
  width = src.width;
  height = src.height;
  this->depthRange = depthRange;
  timestamp = src.timestamp;

  jpeg_compress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);
  jpeg_create_compress(&cInfo);

  if(!cInfo.dest)
    cInfo.dest = static_cast<jpeg_destination_mgr*>(
                 (*cInfo.mem->alloc_small)(reinterpret_cast<j_common_ptr>(&cInfo), JPOOL_PERMANENT, sizeof(jpeg_destination_mgr)));
  cInfo.dest->init_destination = onDestIgnore;
  cInfo.dest->empty_output_buffer = onDestEmpty;
  cInfo.dest->term_destination = onDestIgnore;
  cInfo.dest->next_output_byte = static_cast<JOCTET*>(allocator.data());
  cInfo.dest->free_in_buffer = allocator.size();

  cInfo.image_width = width;
  cInfo.image_height = height;
  cInfo.input_components = 3;
  cInfo.in_color_space = JCS_EXT_RGB;
  cInfo.jpeg_color_space = JCS_EXT_RGB;
  jpeg_set_defaults(&cInfo);
  cInfo.dct_method = JDCT_FASTEST;
  jpeg_set_quality(&cInfo, quality, true);

  jpeg_start_compress(&cInfo, true);

  std::vector<RGB> row;
  row.reserve(width);

  while(cInfo.next_scanline < cInfo.image_height)
  {
    row.clear();
    for(const unsigned short* pSrc = src[cInfo.next_scanline], * pEnd = pSrc + width; pSrc < pEnd; ++pSrc)
    {
      const int value = (depthRange.clamped(*pSrc) - depthRange.min) * (valueRange.getSize() + 1) / (depthRange.getSize() + 1) + valueRange.min;
      if(value <= 255)
        row.emplace_back(255, static_cast<unsigned char>(value), 0);
      else if(value <= 510)
        row.emplace_back(static_cast<unsigned char>(510 - value), 255, 0);
      else if(value <= 765)
        row.emplace_back(0, 255, static_cast<unsigned char>(value - 510));
      else if(value <= 1020)
        row.emplace_back(0, static_cast<unsigned char>(1020 - value), 255);
      else if(value <= 1275)
        row.emplace_back(static_cast<unsigned char>(value - 1020), 0, 255);
      else
        row.emplace_back(255, 0, static_cast<unsigned char>(1529 - value));
    }

    JSAMPROW rowPointer = const_cast<JSAMPROW>(reinterpret_cast<const unsigned char*>(row.data()));
    jpeg_write_scanlines(&cInfo, &rowPointer, 1);
  }

  jpeg_finish_compress(&cInfo);
  size = static_cast<unsigned>(static_cast<char unsigned*>(cInfo.dest->next_output_byte) - allocator.data());
  jpeg_destroy_compress(&cInfo);
}

void JPEGDepthImage::toDepthImage(DepthImage& dest) const
{
  dest.setResolution(width, height);
  dest.timestamp = timestamp;

  jpeg_decompress_struct cInfo;
  jpeg_error_mgr jem;
  cInfo.err = jpeg_std_error(&jem);

  jpeg_create_decompress(&cInfo);

  if(!cInfo.src)
    cInfo.src = static_cast<jpeg_source_mgr*>(
                (*cInfo.mem->alloc_small)(reinterpret_cast<j_common_ptr>(&cInfo), JPOOL_PERMANENT, sizeof(jpeg_source_mgr)));
  cInfo.src->init_source       = onSrcIgnore;
  cInfo.src->fill_input_buffer = onSrcEmpty;
  cInfo.src->skip_input_data   = onSrcSkip;
  cInfo.src->resync_to_restart = jpeg_resync_to_restart;
  cInfo.src->term_source       = onSrcIgnore;
  cInfo.src->bytes_in_buffer   = allocator.size();
  cInfo.src->next_input_byte   = static_cast<const JOCTET*>(allocator.data());

  jpeg_read_header(&cInfo, true);
  jpeg_start_decompress(&cInfo);

  std::vector<RGB> row(width);

  if(cInfo.num_components == 3) // full size images
  {
    // setup rows
    while(cInfo.output_scanline < cInfo.output_height)
    {
      unsigned short* pDest = dest[cInfo.output_scanline];
      JSAMPROW rowPointer = reinterpret_cast<unsigned char*>(row.data());
      static_cast<void>(jpeg_read_scanlines(&cInfo, &rowPointer, 1));
      for(const RGB pixel : row)
      {
        const int value = pixel.r >= pixel.g && pixel.r >= pixel.b
                          ? (pixel.g >= pixel.b
                             ? pixel.g - pixel.b
                             : 1529 + pixel.g - pixel.b)
                          : pixel.g >= pixel.r && pixel.g >= pixel.b
                          ? 510 + pixel.b - pixel.r
                          : 1020 + pixel.r - pixel.g;

        *pDest++ = static_cast<unsigned short>((valueRange.clamped(value) - valueRange.min) * (depthRange.getSize() + 1) / (valueRange.getSize() + 1) + depthRange.min);
      }
    }
  }
  else
  {
    FAIL("Unsupported number of colors: " << cInfo.num_components << ".");
  }

  // finish decompress
  jpeg_finish_decompress(&cInfo);
  jpeg_destroy_decompress(&cInfo);
}

void JPEGDepthImage::read(In& stream)
{
  STREAM(width);
  STREAM(height);
  STREAM(depthRange)
  STREAM(timestamp);
  STREAM(size);
  allocator.resize(size);
  stream.read(allocator.data(), size);
}

void JPEGDepthImage::write(Out& stream) const
{
  unsigned timestamp = this->timestamp | 1 << 31;
  STREAM(width);
  STREAM(height);
  STREAM(depthRange)
  STREAM(timestamp);
  STREAM(size);
  stream.write(allocator.data(), size);
}

void JPEGDepthImage::reg()
{
  PUBLISH(reg);
  REG_CLASS(JPEGDepthImage);
  REG(width);
  REG(height);
  REG(depthRange);
  REG(timestamp);
  REG(size);
}
