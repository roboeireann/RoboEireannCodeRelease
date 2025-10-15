#pragma once

#include <cstddef>

namespace Memory
{
  /** Allocate memory of given size with given alignment. */
#if defined TARGET_ROBOT || !defined __AVX2__  /* definitely not AVX2 */
  void* alignedMalloc(size_t size, size_t alignment = 32);
#else
  void* alignedMalloc(size_t size, size_t alignment = 64);
#endif

  /** Free aligned memory. */
  void alignedFree(void* ptr);
}
