/**
 * @file HbsTypes.h
 *
 * Core primitive types shared by HybridBehaviour and TickTrace.
 * Kept dependency-free so both headers can include this without circularity.
 */

#pragma once

namespace Hbs
{

enum Status
{
  INITIAL,   // Reserved for future use: node never ticked (for HSM integration)
  RUNNING,
  SUCCESS,
  FAILURE
};

using Time = unsigned int; // give flexibility to change how time is represented

struct VarStorage
{
  virtual ~VarStorage() = default;
};

} // namespace Hbs
