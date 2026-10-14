/**
 * @file FmtFormatter.h
 *
 * This file declares a macro to simplify custom formatting of types using fmtlib
 *
 * @author Rudi Villing
 */

#pragma once

#include "fmt/core.h"


/**
 * define a custom formatter for a type
 * 
 * usage example:
 * 
 *   FMT_FORMATTER(Vector2f, v, "{{{:.0f}, {:.0f}}}", v.x(), v.y());
 * 
 * @param Type_ the type to be formatted
 * @param var_ the variable that refers to the specific instance being formatted
 * @param fmtString the format string that defines how the type will be formatted
 * @param fmtArgs... the zero of more args that will be substituted into the format string
 */

#define FMT_FORMATTER(Type_, var_, ...)                                                                                \
  template <> class fmt::formatter<Type_>                                                                              \
  {                                                                                                                    \
  public:                                                                                                              \
    constexpr auto parse(format_parse_context &ctx)                                                                    \
    {                                                                                                                  \
      return ctx.begin();                                                                                              \
    }                                                                                                                  \
    template <typename Context> constexpr auto format(const Type_ &var_, Context &ctx) const                           \
    {                                                                                                                  \
      return format_to(ctx.out(), __VA_ARGS__);                                                                        \
    }                                                                                                                  \
  };

/**
 * define a formatter for a derived type that simply reuses the base type's formatter.
 *
 * usage example:
 *
 *   FMT_DERIVED_FORMATTER(RobotPose, Pose2f);
 *
 * @param Type_ the derived type
 * @param BaseType_ the base type which already has a formatter defined
 */
#define FMT_DERIVED_FORMATTER(Type_, BaseType_)                                                                        \
  template <> class fmt::formatter<Type_> : public fmt::formatter<BaseType_>                                           \
  {                                                                                                                    \
  public:                                                                                                              \
    template <typename Context> auto format(const Type_ &var, Context &ctx) const                                      \
    {                                                                                                                  \
      return fmt::formatter<BaseType_>::format(static_cast<const BaseType_ &>(var), ctx);                              \
    }                                                                                                                  \
  }

/**
 * define a formatter for an ENUM type that uses TypeRegistry to get the string name.
 * 
 * usage example:
 * 
 *   FMT_ENUM_FORMATTER(MyEnum);
 * 
 * @param Type_ the enum type
 */
#define FMT_ENUM_FORMATTER(Type_)                                                                                      \
  template <> class fmt::formatter<Type_>                                                                              \
  {                                                                                                                    \
  public:                                                                                                              \
    constexpr auto parse(format_parse_context &ctx)                                                                    \
    {                                                                                                                  \
      return ctx.begin();                                                                                              \
    }                                                                                                                  \
    template <typename Context> auto format(const Type_ &var, Context &ctx) const                                      \
    {                                                                                                                  \
      const char* name = TypeRegistry::getEnumName(var);                                                               \
      return fmt::format_to char* name = TypeRegistry::getEnumName(var);                                               \
      return fmt::format_to(ctx.out(), "{}", name ? name : "Unknown");                                                 \
    }                                                                                                                  \
  }


