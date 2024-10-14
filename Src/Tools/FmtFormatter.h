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
    template <typename Context> constexpr auto format(Type_ const &var_, Context &ctx) const                           \
    {                                                                                                                  \
      return format_to(ctx.out(), __VA_ARGS__);                                                                        \
    }                                                                                                                  \
  }

