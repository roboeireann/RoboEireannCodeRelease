/**
 * @file GamePad.h
 *
 * A struct that represents the current snapshot of an Xbox style game pad/controller
 * that may be used to remotely control a robot
 *
 * @author Rudi Villing
 */

#pragma once

#include "Streaming/EnumIndexedArray.h"
#include "Streaming/Function.h"

STREAMABLE(GamePad,
{
  // // intentionally using an enum bit mask rather than ENUM with consecutive values
  // HIDE_COMMAS(enum Button : unsigned
  // {
  //   a     = 0x0001,
  //   b     = 0x0002,
  //   x     = 0x0004,
  //   y     = 0x0008,
  //   lb    = 0x0010,
  //   rb    = 0x0020,
  //   lt    = 0x0040,  // digital threshold; analog is lTrigger
  //   rt    = 0x0080,
  //   ls    = 0x0100,  // left stick click
  //   rs    = 0x0200,
  //   back  = 0x0400,
  //   start = 0x0800,
  //   guide = 0x1000 // unused for now
  // });

  ENUM(Button,
  {,
    a,
    b,
    x,
    y,
    lb,
    rb,
    lt,  // digital threshold; analog is lTrigger
    rt,
    ls,  // left stick click
    rs,
    back,
    start,
  });

  static constexpr unsigned maskA = 1 << a;
  static constexpr unsigned maskB = 1 << b;
  static constexpr unsigned maskX = 1 << x;
  static constexpr unsigned maskY = 1 << y;
  static constexpr unsigned maskLB = 1 << lb;
  static constexpr unsigned maskRB = 1 << rb;
  static constexpr unsigned maskLT = 1 << lt;
  static constexpr unsigned maskRT = 1 << rt;
  static constexpr unsigned maskLS = 1 << ls;
  static constexpr unsigned maskRS = 1 << rs;
  static constexpr unsigned maskBack = 1 << back;
  static constexpr unsigned maskStart = 1 << start;

  static constexpr unsigned maskAny = 0xFFFF;

  ENUM(Hat,
  {,
    centered,
    up,
    upRight,
    right,
    downRight,
    down,
    downLeft,
    left,
    upLeft,
  });

  bool hatActive() const { return hat != Hat::centered; }


  /// check if a single button is pressed using the button index (from Button enum)
  /// Take care to ensure you use a Button enum value (cast if needed) if you want this overload.
  bool pressed(Button buttonIndex) const { return pressed(buttonIndex, buttons); }

  /// check if <b>ALL</b> the buttons in some mask were pressed. BitwiseOr the individual 
  /// button masks together to specify a combination.
  bool pressed(unsigned mask) const { return pressed(mask, buttons); }

  /// check if <b>ANY</b> button within the specified mask is pressed
  bool anyPressed(unsigned mask) const { return anyPressed(mask, buttons); }

  // --- static helpers (useful for provider update logic) ---
  // buttonBits refers to a snapshot of the button state at some point

  static bool pressed(Button buttonIndex, unsigned buttonBits) { return (buttonBits & (1 << buttonIndex)) != 0; }
  static bool pressed(unsigned mask, unsigned buttonBits) { return (buttonBits & mask) == mask; }
  static bool anyPressed(unsigned mask, unsigned buttonBits) { return (buttonBits & mask) != 0; }


  /**** separate header from streamable fields - comma at end of comment ****/,

  (bool)(false) connected,

  (unsigned)(0) buttons,         // currently held - e.g. if (buttons & (Button::lt | Button::a)) doSomething...

  (float)(0.f) lStickX,   // –1.0 (left) … +1.0 (right)
  (float)(0.f) lStickY,   // –1.0 (fwd)  … +1.0 (back)
  (float)(0.f) rStickX,
  (float)(0.f) rStickY,

  // analogue triggers - not used currently
  // (float)(0.f) lTrigger,  // 0.0 (off) … 1.0 (full)
  // (float)(0.f) rTrigger,

  (Hat)(centered) hat,
});


STREAMABLE_WITH_BASE(GamePadEx, GamePad, 
{
  bool hatChanged() const { return hat != prevHat; }

  /// has any button within the masked set changed
  bool buttonsChanged(unsigned mask = maskAny) const { return (buttons & mask) != (prevButtons & mask); }

  /// which buttons have been pressed since last check
  unsigned buttonsNew() const { return ~prevButtons & buttons; } 
  /// which buttons have been released since last check
  unsigned buttonsReleased() const { return prevButtons & ~buttons; } // just released in this snapshot

  /// check if the user has just completed the button combination specified in mask
  bool justPressed(unsigned mask) const { return pressed(mask) && !pressed(mask, prevButtons); }
  /// check if the user has just pressed the single button index specified
  bool justPressed(Button btnIndex) const { return pressed(btnIndex) && !pressed(btnIndex, prevButtons); }


  /**** separate header from streamable fields - comma at end of comment ****/,
  (unsigned)(0) prevButtons,
  (Hat)(centered) prevHat,
});