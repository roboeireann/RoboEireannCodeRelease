/**
 * @file Representations/Infrastructure/LedRequestK1.h
 *
 * The LedRequestK1 struct represents the requested state of the single
 * back panel LED on the Booster K1 robot. It can have both a background
 * and foreground state - the foreground always overrides the background
 * so certain combinations of foreground and background are effectively
 * meaningless.
 *
 * @author Rudi Villing
 */

#pragma once

STREAMABLE(LedColorK1,
{
  static const LedColorK1 NONE; // same as black in normal colours
  static const LedColorK1 RED;
  static const LedColorK1 MID_RED;
  static const LedColorK1 DARK_RED;
  static const LedColorK1 GREEN;
  static const LedColorK1 MID_GREEN;
  static const LedColorK1 DARK_GREEN;
  static const LedColorK1 BLUE;
  static const LedColorK1 MID_BLUE;
  static const LedColorK1 DARK_BLUE;
  static const LedColorK1 WHITE;
  static const LedColorK1 PINK;
  static const LedColorK1 PURPLE;
  static const LedColorK1 MAGENTA;
  static const LedColorK1 ORANGE_YELLOW;
  static const LedColorK1 BRIGHT_YELLOW;
  static const LedColorK1 MID_YELLOW;
  static const LedColorK1 DARK_YELLOW;
  static const LedColorK1 CYAN;
  static const LedColorK1 BROWN;
  static const LedColorK1 GRAY;
  static const LedColorK1 LIGHT_GRAY;
  static const LedColorK1 DARK_BLUE_GRAY;


  constexpr LedColorK1() = default;
  constexpr LedColorK1(uint8_t r, uint8_t g, uint8_t b) : r(r) COMMA g(g) COMMA b(b) {}

  bool operator==(const LedColorK1& o) const { return r == o.r && g == o.g && b == o.b; };

  /// make a colour from float r,g,b values
  static LedColorK1 make(float r, float g, float b)
  {
    return LedColorK1(static_cast<uint8_t>(255.f * std::clamp(r, 0.f, 1.f)),
                      static_cast<uint8_t>(255.f * std::clamp(g, 0.f, 1.f)),
                      static_cast<uint8_t>(255.f * std::clamp(b, 0.f, 1.f)));
  }

  LedColorK1 scaled(float scale)
  {
    return LedColorK1(static_cast<uint8_t>(r * scale),
                      static_cast<uint8_t>(g * scale),
                      static_cast<uint8_t>(b * scale));
  }

  /**** separate header from streamable fields - comma at end of comment ****/,

  (uint8_t)(0) r,
  (uint8_t)(0) g,
  (uint8_t)(0) b,
});


inline const LedColorK1 LedColorK1::NONE = {0, 0, 0}; // same as black in normal colours
inline const LedColorK1 LedColorK1::RED = {255, 0, 0};
inline const LedColorK1 LedColorK1::MID_RED = {128, 0, 0};
inline const LedColorK1 LedColorK1::DARK_RED = {64, 0, 0};
inline const LedColorK1 LedColorK1::GREEN = {0, 255, 0};
inline const LedColorK1 LedColorK1::MID_GREEN = {0, 128, 0};
inline const LedColorK1 LedColorK1::DARK_GREEN = {0, 64, 0};
inline const LedColorK1 LedColorK1::BLUE = {0, 0, 255};
inline const LedColorK1 LedColorK1::MID_BLUE = {0, 0, 128};
inline const LedColorK1 LedColorK1::DARK_BLUE = {0, 0, 64};
inline const LedColorK1 LedColorK1::WHITE = {255, 255, 255};
inline const LedColorK1 LedColorK1::PINK = {255, 77, 77};
inline const LedColorK1 LedColorK1::PURPLE = {128, 0, 255};
inline const LedColorK1 LedColorK1::MAGENTA = {255, 0, 255};
inline const LedColorK1 LedColorK1::ORANGE_YELLOW = {255, 128, 0};
inline const LedColorK1 LedColorK1::BRIGHT_YELLOW = {255, 225, 0};
inline const LedColorK1 LedColorK1::MID_YELLOW = {160, 120, 0};
inline const LedColorK1 LedColorK1::DARK_YELLOW = {75, 55, 0};
inline const LedColorK1 LedColorK1::CYAN = {0, 128, 255};
inline const LedColorK1 LedColorK1::BROWN = {77, 38, 0};
inline const LedColorK1 LedColorK1::GRAY = {77, 77, 77};
inline const LedColorK1 LedColorK1::LIGHT_GRAY = {152, 152, 152};
inline const LedColorK1 LedColorK1::DARK_BLUE_GRAY = {40, 40, 64};


/**
 * This describes a LedRequestK1
 *
 * You can specify a foreground and background for the LED.
 * Foreground layer: takes priority over bg whenever its mask bit is set (or always, for fades).
 * Fade/Fade2 fg with any bg is a meaningless combo — fg is never "off" to reveal bg.
 * Using a blink pattern on the fg only makes sense if the bg is on during times
 * when the fg is off (e.g. if the background is full on, a fade, or antiphase blinking)
 */
STREAMABLE(LedRequestK1,
{
  ENUM(Mode,
  {, 
    binaryPattern, 
    fade, // slow pulse over full period (part=halfPeriod)
    fade2, // pulse over full period, but rise and fall take place over 0.25 period each and fully on for half period (part=quarterPeriod)
    fade3, // pulse over 0.75 period, and 0.25 period off (part=quarterPeriod)
    fade4, // pulse over 0.5 period, and 0.5 period off (part=quarterPeriod)
  });


  // Bitmask patterns for off, on (unblinking), and various blink patterns.
  // It uses 32 bits over a 1600ms macro cycle, so each bit represents 50ms.
  // Bit 0 (LSB, rightmost) = [0,50ms); Bit 31 (MSB, leftmost) = [1550,1600ms).
  static constexpr uint32_t OFF         = 0b00000000'00000000'00000000'00000000;
  static constexpr uint32_t ON          = 0b11111111'11111111'11111111'11111111;
  static constexpr uint32_t BLINK       = 0b00000000'11111111'00000000'11111111;
  static constexpr uint32_t BLINK2      = 0b11111111'00000000'11111111'00000000; // antiphase to blink
  static constexpr uint32_t FAST_BLINK  = 0b00110011'00110011'00110011'00110011;
  static constexpr uint32_t FAST_BLINK2 = 0b11001100'11001100'11001100'11001100; // antiphase to fast blink
  static constexpr uint32_t VERY_FAST_BLINK  = 0b01001001'00100100'10010010'01001001;
  static constexpr uint32_t SLOW_DOUBLE = 0b00000000'00000000'00001111'00001111; // double blink, gap
  static constexpr uint32_t FAST_DOUBLE = 0b00000000'00110011'00000000'00110011; // fast double blink, gap
  static constexpr uint32_t TRIPLE      = 0b00000000'00010101'00000000'00010101; // triple blink, gap
  static constexpr uint32_t TRIPLE2     = 0b00010101'00000000'00010101'00000000; // antiphase to triple


  bool operator==(const LedRequestK1 &other) const
  {
    return (fgColor == other.fgColor && fgMode == other.fgMode && fgMask == other.fgMask) &&
           (bgColor == other.bgColor && bgMode == other.bgMode && bgMask == other.bgMask);
  }

  /**
   * set the colours and patterns
   */
  void setFgPattern(const LedColorK1 &color, uint32_t pattern = ON)
  {
    fgColor = color;
    fgMode = Mode::binaryPattern;
    fgMask = pattern;
  }

  /**
   * set the fade
   *
   * @param color  the colour of the LEDs
   * @param fadeMode  what type of fade to use, @see Mode comments for details
   * @param offsetIndex the offset part index - depends on the fade type, but offsets by either half or quarter period
   */
  void setFgFade(const LedColorK1 &color, Mode fadeMode = Mode::fade, unsigned offsetIndex=0)
  {
    fgColor = color;
    ASSERT(fadeMode == Mode::fade || fadeMode == Mode::fade2 || fadeMode == Mode::fade3 || fadeMode == Mode::fade4);
    ASSERT(offsetIndex<4);
    fgMode = fadeMode;
    fgMask = offsetIndex;
  }

  void setBgPattern(const LedColorK1 &color, uint32_t pattern = ON)
  {
    bgColor = color;
    bgMode = Mode::binaryPattern;
    bgMask = pattern;
  }

  void setBgFade(const LedColorK1 &color, Mode fadeMode = Mode::fade, unsigned offsetIndex=0)
  {
    bgColor = color;
    ASSERT(fadeMode == Mode::fade || fadeMode == Mode::fade2 || fadeMode == Mode::fade3 || fadeMode == Mode::fade4);
    ASSERT(offsetIndex<4);
    bgMode = fadeMode;
    bgMask = offsetIndex;
  }



  /**** separate header from streamable fields - comma at end of comment ****/,

  (LedColorK1)(LedColorK1::NONE) fgColor, ///< the foreground LED colour (overrides bg state)
  (Mode)(Mode::binaryPattern) fgMode, ///< the foreground mode
  (uint32_t)(LedRequestK1::OFF) fgMask, ///< the foreground on/off pattern if not fade

  (LedColorK1)(LedColorK1::NONE) bgColor, ///< the background LED colour
  (Mode)(Mode::binaryPattern) bgMode, ///< the background mode
  (uint32_t)(LedRequestK1::OFF) bgMask, ///< the background on/off pattern if not fade
});
