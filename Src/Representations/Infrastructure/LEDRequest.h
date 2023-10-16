/**
 * @file Representations/Infrastructure/LEDRequest.h
 *
 * This file contains the LEDRequest struct.
 * 
 * [RV] Added new convenience interface to provide greater control over LEDs
 * while providing a simpler interface
 *
 * @author Thomas Röfer
 * @author Rudi Villing
 */

#pragma once

#include "Tools/Streams/EnumIndexedArray.h"


struct LEDColor
{
  float r;
  float g;
  float b;

  LEDColor() : r(0), g(0), b(0) {}
  LEDColor(float rgb) : r(rgb), g(rgb), b(rgb) {}
  LEDColor(float r, float g, float b) : r(r), g(g), b(b) {}

  static const LEDColor RED;
  static const LEDColor GREEN;
  static const LEDColor BLUE;
  static const LEDColor WHITE;
  static const LEDColor MAGENTA;
  static const LEDColor PURPLE;
  static const LEDColor ORANGE_YELLOW;
  static const LEDColor BRIGHT_YELLOW;
  static const LEDColor CYAN;
  static const LEDColor BROWN;
  static const LEDColor GRAY;
  static const LEDColor NONE; // same as black
};



/**
 * This describes a LEDRequest
 */
STREAMABLE(LEDRequest,
{
  /** ids for all LEDs */
  ENUM(LED,
  {,
    faceLeftRed0Deg,
    faceLeftRed45Deg,
    faceLeftRed90Deg,
    faceLeftRed135Deg,
    faceLeftRed180Deg,
    faceLeftRed225Deg,
    faceLeftRed270Deg,
    faceLeftRed315Deg,
    faceLeftGreen0Deg,
    faceLeftGreen45Deg,
    faceLeftGreen90Deg,
    faceLeftGreen135Deg,
    faceLeftGreen180Deg,
    faceLeftGreen225Deg,
    faceLeftGreen270Deg,
    faceLeftGreen315Deg,
    faceLeftBlue0Deg,
    faceLeftBlue45Deg,
    faceLeftBlue90Deg,
    faceLeftBlue135Deg,
    faceLeftBlue180Deg,
    faceLeftBlue225Deg,
    faceLeftBlue270Deg,
    faceLeftBlue315Deg,
    faceRightRed0Deg,
    faceRightRed45Deg,
    faceRightRed90Deg,
    faceRightRed135Deg,
    faceRightRed180Deg,
    faceRightRed225Deg,
    faceRightRed270Deg,
    faceRightRed315Deg,
    faceRightGreen0Deg,
    faceRightGreen45Deg,
    faceRightGreen90Deg,
    faceRightGreen135Deg,
    faceRightGreen180Deg,
    faceRightGreen225Deg,
    faceRightGreen270Deg,
    faceRightGreen315Deg,
    faceRightBlue0Deg,
    faceRightBlue45Deg,
    faceRightBlue90Deg,
    faceRightBlue135Deg,
    faceRightBlue180Deg,
    faceRightBlue225Deg,
    faceRightBlue270Deg,
    faceRightBlue315Deg,
    earsLeft0Deg,
    earsLeft36Deg,
    earsLeft72Deg,
    earsLeft108Deg,
    earsLeft144Deg,
    earsLeft180Deg,
    earsLeft216Deg,
    earsLeft252Deg,
    earsLeft288Deg,
    earsLeft324Deg,
    earsRight0Deg,
    earsRight36Deg,
    earsRight72Deg,
    earsRight108Deg,
    earsRight144Deg,
    earsRight180Deg,
    earsRight216Deg,
    earsRight252Deg,
    earsRight288Deg,
    earsRight324Deg,
    chestRed,
    chestGreen,
    chestBlue,
    firstHeadLED,
    headRearLeft0 = firstHeadLED,
    headRearLeft1,
    headRearLeft2,
    headRearRight0,
    headRearRight1,
    headRearRight2,
    headMiddleRight0,
    headFrontRight0,
    headFrontRight1,
    headFrontLeft0,
    headFrontLeft1,
    headMiddleLeft0,
    lastHeadLED = headMiddleLeft0,
    footLeftRed,
    footLeftGreen,
    footLeftBlue,
    footRightRed,
    footRightGreen,
    footRightBlue,
  });

  static constexpr size_t numOfHeadLeds = LED::headMiddleLeft0 - LED::headRearLeft0 + 1;
  static constexpr size_t numOfEarLeds = LED::earsLeft324Deg - LED::earsLeft0Deg + 1;
  static constexpr size_t numOfEyeLeds = LED::faceLeftRed315Deg - LED::faceLeftRed0Deg + 1;

  ENUM(LEDState,
  {,
    off,
    // for all following values, the float values in ledValues array are used
    on,
    blinking,
    blinking2, // antiphase to blinking (i.e. on when blinking is off and vice versa)
    fastBlinking,
    fastBlinking2, // antiphase to fastBlinking
    tripleBlink, // 3 fast blinks and a gap
    tripleBlink2, // antiphase to tripleBlink
    fade, // 3000 ms period, 50% duty cycle
    fade2, // 3000 ms period, 75% duty cycle
    // deprecated legacy interface
    half,
  });

  ENUM(Side,
  {,
    left,
    right,
  });



  static constexpr int FULL_SET = -1; // use for led index when all LEDs in set should be changed

  LEDRequest()
  {
    FOREACH_ENUM(LED, i)
    {
      ledStates[i] = off;
      ledValues[i] = 0.f;
    }
  }

  bool operator==(const LEDRequest& other) const
  {
    FOREACH_ENUM(LED, i)
      if ((ledStates[i] != other.ledStates[i]) || (ledValues[i] != other.ledValues[i]))
        return false;
    return true;
  }

  bool operator!=(const LEDRequest& other) const
  {
    return !(*this == other);
  }

  // low level interface
  void setLed(LED led, LEDState state, float value = 1.f)
  {
    // handle legacy values
    if (state == half)
    {
      state = on;
      value = 0.5f;
    }

    ledStates[led] = state;
    ledValues[led] = (state == off) ? 0.f : value;
  }

  /**
   * set one eye LED or all LEDs in that eye to a specific colour
   */
  void setEyeLeds(Side side, int index, LEDState state, const LEDColor& color)
  {
    static const int redOffset = 0;
    static const int greenOffset = faceLeftGreen0Deg - faceLeftRed0Deg;
    static const int blueOffset = faceLeftBlue0Deg - faceLeftRed0Deg;
    int firstLed = (side == left) ? faceLeftRed0Deg : faceRightRed0Deg;

    const int minIndex = index < 0 ? 0 : std::min(index, int(numOfEyeLeds)-1);
    const int maxIndex = index < 0 ? int(numOfEyeLeds)-1 : minIndex;

    for (int i=minIndex; i <= maxIndex; i++)
    {
      setLed(LED(firstLed + redOffset + i), state, color.r);
      setLed(LED(firstLed + greenOffset + i), state, color.g);
      setLed(LED(firstLed + blueOffset + i), state, color.b);
    }
  }

  /// convenience wrapper (with fewer params) for function above
  void setEyeLeds(Side side, const LEDColor& color) { setEyeLeds(side, FULL_SET, on, color); }

  /**
   * set the chest LEDs to a specific colour
   */
  void setChestLeds(LEDState state, const LEDColor& color)
  {
    setLed(chestRed, state, color.r);
    setLed(chestGreen, state, color.g);
    setLed(chestBlue, state, color.b);
  }

  /**
   * set the foot LEDs to a specific colour
   */
  void setFootLeds(Side side, LEDState state, const LEDColor& color)
  {
    static const int greenOffset = footLeftGreen - footLeftRed;
    static const int blueOffset = footLeftBlue - footLeftRed;
    int firstLed = (side == left) ? footLeftRed : footRightRed;

    setLed(LED(firstLed), state, color.r);
    setLed(LED(firstLed + greenOffset), state, color.g);
    setLed(LED(firstLed + blueOffset), state, color.b);
  }
  
  /**
   * set the ear LEDs to a specific value
   * 
   * Note that the ear LEDs are blue only so they only have a brightness, no colour.
   * The brightness values are pretty difficult to distinguish so recommend
   * just on or off
   */
  void setEarLeds(Side side, int index, LEDState state, float value)
  {
    int firstLed = (side == left) ? earsLeft0Deg : earsRight0Deg;
    const int minIndex = index < 0 ? 0 : std::min(index, int(numOfEarLeds)-1);
    const int maxIndex = index < 0 ? int(numOfEarLeds)-1 : minIndex;

    for (int i=minIndex; i <= maxIndex; i++)
      setLed(LED(firstLed + i), state, value);
  }

  /**
   * set the head LEDs to a specific value
   * 
   * Note that the head LEDs are white only so they only have a brightness, no colour.
   */
  void setHeadLeds(int index, LEDState state, float value)
  {
    const int minIndex = index < 0 ? 0 : std::min(index, int(numOfHeadLeds)-1);
    const int maxIndex = index < 0 ? int(numOfHeadLeds)-1 : minIndex;

    for (int i=minIndex; i <= maxIndex; i++)
      setLed(LED(firstHeadLED + i), state, value);
  }


  // the streamable values begin after the comma
  ,

  (ENUM_INDEXED_ARRAY(LEDRequest::LEDState, LED)) ledStates, /**< The intended states of the LEDs (use type State). */
  (ENUM_INDEXED_ARRAY(float, LED)) ledValues, /**< The intended float values of the LEDs in all states except off */
});
