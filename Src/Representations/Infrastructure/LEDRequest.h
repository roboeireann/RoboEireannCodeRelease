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
  static const LEDColor PINK;
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
    firstLeftEyeLED,
    faceLeftRed0Deg = firstLeftEyeLED,
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
    firstRightEyeLED,
    faceRightRed0Deg = firstRightEyeLED,
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
    firstLeftEarLED,
    earsLeft0Deg = firstLeftEarLED,
    earsLeft36Deg,
    earsLeft72Deg,
    earsLeft108Deg,
    earsLeft144Deg,
    earsLeft180Deg,
    earsLeft216Deg,
    earsLeft252Deg,
    earsLeft288Deg,
    earsLeft324Deg,
    firstRightEarLED,
    earsRight0Deg = firstRightEarLED,
    earsRight36Deg,
    earsRight72Deg,
    earsRight108Deg,
    earsRight144Deg,
    earsRight180Deg,
    earsRight216Deg,
    earsRight252Deg,
    earsRight288Deg,
    earsRight324Deg,
    firstChestLED,
    chestRed = firstChestLED,
    chestGreen,
    chestBlue,
    firstHeadLED,
    headFrontRight1 = firstHeadLED,
    headFrontRight0,
    headMiddleRight0,
    headRearRight0,
    headRearRight1,
    headRearRight2,
    headRearLeft2,
    headRearLeft1,
    headRearLeft0,
    headMiddleLeft0,
    headFrontLeft0,
    headFrontLeft1,
    lastHeadLED = headFrontLeft1,
    firstLeftFootLED,
    footLeftRed = firstLeftFootLED,
    footLeftGreen,
    footLeftBlue,
    firstRightFootLED,
    footRightRed = firstRightFootLED,
    footRightGreen,
    footRightBlue,
  });

  static constexpr size_t numOfHeadLeds = LED::lastHeadLED - LED::firstHeadLED + 1;
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
    slowDoubleBlink, // slower double blink and gap
    fastDoubleBlink, // faster double blink and gap
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



  static constexpr unsigned MASK = 0x8000; // set bit 15
  static constexpr unsigned ALL_LEDS = 0xFFFF; // all possible LED indexes specified

  LEDRequest();

  bool operator==(const LEDRequest& other) const;
  bool operator!=(const LEDRequest& other) const
  {
    return !(*this == other);
  }

  // low level interface
  void setLed(LED led, float value, LEDState state = on);

  /**
   * set one eye LED or all LEDs in that eye to a specific colour
   * 
   * There are 2 main ways to call this function. To set a specific LED by
   * index, e.g. set the 2nd eye led to red, use
   * 
   *     setEyeLeds(left, 2, LEDColor::RED, on)
   * 
   * To set a number of leds at once, use a mask. E.g. to set first, third, and
   * fourth LED to BLUE, then the fifth and sixth to RED and the remainder off, use
   * 
   *     setEyeLeds(left, 0b00001101, LEDColor::BLUE, on)
   *     setEyeLeds(left, 0b00110000, LEDColor::RED, on)
   *     setEyeLeds(left, 0b11000000, LEDColor::NONE, off)
   * 
   * @param side left or right eye
   * @param indexOrMask by default this value refers to an index within the eye LED set.
   *                    If the MASK bit is set, then this is treated as a mask and the color and state
   *                    will be applied to the LEDs whose corresponding bits are set to 1.
   *                    LEDs whose bits are 0 will be unaffected. 
   * @param color the color to apply to the LED (unless the state is off)
   * @param state whether to switch the LED on, blink it, fade it, or switch it off
   * Generally you'll need to
   *                    call this function a second time to explicitly switch off other LEDs using
   *                    the off state.
   */
  void setEyeLeds(Side side, unsigned indexOrMask, const LEDColor& color, LEDState state = on);

  /**
   * set the chest LEDs to a specific colour
   */
  void setChestLeds(const LEDColor& color, LEDState state = on);

  /**
   * set the foot LEDs to a specific colour
   */
  void setFootLeds(Side side, const LEDColor& color, LEDState state = on);
  
  /**
   * set the ear LEDs to a specific value
   * 
   * Note that the ear LEDs are blue only so they only have a brightness, no colour.
   * The brightness values are pretty difficult to distinguish so recommend
   * just on or off
   * 
   * See setEyeLeds for use of indexOrMask
   */
  void setEarLeds(Side side, unsigned indexOrMask, float value, LEDState state = on);

  /**
   * set the head LEDs to a specific value
   * 
   * Note that the head LEDs are white only so they only have a brightness, no colour.
   * 
   * See setEyeLeds for use of indexOrMask
   */
  void setHeadLeds(unsigned indexOrMask, float value, LEDState state = on);


  /* ----------------- separate header from streamable members (note comma at end) ------------------ */,

  (ENUM_INDEXED_ARRAY(LEDRequest::LEDState, LED)) ledStates, /**< The intended states of the LEDs (use type State). */
  (ENUM_INDEXED_ARRAY(float, LED)) ledValues, /**< The intended float values of the LEDs in all states except off */
});
