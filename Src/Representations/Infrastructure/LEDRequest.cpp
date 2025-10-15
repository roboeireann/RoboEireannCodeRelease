/**
 * @file Representations/Infrastructure/LEDRequest.cpp
 *
 * Mainly needed to support static members of representation
 * 
 * [RV] Added new convenience interface to provide greater control over LEDs
 * while providing a simpler interface
 *
 * @author Rudi Villing
 */


#include "LEDRequest.h"

const LEDColor LEDColor::RED = LEDColor(1.f, 0.f, 0.f);
const LEDColor LEDColor::GREEN = LEDColor(0.f, 1.f, 0.f);
const LEDColor LEDColor::BLUE = LEDColor(0.f, 0.f, 1.f);
const LEDColor LEDColor::WHITE = LEDColor(1.f, 1.f, 1.f);
const LEDColor LEDColor::PINK = LEDColor(1.f, 0.3f, 0.3f);
const LEDColor LEDColor::PURPLE = LEDColor(0.5f, 0.f, 1.f);
const LEDColor LEDColor::MAGENTA = LEDColor(1.f, 0.f, 1.f);
const LEDColor LEDColor::ORANGE_YELLOW = LEDColor(1.f, 0.5f, 0.f);
const LEDColor LEDColor::BRIGHT_YELLOW = LEDColor(1.f, 1.0f, 0.f);
const LEDColor LEDColor::CYAN = LEDColor(0.f, 0.5f, 1.f);
const LEDColor LEDColor::BROWN = LEDColor(0.3f, 0.15f, 0.f);
const LEDColor LEDColor::GRAY = LEDColor(0.3f, 0.3f, 0.3f);
const LEDColor LEDColor::NONE = LEDColor(0.f, 0.f, 0.f);



LEDRequest::LEDRequest()
{
  FOREACH_ENUM(LED, i)
  {
    ledStates[i] = off;
    ledValues[i] = 0.f;
  }
}

bool LEDRequest::operator==(const LEDRequest &other) const
{
  FOREACH_ENUM(LED, i)
  {
    if ((ledStates[i] != other.ledStates[i]) || (ledValues[i] != other.ledValues[i]))
      return false;
  }
  return true;
}

// low level interface
void LEDRequest::setLed(LED led, float value, LEDState state)
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
void LEDRequest::setEyeLeds(Side side, unsigned indexOrMask, const LEDColor &color, LEDState state)
{
  static const int redOffset = 0;
  static const int greenOffset = faceLeftGreen0Deg - faceLeftRed0Deg;
  static const int blueOffset = faceLeftBlue0Deg - faceLeftRed0Deg;
  int firstLed = (side == left) ? faceLeftRed0Deg : faceRightRed0Deg;

  if (indexOrMask & MASK) // is it a bit mask for the LEDs?
  {
    for (unsigned i=0; i < numOfEyeLeds; i++)
      if (indexOrMask & (0x1 << i))
      {
        setLed(LED(firstLed + redOffset + i), color.r, state);
        setLed(LED(firstLed + greenOffset + i), color.g, state);
        setLed(LED(firstLed + blueOffset + i), color.b, state);
      }
  }
  else
  {
    ASSERT(indexOrMask < numOfEyeLeds);

    setLed(LED(firstLed + redOffset + indexOrMask), color.r, state);
    setLed(LED(firstLed + greenOffset + indexOrMask), color.g, state);
    setLed(LED(firstLed + blueOffset + indexOrMask), color.b, state);
  }
}

/**
 * set the chest LEDs to a specific colour
 */
void LEDRequest::setChestLeds(const LEDColor &color, LEDState state)
{
  setLed(chestRed, color.r, state);
  setLed(chestGreen, color.g, state);
  setLed(chestBlue, color.b, state);
}

/**
 * set the foot LEDs to a specific colour
 */
void LEDRequest::setFootLeds(Side side, const LEDColor &color, LEDState state)
{
  static const int greenOffset = footLeftGreen - footLeftRed;
  static const int blueOffset = footLeftBlue - footLeftRed;
  int firstLed = (side == left) ? footLeftRed : footRightRed;

  setLed(LED(firstLed), color.r, state);
  setLed(LED(firstLed + greenOffset), color.g, state);
  setLed(LED(firstLed + blueOffset), color.b, state);
}

/**
 * set the ear LEDs to a specific value
 *
 * Note that the ear LEDs are blue only so they only have a brightness, no colour.
 * The brightness values are pretty difficult to distinguish so recommend
 * just on or off
 */
void LEDRequest::setEarLeds(Side side, unsigned indexOrMask, float value, LEDState state)
{
  int firstLed = (side == left) ? earsLeft0Deg : earsRight0Deg;

  if (indexOrMask & MASK) // is it a bit mask for the LEDs?
  {
    for (unsigned i=0; i < numOfEarLeds; i++)
      if (indexOrMask & (0x1 << i))
        setLed(LED(firstLed + i), value, state);
  }
  else
  {
    ASSERT(indexOrMask < numOfEarLeds);
    setLed(LED(firstLed + indexOrMask), value, state);
  }

}

/**
 * set the head LEDs to a specific value
 *
 * Note that the head LEDs are white only so they only have a brightness, no colour.
 */
void LEDRequest::setHeadLeds(unsigned indexOrMask, float value, LEDState state)
{
  if (indexOrMask & MASK) // is it a bit mask for the LEDs?
  {
    for (unsigned i=0; i < numOfHeadLeds; i++)
      if (indexOrMask & (0x1 << i))
        setLed(LED(firstHeadLED + i), value, state);
  }
  else
  {
    ASSERT(indexOrMask < numOfHeadLeds);
    setLed(LED(firstHeadLED + indexOrMask), value, state);
  }
}
