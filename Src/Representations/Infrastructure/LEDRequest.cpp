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
const LEDColor LEDColor::PURPLE = LEDColor(0.5f, 0.f, 1.f);
const LEDColor LEDColor::MAGENTA = LEDColor(1.f, 0.f, 1.f);
const LEDColor LEDColor::ORANGE_YELLOW = LEDColor(1.f, 0.5f, 0.f);
const LEDColor LEDColor::BRIGHT_YELLOW = LEDColor(1.f, 1.0f, 0.f);
const LEDColor LEDColor::CYAN = LEDColor(0.f, 0.5f, 1.f);
const LEDColor LEDColor::BROWN = LEDColor(0.3f, 0.15f, 0.f);
const LEDColor LEDColor::GRAY = LEDColor(0.3f, 0.3f, 0.3f);
const LEDColor LEDColor::NONE = LEDColor(0.f, 0.f, 0.f);