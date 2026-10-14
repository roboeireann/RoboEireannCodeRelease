/**
 * GamePadExProvider.cpp
 *
 * @author Rudi Villing
 */

#include "GamePadExProvider.h"
#include "TextLogging/TextLogging.h"

MAKE_MODULE(GamePadExProvider);

DECL_TLOGGER(tlogger, "GamePadExProvider", TextLogging::INFO);

void GamePadExProvider::update(GamePadEx& gamePadEx)
{
  gamePadEx.prevButtons = gamePadEx.buttons;
  gamePadEx.prevHat = gamePadEx.hat;
  static_cast<GamePad&>(gamePadEx) = theGamePad;

  if (gamePadEx.hatChanged() || gamePadEx.buttonsChanged())
  {
    TLOGD(tlogger,
          "left=({:.2f},{:.2f}), right=({:.2f},{:.2f}), hat={}, "
          "abxy={:d}{:d}{:d}{:d}, lbrb={:d}{:d}, ltrt={:d}{:d} lsrs={:d}{:d}, back/start={:d}{:d}",
          gamePadEx.lStickX, gamePadEx.lStickY, gamePadEx.rStickX, gamePadEx.rStickY,
          TypeRegistry::getEnumName(gamePadEx.hat),

          gamePadEx.pressed(GamePad::a), gamePadEx.pressed(GamePad::b), gamePadEx.pressed(GamePad::x),
          gamePadEx.pressed(GamePad::y), gamePadEx.pressed(GamePad::lb), gamePadEx.pressed(GamePad::rb),
          gamePadEx.pressed(GamePad::lt), gamePadEx.pressed(GamePad::rt), gamePadEx.pressed(GamePad::ls),
          gamePadEx.pressed(GamePad::rs), gamePadEx.pressed(GamePad::back), gamePadEx.pressed(GamePad::start));


    // TLOGD(tlogger, "Just pressed: abxy={:d}{:d}{:d}{:d}, back/start={:d}{:d}", gamePadEx.justPressed(GamePad::maskA),
    //       gamePadEx.justPressed(GamePad::maskB), gamePadEx.justPressed(GamePad::maskX),
    //       gamePadEx.justPressed(GamePad::maskY), gamePadEx.justPressed(GamePad::back),
    //       gamePadEx.justPressed(GamePad::start));
  }
}