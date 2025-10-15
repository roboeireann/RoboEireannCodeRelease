/**
 * This file defines the test code for the normalizeDegrees function
 * and Game class
 */

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_all.hpp"

#include "tutorial_src.h"


using namespace Catch::Matchers;


/**
 * based on the catch tutorial
 * 
 * This also demonstrates a very simple test case with no sections
 * 
 * The two parameters to TEST_CASE are the name of the test that will be
 * printed in the output and the test case tag. You can easily run all test cases
 * that have the same tag from the command line. 
 */
TEST_CASE( "Verify various factorials computed correctly", "[factorial]" ) 
{
  REQUIRE( factorial(1) == 1 );
  REQUIRE( factorial(2) == 2 );
  REQUIRE( factorial(3) == 6 );
  REQUIRE( factorial(10) == 3628800 );

  REQUIRE( factorial(0) == 1 );
}






// ###########################################################################
// ###########################################################################
// ###########################################################################



/**
 * demonstrate some simple tests for normalizing angles
 * and the use of sections in test cases
 *
 * Sections are used to group related tests together and to share setup.
 * The TEST_CASE runs from the start for each SECTION and only one SECTION
 * is executed each time. Therefore any common setup code used by all
 * SECTIONs is written at the top of the TEST_CASE, outside all the SECTIONs.
 * (Note: any SECTION-specfic setup should go inside the SECTION itself.)
 */

TEST_CASE("Normalize angles in degrees", "[normalizeDegrees]") 
{
  // since normalizeDegrees is a standalone "pure" function that doesn't
  // depend on any environment setup in advance and doesn't have any side
  // effects, we don't need to do any common setup at the start. 
  // If we did, we would put it here.


  // Sections are used to group related tests together and to share setup.
  // In this case we don't have any shared setup so they are just used
  // to group related tests.
  //
  // If any CHECKS assertion in a section fails, the failure is recorded,
  // but execution of subsequent code in the section proceeds as normal.
  //
  // If any REQUIRES assertion in a section fails, the section is exited
  // immediately. (Other sections will still run however.)
  //
  // Only use CHECKS for things which "should" be true but are not essential
  // for the correctness of subsequent assertions in the section. Mostly
  // you should use REQUIRES.

  SECTION("section exit demo")
  {
    CHECK( 0==1 ); // fails but doesn't exit
    REQUIRE( 0==1 ); // fails and exits the section immediately
    REQUIRE( 2==3 ); // this one never gets to run
  }


  SECTION("zero angles")
  {
    REQUIRE( normalizeDegrees(0.0f) == 0.0f );

    // demonstrate REQUIRE_FALSE 
    // - use REQUIRE_FALSE(something) instead of REQUIRE(!something)
    //
    // normalizeDegrees(0.0f) is equal to 0.0f so the expression that it is not
    // equal will definitely be false (if normalizeDegrees is working properly)
    REQUIRE_FALSE( normalizeDegrees(0.0f) != 0.0f );
  }

  SECTION("angles between -180..180")
  {
    REQUIRE( normalizeDegrees(97.0f) == 97.0f );
    REQUIRE( normalizeDegrees(179.9f) == 179.9f );
    REQUIRE( normalizeDegrees(-180.0f) == -180.0f );

    // NOTE: this test currently fails
    // REQUIRE( normalizeDegrees(180.0f) == 180.0f );
  }

  SECTION("larger positive angles")
  {
    // This one does not work due to a slight difference in the value we 
    // are checking with. This can sometimes occur due to floating point
    // representation and rounding errors as demonstrated here.
    //
    // Note that the CHECK assertion does not exit the section if it fails
    // (unlike REQUIRES).
    CHECK( normalizeDegrees(540.45907f) == -179.54093f );

    // this next line works because we allow some error tolerance.
    // WithinAbs checks that the actual result is with the specified absolute difference
    // from the expected value. The usage is
    // WithinAbs(expectedValue, maxAbsDifferenceToAccept)
    REQUIRE_THAT( normalizeDegrees(540.45907f), WithinAbs(-179.54093f, 0.00005f) );

    // when we don't want to use absolute error values, we can instead specify
    // the maximum number of ULPs by which the actual answer and expected answer
    // can differ. 
    // A ULP (unit of least precision) is the smallest 
    // increase or decrease possible for a given floating point
    // number (change one bit of the binary representation). The usage is
    // WithinULP(expectedValue, maxNumberOfULPsDifferenceToAccept).
    REQUIRE_THAT( normalizeDegrees(540.45907f), WithinULP(-179.54093f, 2) );

    // other values to test
    REQUIRE_THAT( normalizeDegrees(390.0f), WithinULP(30.0f, 2) );
    REQUIRE_THAT( normalizeDegrees(350.0f), WithinULP(-10.0f, 2) );
    REQUIRE_THAT( normalizeDegrees(360.0f), WithinULP(0.0f, 2) );
    REQUIRE_THAT( normalizeDegrees(3603.0f), WithinULP(3.0f, 2) );
  }

  // uncomment the section below to demonstrate some bugs in the existing
  // normalizeDegrees function

  SECTION("larger negative angles")
  {
    REQUIRE_THAT( normalizeDegrees(-390.0f), WithinULP(-30.0f, 0) );
    REQUIRE_THAT( normalizeDegrees(-350.0f), WithinULP(10.0f, 0) );
  }

}





// ###########################################################################
// ###########################################################################
// ###########################################################################

/**
 * test cases for the RobotHelper class
 */

TEST_CASE("RobotHelper", "[RobotHelper]")
{
  // common setup
  RobotHelper robotHelper;

  SECTION("Not striker")
  {
    REQUIRE_FALSE( robotHelper.isStriker() ); // verify that the default setting is not the striker

    SECTION("no walk or kick with distant ball and target")
    {
      robotHelper.setBallDistance(2000.0f);
      robotHelper.setTargetDistance(5000.0f);

      REQUIRE_THAT( robotHelper.getWalkSpeed(), WithinULP(0.0f, 0) ); // no walk
      REQUIRE_THAT( robotHelper.getKickStrength(), WithinULP(0.0f, 0) ); // no kick
    }

    SECTION("no kick with close ball and distant target")
    {
      robotHelper.setBallDistance(150.0f); // ball is at our feet
      robotHelper.setTargetDistance(5000.0f);

      REQUIRE_THAT( robotHelper.getWalkSpeed(), WithinULP(0.0f, 0) ); // no walk
      REQUIRE_THAT( robotHelper.getKickStrength(), WithinULP(0.0f, 0) ); // no kick
    }

  }

  SECTION("Is striker")
  {
    robotHelper.setStriker(true);

    SECTION("walk and kick to max for distant ball and target")
    {
      robotHelper.setBallDistance(2000.0f);
      robotHelper.setTargetDistance(5000.0f);

      REQUIRE_THAT( robotHelper.getWalkSpeed(), WithinULP(1.0f, 0) ); // max walk
      REQUIRE_THAT( robotHelper.getKickStrength(), WithinULP(1.0f, 0) ); // max kick
    }

    SECTION("walk to min and kick to max for close ball and distant target")
    {
      robotHelper.setBallDistance(150.0f);
      robotHelper.setTargetDistance(5000.0f);

      REQUIRE_THAT( robotHelper.getWalkSpeed(), WithinULP(0.0f, 0) ); // min walk
      REQUIRE_THAT( robotHelper.getKickStrength(), WithinULP(1.0f, 0) ); // max kick
    }

    SECTION("linear interpolate walk speed")
    {
      robotHelper.setBallDistance(225.0f); // half way between 150 and 300 (see spec)
      robotHelper.setTargetDistance(5000.0f);

      REQUIRE_THAT( robotHelper.getWalkSpeed(), WithinULP(0.5f, 0) ); // no walk
      REQUIRE_THAT( robotHelper.getKickStrength(), WithinULP(1.0f, 0) ); // max kick
    }

    // TODO: add other tests here
    // E.g. make sure that linear interpolation of kick distances is correct
    // Make sure that kick is set to minimum when appropriate, etc.
  }
}











// ###########################################################################
// ###########################################################################
// ###########################################################################


/**
 * demonstrate a test case for Game FSM, including the use of a
 * common initialization section, and the need to set the initial
 * conditions, then the condition under test, then check the output or
 * side effects
 */

TEST_CASE("Game FSM", "[.][game]") 
{
  // common setup goes here - it will run before each section

  Game game; // initialize a game object in the IDLE state

  REQUIRE( game.getState() == Game::IDLE ); // confirm it is in the right state

  SECTION("transitions from IDLE")
  {
    // game is already in IDLE (checked above)

    SECTION("IDLE -> READY")
    {
      // provide test setup/inputs by calling setState
      game.setState(Game::READY);
      // verify outputs/side-effects by calling getState to check the game
      // has changed to the expected state
      REQUIRE(game.getState() == Game::READY);
    }

    SECTION("IDLE -> PLAYING")
    {
      game.setState(Game::READY);
      REQUIRE(game.getState() == Game::READY);
    }

    SECTION("IDLE -> others")
    {
      game.setState(Game::SET);
      REQUIRE_FALSE( game.getState() == Game::SET ); // it must not have changed to SET
      REQUIRE( game.getState() == Game::IDLE ); // in fact it must still be IDLE
    }
  }

  SECTION("transitions from READY")
  {
    // game is initially in IDLE so switch to ready state and confirm it
    game.setState(Game::READY);
    REQUIRE( game.getState() == Game::READY );

    SECTION("READY -> SET")
    {
      game.setState(Game::SET);
      REQUIRE(game.getState() == Game::SET);
    }
  }

}
