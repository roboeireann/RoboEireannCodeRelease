/**
 * @file SmokeTestIntegration.cpp
 *
 * Minimal tests to ensure that CatchTests is able to access header files
 * and link with the appropriate libraries/objects to access code in
 * the robot codebase
 *
 * @author Rudi Villing
 */

#include "Libs/Math/Angle.h"
#include "Representations/Sensing/FallDownState.h" 

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"



TEST_CASE("RobotCore Integration Smoke Test", "[smoke_tests][core]") 
{
    
  SECTION("Verify Core Math")  // it only uses headers however, so doesn't prove linking is good
  {
    Angle someAngle = 180_deg;

    REQUIRE_THAT(someAngle, Catch::Matchers::WithinAbs(3.14159f, 0.00001f));
    REQUIRE_THAT(someAngle.toDegrees(), Catch::Matchers::WithinAbs(180.f, 0.0001f));
  }

  SECTION("Link test using FallDownState")
  {
    FallDownState fallDownState;

    fallDownState.state = FallDownState::fallen;
    fallDownState.predictedCom = Vector3f::UnitX();

    REQUIRE(fallDownState.state == FallDownState::fallen);
    REQUIRE(fallDownState.predictedCom.x() == 1.f);
    REQUIRE(fallDownState.predictedCom.y() == 0.f);
  }
}