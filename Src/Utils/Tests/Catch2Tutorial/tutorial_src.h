/**
 * this file declares the units (functions and classes) to be tested in our catch tutorial
 */



/**
 * simple function to return the factorial of a number
 * (Based on the official catch2 tutorial.)
 */
unsigned int factorial( unsigned int number );




/**
 * normalize an angle to -180 (incl) ..180 (excl)
 * @param deg - input angle in degrees, can be positive to negative
 * @returns normalized angle which must be positive
 */
float normalizeDegrees(float deg);




/**
 * A class which implements a hugely simplified robot soccer robot
 * and the following specifications (which are not normal for robot soccer):
 * 
 * - the robot should not walk (i.e. speed=0) or kick (i.e. strength=0)
 *   unless it is the striker
 * 
 * - the robot should only walk if the ball distance is more than 150 mm
 *   and should only kick otherwise
 * 
 * - the robot's walk speed is 1.0 unless the ballDistance is less than 300 mm.
 *   From there the speed decreases linearly to 0.0 at 150 mm.
 * 
 * - the robot's kick strength varies linearly between 0.0 when targetDistance is 0
 *   up to 1.0 when targetDistance is 3000 mm or more
 */
class RobotHelper
{
public:
  bool isStriker() { return striker; }
  void setStriker(bool val); // see CPP file

  float getBallDistance() { return ballDistance; }
  void setBallDistance(float val); // see CPP file

  float getTargetDistance() { return targetDistance; }
  void setTargetDistance(float val); // see CPP file

  float getWalkSpeed(); // see CPP file
  float getKickStrength(); // see CPP file

private:
  bool striker = false;
  float ballDistance = 0.f;
  float targetDistance = 0.f;

  bool needsUpdate = false;

  float walkSpeed = 0.f;
  float kickStrength = 0.f;

  void updateWalkAndKick(); // see CPP file
};





/**
 * a class which implements a simple state machine.
 * We use this to demonstrate the concepts around setting up objects,
 * inputting data via one function and checking the result or side-effect
 * via another
 */
class Game
{
public:
  enum State { IDLE, READY, SET, PLAYING };

private:
  State state;

public:
  Game() : state(IDLE) {}

  /// return the current state
  State getState() { return state; }

  /**
   * change the current state.
   * If the new state is not allowed it simply won't be set.
   * The only way to check this is by checking what state is active
   * after calling setState
   * 
   * The permitted state transitions are as follows
   * 
   * +-------------------------------------------+
   * |                                           |
   * +->IDLE --> READY ---> SET ----> PLAYING -->+
   *     |        |                    ^  ^
   *     |        +--------------------+  |
   *     +--------------------------------+
   */
  void setState(State newState);
};
