//Libraries
#include "MSMotorShield.h"

//Motor Declarations
MS_DCMotor motorR(4);
MS_DCMotor motorL(3);

void motorRFor(int Speed) {
  motorR.run(FORWARD);
  motorR.setSpeed(Speed);
}

void motorLFor(int Speed) {
  motorL.run(FORWARD);
  motorL.setSpeed(Speed);
}

void motorRBac(int Speed) {
  motorR.run(BACKWARD);
  motorR.setSpeed(Speed);
}

void motorLBac(int Speed) {
  motorL.run(BACKWARD);
  motorL.setSpeed(Speed);
}

//Movement Functions
void turnLeft()
{
    // Motor Portion
  motorLFor(0);
  motorRFor(150);
  delay(50);
  motorLFor(150);
  motorRFor(150);
}

void turnRight()
{
    // Motor Portion
  motorLFor(150);
  motorRFor(0);
  delay(50);
  motorLFor(150);
  motorRFor(150);
}
void goStraight()
{

  motorLFor(150);
  motorRFor(150);

}

extern
