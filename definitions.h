//IR Portion
#define rightIR A5 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define leftIR A0

//Ultrasonic
const int trigPin = 16;
const int echoPin = 18;

//Encoder Portion
int encoderLeft = 14;
int encoderRight = 15;

//IR Distances
long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)

{
  return microseconds / 29 / 2;}



