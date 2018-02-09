//IR Portion
#define rightIR A5 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
#define leftIR A0

//Encoder Portion
//int led=3;
int encoder1 = 14;
int encoder2 = 15;

long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)

{
  return microseconds / 29 / 2;}

