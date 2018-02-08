//All Libraries
#include "MSMotorShield.h"
#include "PID_v1.h"
//To be removed
//#include "motorShield.h"

//All Parameters
#include "definitions.h"
const int trigPin = 16;
const int echoPin = 18;
// Setup Routine
void setup() {
  Serial.begin(9600);
  pinMode(encoder1, INPUT);  
  pinMode(encoder2, INPUT);
  
}

void loop() {

  //findMiner();
  //pickupMiner();
  //returnHome();

 
// Suppose there is a relay, or light or solenoid  
// connected to M3_A and GND. 
// Note that the 'speed' (the PWM, the intensity)   
// is for both M3_A and M3_B.  
// The output is a push-pull output (half bridge),   
// so it can also be used to drive something low.  
// The 'speed' (the PWM, the intensity) can be set   
// to zero, that would make the output disabled   
// and floating.   
// Suppose a DC motor is connected to M1_A(+) and M1_B(-)
// Let it run full speed forward and half speed backward.  
// If 'BRAKE' or 'RELEASE' is used, the 'speed' parameter  
// is ignored.
  

 /* 
  //Encoder Portion
  // read the input pin:  
  int buttonState = digitalRead(encoder1);  
  int buttonState2 = digitalRead(encoder2);  
  // print out the state of the button:  
  Serial.print(buttonState);  
  Serial.println(buttonState2);  
  delay(1);       
  // delay in between reads for stability
  */
  //IR portion
    // 5v
 /* float volts1 = analogRead(rightIR)*0.0048828125;  // value from sensor * (5/1024)
  float volts2 = analogRead(leftIR)*0.0048828125;  // value from sensor * (5/1024)
  int distance1 = 13*pow(volts1, -1); // worked out from datasheet graph
  int distance2 = 13*pow(volts2, -1); // worked out from datasheet graph
  delay(10); // slow down serial port 
  Serial.print(volts1);
  Serial.print("\t");
  Serial.println(volts2);*/

long duration, inches, cm;
pinMode(trigPin, OUTPUT);
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
pinMode(echoPin, INPUT);
duration = pulseIn(echoPin, HIGH);
inches = microsecondsToInches(duration);
cm = microsecondsToCentimeters(duration);
Serial.print(inches);
Serial.print("in, ");
Serial.print(cm);
Serial.print("cm");
Serial.println();
delay(100);
}

long microsecondsToInches(long microseconds)
{
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)

{
  return microseconds / 29 / 2;}


void turnLeft()
{
    // Motor Portion
  //motor_output(3, FORWARD, 0);
  //motor(4, FORWARD, 150);
}

void turnRight()
{
    // Motor Portion
  //motor(3, FORWARD, 150);
  //motor(4, FORWARD, 0);
}
