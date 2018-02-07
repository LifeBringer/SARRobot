//All Libraries
#include <MSMotorSheild.h>
#include <PID_v1.h>
//To be removed
#include <TestCodeMotorShield.h>

//All Parameters
#include <definitions.h>

// Setup Routine
void setup() {
  Serial.begin(9600);  
  pinMode(encoder1, INPUT);  
  pinMode(encoder2, INPUT);
}

void loop() {
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
  
  // Motor Portion
  motor(3, FORWARD, 255);  
  motor(4, FORWARD, 255);
  
  //Encoder Portion
  // read the input pin:  
  int buttonState = digitalRead(encoder1);  
  int buttonState2 = digitalRead(encoder2);  
  // print out the state of the button:  
  Serial.print(buttonState);  
  Serial.println(buttonState2);  
  delay(1);       
  // delay in between reads for stability
  
  //IR portion
    // 5v
  float volts1 = analogRead(sensor1)*0.0048828125;  // value from sensor * (5/1024)
  float volts2 = analogRead(sensor2)*0.0048828125;  // value from sensor * (5/1024)
  int distance1 = 13*pow(volts1, -1); // worked out from datasheet graph
  int distance2 = 13*pow(volts2, -1); // worked out from datasheet graph
  delay(10); // slow down serial port 
  Serial.print(volts1);
  Serial.println(distance2);
 
}