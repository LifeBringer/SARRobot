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
  Serial.println("Simple Motor Shield sketch");
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
/*motor_output(MOTOR3_A, HIGH, 255);  
delay(2000);  
motor_output(MOTOR3_A, LOW, 255);*/
  // Suppose a DC motor is connected to M1_A(+) and M1_B(-)  // Let it run full speed forward and half speed backward.  // If 'BRAKE' or 'RELEASE' is used, the 'speed' parameter  // is ignored.  motor(3, FORWARD, 255);  motor(4, FORWARD, 255);
}
