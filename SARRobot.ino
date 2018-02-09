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
float RIGHTir = analogRead(rightIR)*0.0048828125;  // value from sensor * (5/1024)
  float LEFTir = analogRead(leftIR)*0.0048828125;  // value from sensor * (5/1024)
  Serial.print(RIGHTir);
  Serial.print("\t");
  Serial.println(LEFTir);
  if (2<RIGHTir && 2<LEFTir)
  {
    goStraight();
  }
    else if (LEFTir<1.5 && RIGHTir>1.5)
    {
     turnLeft();

    }
    else if (RIGHTir<1.5 && LEFTir>1.5)
    {
     turnRight();
    
    }
    else if (RIGHTir<1 && LEFTir<1)
    {     turnLeft();    
    
  }
  
}



void turnLeft()
{
    // Motor Portion
  motor(3, FORWARD, 0);
  motor(4, FORWARD, 150);
  delay(50);
  motor(3, FORWARD, 150);
  motor(4, FORWARD, 150);
}

void turnRight()
{
    // Motor Portion
  motor(3, FORWARD, 150);
  motor(4, FORWARD, 0);
  delay(50);
  motor(3, FORWARD, 150);
  motor(4, FORWARD, 150);
}
void goStraight()
{

  motor(3, FORWARD, 150);
  motor(4, FORWARD, 150);

}
