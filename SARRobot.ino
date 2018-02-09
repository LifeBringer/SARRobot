//All Libraries
#include "PID_v1.h"
#include "motoMoves.h"
//To be removed
//#include "motorShield.h"

//All Parameters
#include "definitions.h"

// Setup Routine
void setup() {
  Serial.begin(9600);
  pinMode(encoderLeft, INPUT);  
  pinMode(encoderRight, INPUT);
  
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
