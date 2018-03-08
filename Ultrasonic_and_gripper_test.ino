#include "PID_v1.h"
#include "TimerOne.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>

/////// MINER PARAMETERS //////
bool minerRetrieved = false;
float islandReached = 100000000; /// Distance to switch wall follower in mm
float exitReached = 100000000; /// Distance on the way back to switch wall follower

/////// MOVEMENT PARAMETERS ///// START

//Motor Pin Values
Adafruit_MotorShield motorS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = motorS.getMotor(1);
Adafruit_DCMotor *rightMotor = motorS.getMotor(2);
Adafruit_StepperMotor *stepperMotor = motorS.getStepper(200, 3);

//
// int E1 = 3;
// int M1 = 12;
// int E2 = 11;
// int M2 = 13;

/////// PID ///// START
//Global Variables for Angle Calculation
float opposite;
float S; // Distance from Wall
float D;
const float adjacent = 10.820; //Distance between IR sensors used to calc. robo. angle relative to wall
float theta;

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double angularSetpnt, angularInput, angularOutput;

//Specify the links and initial tuning parameters
PID dispPID(&Input, &Output, &Setpoint, 115, 0, 0, DIRECT);
PID angularPID(&angularInput, &angularOutput, &angularSetpnt, 10, 0, 0, DIRECT);

//Ultrasonic SENSOR Parameters
int trigPin = 9;
int echoPin = 10;
long duration;
int distance;
int ultrasonic;

//IR SENSOR Parameters
int forwardLeftSensor = 16;// Analog pin 2
int backLeftSensor = 5;
int forwardRightSensor = 14; // Analog pin 0
int backRightSensor = 1;
int frontSensor = 15; //Analog pin 1
float rearWallDistance; //Distance input from dorsal IR sensor (rear)
float forwardWallDistance; //Distance input from lateral IR sensor (front)
float frontDistance;
int speedRight;
int speedLeft;
bool Right; //If true then we use the right IR sensors
bool Left; //If true then we use the left IR sensors

/////// PID PARAMETERS ///// END
/////// MOVEMENT PARAMETERS ///// END



/////// ODOMETRY PARAMETERS ///// START

// Encoder Parameters
#define RH_ENCODER_A 2
#define RH_ENCODER_B 8
#define LH_ENCODER_A 3
#define LH_ENCODER_B 9

// variables to store the number of encoder pulses
// for each motor according to arduino the volatile identifier is to be used with interrupts
volatile long leftCount = 0;
volatile long rightCount = 0;
volatile long PrevLeftCount = 0;
volatile long PrevRightCount = 0;

//Odometry Calculations
const int pulses = 2940; //Pulses per 180 degrees
float degreePerPulse = 0.0613; //Degree per a pulse from encoder
const float mmPerPulse = 0.09243; //mm per pulse = pi*86.5 (diameter)./2940
float thetaPast = 0;
float thetaPresent = 0;
float thetaAvg = 0; //Average Theta between past and present yaw
float sO = 0; //Linear Translation
float x = 0; // Total distance of x
float y = 0; // Total distance of y (side to side shifting, should be ~0)
float SR = 0; //Translation of Right Motor
float SL = 0; //Translation of Left Motor
const float b = 104; //Wheel Base
/////// ODOMETRY PARAMETERS ///// END

////////////////    NOW THE GOOD STUFF    /////////////////////

////////////////    SETUP    /////////////////////
void setup() {

  /// Warming up...
  //Timer Interrupt
  Timer1.initialize(10000);
  Timer1.attachInterrupt(odometryCalc);

  /// Start your senses...
  //Encoder
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  // initialize hardware interrupts
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  //Ultrasonic Sensor
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  /// Balance yourself...
  //initialize the PID parameters
  Setpoint = 0;
  angularSetpnt = 3.5;
  dispPID.SetOutputLimits(-255, 255); //tell the PID to range the output from -255 to 255
  angularPID.SetOutputLimits(-255, 255);
  //Turn the PID on
  dispPID.SetMode(AUTOMATIC);
  angularPID.SetMode(AUTOMATIC);

  /// Start your engines...
  motorS.begin(); //Motors start with the default 1.6kHz
  rightMotor->run(RELEASE);
  leftMotor->run(RELEASE);
  servoMotor->setSpeed(10);  // 10 rpm 
  servoMotor->step(100, BACKWARD, SINGLE);
  // pinMode(M1, OUTPUT);
  // pinMode(M2, OUTPUT);
  // pinMode (M3 & M4, OUTPUT);

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}


//////////////////   LOOP   ////////////////////////
void loop()
{
  delay(100);

  mazeSolver(); // solves the maze buddy! Logic to switch from left to right wall following

  getIR();

  calculatePID();

  howToHandleThoseCurvesnEdges(); //Deals with dead ends and turns.
  
  ultraSonic();

}


////////////////    NOW THE BAD ASS STUFF    /////////////////////

/////////  HIGH LEVEL METHODS  /////////////
void ultraSonic()
{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
// Sets the trigPin on HIGH state for 10 micro seconds
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
// Reads the echoPin, returns the sound wave travel time in microseconds
duration = pulseIn(echoPin, HIGH);
// Calculating the distance
distance= duration*0.034/2;
ultrasonic = distance; 
}

void areYouMiner()
{
  if (minerRetrieved = true)
  {
   return 0 ;
   {
   else if (ultrasonic >6 && frontSensor<=3.3)
   {
    delay(100);
    grabMiner();
   }
    
   }
  }
  // Bryan
  // if minerRetrieved = true then return 0
  // else if ultrasonic > 6 cm then Stop(); and grabMiner();
  // else: no else!
  //
}

void mazeSolver()
{
  leftFollower();
  if (x > islandReached) {
    rightFollower();
  }
  else if (x > exitReached) {
    leftFollower();
  }
}

void rightFollower() {
  Right = true;
  Left = false;
}

void leftFollower() {
  Right = false;
  Left = true;
}

void howToHandleThoseCurvesnEdges() {
  // What to do when you hit a wall
  if ((frontDistance <= 6 && forwardWallDistance <= 6) && Left)
  {
    Stop();
    areYouMiner(); // Did I find the one?
    int lTest;
    while (lTest <= 5000) { ///stuck at a wall turning value
      RightTurn();
      lTest++;
    }
    lTest = 0;
  }
  else if ((frontDistance <= 6 && forwardWallDistance <= 6) && Right)
  {
    Stop();
    areYouMiner(); // Did I find the one?
    int rTest;
    while (rTest <= 5000) { ///stuck at a wall turning value
      LeftTurn();
      rTest++;
    }
    rTest = 0;
  }
  else if (frontDistance <= 6 && Right) {
    Stop();
    LeftTurn();
  }
  else if (frontDistance > 6) {
    Forward();
  }
  else if ((Right && Left) == false)
  {
    //Stop(); // How the hell did you get here?
  }
  else
    Forward();
}

void calculatePID() {
  if (rearWallDistance >= 30 || forwardWallDistance >= 30) {
    Input = 30 ;
    //Graphing Purposes
    forwardWallDistance = 30;
    rearWallDistance = 30;
  }
  //Offset between two IR sensor due to mounting
  forwardWallDistance = forwardWallDistance - 1;
  angle(); //Determine Angle of Machine relative to the wall
  displacementFromWall(theta); //Determine rearWallDistance of the machine relative to the wall
  //PID Inputs
  angularInput = (D);
  Input = (theta);
  dispPID.Compute();
  angularPID.Compute();
}

/////////  LOW LEVEL METHODS  /////////////


/////////  Actuation  /////////////
void grabMiner() 
{
  if (frontSensor <2 && ultrasonic>6)
  {
     stepperMotor->step(100, FORWARD, SINGLE); 
      minerRetrived = true;
  }
     
  
  // Bryan
  // loop forward until frontSensor < 2 cm
  // Put stepper code here
  // minerRetrieved = true
}

/// FOR PID ///
void displacementFromWall(float angle)
{
  S = (forwardWallDistance + rearWallDistance) / 2 ; //Average of sensor 1 and sensor 2
  D = S * cos(theta); //Distance from wall
}
void angle()
{
  opposite = forwardWallDistance - rearWallDistance;
  theta = atan(opposite / adjacent);
  //Serial.println(theta);
}
//Used when the right wall follower is implemented. When the machine encounters a wall in the front
//and a wall the the right the right wall follower will simply stop. Therefore the machine will have to make a left turn while still
//in the right wall follower mode to continue following the wall.
// ___________________
// |
// | ----------->
// | | ___________
// | | |
// | | |
void RightTurn() // Continuously Turns Right Never Called
{
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  // digitalWrite(M1, HIGH);
  // digitalWrite(M2, HIGH);
  speedRight = (100);
  speedLeft = (100);
  leftMotor->setSpeed(speedLeft);
  rightMotor->setSpeed(speedRight);
  // analogWrite(E2, speedRight);
  // analogWrite(E1, speedLeft);
}
void LeftTurn() // Continuously Turns Left Never Called
{
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  // digitalWrite(M1, HIGH);
  // digitalWrite(M2, HIGH);
  speedRight = (100);
  speedLeft = (100);
  leftMotor->setSpeed(speedLeft);
  rightMotor->setSpeed(speedRight);
  // analogWrite(E2, speedRight);
  // analogWrite(E1, speedLeft);
}

//Stop Function
void Stop()
{
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  // digitalWrite(M1, LOW);
  // digitalWrite(M2, HIGH);
  speedRight = (0);
  speedLeft = (0);
  leftMotor->setSpeed(speedLeft);
  rightMotor->setSpeed(speedRight);
  // analogWrite(E1, speedLeft);
  // analogWrite(E2, speedRight);
}

//Forward Function
void Forward()
{
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
  // digitalWrite(M1, LOW);
  // digitalWrite(M2, HIGH);
  if (Right == true) {
    speedRight = (65 - Output + angularOutput);
    speedLeft = (65 + Output - angularOutput);
  }
  else if (Left == true) {
    speedRight = (65 + Output - angularOutput );
    speedLeft = (65 - Output + angularOutput);
  }
  else {
    speedRight = 0;
    speedLeft = 0;
  }
  //speedRight = (55 -Output);
  //speedLeft = (55 + Output);
  //To Prevent Negitive Speed Values Since they screw up motor
  if (speedRight <= 0)
  {
    speedRight = 0;
  }
  else if (speedLeft <= 0)
  {
    speedLeft = 0;
  }
  leftMotor->setSpeed(speedLeft);
  rightMotor->setSpeed(speedRight);
  // analogWrite(E1, speedLeft); //PWM Speed Control Right
  // analogWrite(E2, speedRight); //PWM Speed Control Left
}


/////////  Sensing  /////////////

void getIR()
{
  ///////////Front IR Sensor
  float voltsFront = analogRead(frontSensor) * 0.0048828125;
  frontDistance = 12 * pow(voltsFront, -1);
  // From the Left Side of Machine
  if (Left == true) {
    float centiLeftForward = analogRead(forwardLeftSensor) * 0.0048828125;
    forwardWallDistance = 12 * pow(centiLeftForward, -1);
    float centiLeftRear = analogRead(backLeftSensor) * 0.0048828125;
    rearWallDistance = 12 * pow(centiLeftRear, -1);
  }
  // From the Right Side of Machine
  if (Right == true) {
    float centiRightForward = analogRead(forwardRightSensor) * 0.0048828125;
    forwardWallDistance = 12 * pow(centiRightForward, -1);
    float centiRightRear = analogRead(backRightSensor) * 0.0048828125;
    rearWallDistance = 12 * pow(centiRightRear, -1);
  }
}

////////////////////  Odometry Calculation   ////////////////////////
void odometryCalc(void)
{
  //Right and Left Wheel Translation
  SR = (rightCount - PrevRightCount) * mmPerPulse;
  SL = (leftCount - PrevLeftCount) * mmPerPulse;
  PrevRightCount = rightCount;
  PrevLeftCount = leftCount;
  //Average Linear Displacement in mm
  sO = (SR + SL) / 2;
  //Present Yaw Calculation
  thetaPresent = ((SR - SL) / b) + thetaPast;

  //ThetaAvg
  thetaAvg = (thetaPresent + thetaPast) / 2;
  //Update Theta Past
  thetaPast = thetaPresent;
  //X value and Y value
  x = x + sO * cos(thetaAvg); // Total Forward Distance
  y = y + sO * sin(thetaAvg); // Total Side-to-Side Distance
}

//////////////////////////////encoder event for the interrupt call////////////////
void leftEncoderEvent() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount++;
    } else {
      leftCount--;
    }
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) {
      leftCount--;
    } else {
      leftCount++;
    }
  }
}
// encoder event for the interrupt call
void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  }
}
