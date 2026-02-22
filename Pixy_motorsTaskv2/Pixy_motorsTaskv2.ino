#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

DualMAX14870MotorShield Motors;
Pixy2 pixy;

//**
// Ultrasonic sensor code
int signal=26;
int distance;
int gap = 10;
unsigned long pulseduration=0;
//**


/*
Motor1: LEFT
Motor2: RIGHT
*/

// Encoder pins (consistent naming)
const int Motor1EPinA = 2;
const int Motor1EPinB = 3;

const int Motor2EPinA = 18;
const int Motor2EPinB = 19;

volatile long counter1 = 0;
volatile long counter2 = 0;

// Motion constants
float COUNTS_PER_REV = 12.0 * 4.0 * 20.0;
float TURN_CIRC = 100;       // MEASURE THIS //180 for now
float WHEEL_CIRC_CM = 16;    // MEASURE THIS
float FULL_ROTATION_COUNTS = (TURN_CIRC / WHEEL_CIRC_CM) * COUNTS_PER_REV;

int TURN_SPEED = 50;
int STRAIGHT_SPEED = 100;

int CORRECTION_COUNT = 500;

enum States
{
  initialize,
  main,
  correctLeft,
  correctRight,
  left,
  right,
  back
};

States myState = initialize;

void setup()
{

//** ultrasonic sensor

 pinMode(signal, OUTPUT);

//**

  Motors.enableDrivers();

  Serial.begin(115200);
  pixy.init();
  Serial.println("Pixy Initialized...");

  pinMode(Motor1EPinA, INPUT_PULLUP);
  pinMode(Motor1EPinB, INPUT_PULLUP);
  pinMode(Motor2EPinA, INPUT_PULLUP);
  pinMode(Motor2EPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Motor1EPinA), changeUp1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor1EPinB), changeUp1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2EPinA), changeUp2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Motor2EPinB), changeUp2B, CHANGE);
}

void loop()
{
//**
//Get the raw distance measurement value
measureDistance();
 distance = (pulseduration / 2) * 0.0343;

// Display on serial monitor
 Serial.print("Distance - ");
 Serial.print(distance);
 Serial.println(" cm");
 delay(500);
//**

pixy.ccc.getBlocks();

  switch (myState)
  {
    case initialize:
      myState = main;
      break;

    case main:
      Motors.setSpeeds(STRAIGHT_SPEED, STRAIGHT_SPEED);
      Serial.println("spinning...");
      if (pixy.ccc.numBlocks)
      {
        for (int i = 0; i < pixy.ccc.numBlocks; i++)
        {
          int sig = pixy.ccc.blocks[i].m_signature;

          Serial.print("Detected Signature: ");
          Serial.println(sig);

// CASE 1
          if (sig == 1)
          {
            Serial.println("Green - 180 Turn");
            myState = back;
            if (distance<=gap){
// Do this, stop motors, do 180 degrees turn
// **********
  Motors.setSpeeds(0, 0);   
  delay(200);               // small pause for stability
  myState = back;           // perform 180 turn
  // **********

            }

            break;
          }

// CASE 2
          else if (sig == 2)
          {
            Serial.println("Blue - Turn Right");
            myState = right;
if (distance<=gap){
// Do this, stop motors, turn right
// **********
  Motors.setSpeeds(0, 0);   
  delay(200);               //stability
  myState = right;           // turn right
  // **********

            }

            break;
          }

// CASE 3
          else if (sig == 3)
          {
            Serial.println("Red - Turn Left");
            myState = left;
if (distance<=gap){
// stop motors, turn left
// **********
  Motors.setSpeeds(0, 0);   
  delay(200);               // small pause for stability
  myState = back;           // perform left turn
  // **********

            }

            break;
          }
        }
      }
      break;



    case correctLeft:
      Serial.println("left wall too close - moving right");

      while (abs(counter1) < CORRECTION_COUNT)
      {
        Motors.setSpeeds(TURN_SPEED, 0);
        Serial.println(counter1);
      }      

    case correctRight:
      Serial.println("right wall too close - moving left");

      while (abs(counter2) < CORRECTION_COUNT)
      {
        Motors.setSpeeds(0, TURN_SPEED);
        Serial.println(counter2);
      }    


    case left:
      Serial.println("turning left");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter1) < 0.5 * FULL_ROTATION_COUNTS)
      {
        Motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);   // STOP
      myState = main;
      break;

    
    
    case right:
      Serial.println("turning right");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter1) < 0.5 * FULL_ROTATION_COUNTS)
      {
        Motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);   // STOP
      myState = main;
      break;

    
    
    case back:
      Serial.println("turning around");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter1) < FULL_ROTATION_COUNTS)
      {
        Motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);   // STOP
      myState = main;
      break;

    default:
      Serial.println("Error!");
      break;
    
  }
}

// ======================
// INTERRUPT ROUTINES
// ======================

void changeUp1A()
{
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB))
    counter1++;
  else
    counter1--;
}

void changeUp1B()
{
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB))
    counter1--;
  else
    counter1++;
}

void changeUp2A()
{
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB))
    counter2++;
  else
    counter2--;
}

void changeUp2B()
{
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB))
    counter2--;
  else
    counter2++;
}

//**
// ultrasonic sensor distance measuring function
void measureDistance()
{
 // set pin as output so we can send a pulse
 pinMode(signal, OUTPUT);
// set output to LOW
 digitalWrite(signal, LOW);
 delayMicroseconds(5);
 
 // now send the 5uS pulse out to activate Ping)))
 digitalWrite(signal, HIGH);
 delayMicroseconds(5);
 digitalWrite(signal, LOW);
 
 // now we need to change the digital pin
 // to input to read the incoming pulse
 pinMode(signal, INPUT);
 
 // finally, measure the length of the incoming pulse
 pulseduration = pulseIn(signal, HIGH);
}

//**