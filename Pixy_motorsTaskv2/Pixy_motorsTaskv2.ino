#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>

DualMAX14870MotorShield Motors;
Pixy2 pixy;


// Ultrasonic sensor code
int signal = 26;
int distance;
int gap = 23;
unsigned long pulseduration = 0;
//**

//ir sensors
#define irLeft A8
#define irRight A9
const float DISTANCE_CUTOFF = 2.25;


/*
Motor1: RIGHT
Motor2: LEFT
*/

// Encoder pins (consistent naming)
const int Motor1EPinA = 2;
const int Motor1EPinB = 3;

const int Motor2EPinA = 18;
const int Motor2EPinB = 19;

volatile long counter1 = 0;
volatile long counter2 = 0;

// Motion constants
float COUNTS_PER_REV = 12.0 * 4.0 * 9.8;
float TURN_CIRC = 59.69;
float WHEEL_CIRC_CM = 22;
float FULL_ROTATION_COUNTS = (TURN_CIRC / WHEEL_CIRC_CM) * COUNTS_PER_REV;

int TURN_SPEED = 80;
int STRAIGHT_SPEED = 90;
int CORRECT_SPEED = 85;

int CORRECTION_COUNT = 20;
float correctStartTime = 0;
float correctDelay = 5000;

int SEARCH_COOLDOWN = 200000;
bool isSearching = false;



enum States {
  initialize,
  main,
  correctLeft,
  correctRight,
  left,
  right,
  back,
  search
};

States myState = initialize;

void setup() {

  //** ultrasonic sensor
  delay(5000);
  pinMode(signal, OUTPUT);

  //**

  Motors.enableDrivers();

  Serial.begin(9600);
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

void loop() {
  //**
  //Get the raw distance measurement value
  measureDistance();
  distance = (pulseduration / 2) * 0.0343;

  // Display on serial monitor
  //Serial.println(myState);
  //Serial.print(distance);
  //Serial.println(" cm");
  delay(500);
  //**

  pixy.ccc.getBlocks();

  switch (myState) {
    case initialize:
      myState = main;
      break;

    case main: {

      //if (distance < gap-5) {
        //myState = left;
        //break;
      //}

      //check IR Sensors
      float leftVolts = analogRead(irLeft) * 0.0048828125;  // value from sensor * (5/1024)
      float rightVolts = analogRead(irRight) * 0.0048828125;
      //Serial.print("left ");
      //Serial.println(leftVolts);
      //Serial.print("right ");
      //Serial.println(rightVolts);
      if (leftVolts > DISTANCE_CUTOFF && millis() - correctStartTime > correctDelay) {
        //Serial.print("ABOVE CUT");
        myState = correctRight;
        Serial.print("SET STATE -> ");
        Serial.println(myState);
        break;
      }

      if (rightVolts > DISTANCE_CUTOFF && millis() - correctStartTime > correctDelay) {
        //Serial.print("ABOVE CUT");
        myState = correctLeft;
        break;
      }


      //Move Forwards
      Motors.setSpeeds(STRAIGHT_SPEED, STRAIGHT_SPEED + 1);
      //Serial.println("spinning...");
      //Serial.print("Counter1: " );
      //Serial.println(counter1);
      //Serial.print("Counter2: " );
      //Serial.println(counter2);

      //check Pixy
      if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
          int sig = pixy.ccc.blocks[i].m_signature;

          Serial.print("Detected Signature: ");
          Serial.println(sig);

          // CASE 1
          if (sig == 3) {
            Serial.println("Green - 180 Turn");
            if (distance <= gap) {

              // Do this, stop motors, do 180 degrees turn
              Motors.setSpeeds(0, 0);
              delay(200);
              myState = back;
            }
          break;
          }

          // CASE 2
          else if (sig == 2) {
            Serial.println("Blue - Turn Right");
            if (distance <= gap) {

              // Do this, stop motors, turn right
              Motors.setSpeeds(0, 0);
              delay(200);       //stability
              myState = right;  // turn right
            }
          break;
          }

          // CASE 3
          else if (sig == 1) {
            Serial.println("Red - Turn Left");
            if (distance <= gap) {
              Serial.println("Close!");
              // stop motors, turn left

              Motors.setSpeeds(0, 0);
              delay(200);      // small pause for stability
              myState = left;  // perform left turn
              //Serial.println(myState);
            }
          break;
            
          }
        }
      }
      if (abs(counter1) > SEARCH_COOLDOWN) {
        //myState = search;
        //break;
      }

      break;
    }


    case correctLeft:
      correctStartTime = millis();
      Serial.println("left wall too close - moving right");
      counter1 = 0;
      counter2 = 0;
      while (abs(counter2) < CORRECTION_COUNT && abs(counter1) < CORRECTION_COUNT) {
        Motors.setSpeeds(0, -CORRECT_SPEED);
        delay(100);
        Serial.println(counter1);
      }
      myState = main;
      counter1 = 0;
      counter2 = 0;
      break;

    case correctRight:
      correctStartTime = millis();
      counter1 = 0;
      counter2 = 0;
      Serial.println("right wall too close - moving left");

      while (abs(counter2) < CORRECTION_COUNT && abs(counter1) < CORRECTION_COUNT) {
        Motors.setSpeeds(-CORRECT_SPEED, 0);
        delay(100);
        Serial.println(counter2);
      }

      myState = main;
      counter1 = 0;
      counter2 = 0;
      break;


    case left:
      Serial.println("turning left");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter1) < 0.2 * FULL_ROTATION_COUNTS) {
        Motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);  // STOP
      myState = main;
      counter1 = 0;
      counter2 = 0;
      break;





    case right:
      Serial.println("turning right");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter2) < 0.2 * FULL_ROTATION_COUNTS) {
        Motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);  // STOP

      if (isSearching) {
        myState = search;
        break;
      } else {
        myState = main;
        counter1 = 0;
        counter2 = 0;
        break;
      }



    case back:
      Serial.println("turning around");
      counter1 = 0;
      counter2 = 0;

      while (abs(counter1) < 0.4 * FULL_ROTATION_COUNTS) {
        Motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        //Serial.println(counter1);
      }

      Motors.setSpeeds(0, 0);  // STOP
      myState = main;
      counter1 = 0;
      counter2 = 0;
      break;

    case search:
      {
        Serial.println("searching");
        static int searchCount = 0;

        pixy.ccc.getBlocks();

        if (pixy.ccc.numBlocks) {
          searchCount = 0;
          isSearching = false;

          int sig = pixy.ccc.blocks[0].m_signature;
          myState = main;
          break;
        }

        // no blocks seen
        if (searchCount >= 3) {
          searchCount = 0;
          isSearching = false;
          myState = main;
        } else {
          searchCount++;
          myState = right;
        }
        break;
      }



    default:
      Serial.println("Error!");
      break;
  }
}

// ======================
// INTERRUPT ROUTINES
// ======================

void changeUp1A() {
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB))
    counter1++;
  else
    counter1--;
}

void changeUp1B() {
  if (digitalRead(Motor1EPinA) == digitalRead(Motor1EPinB))
    counter1--;
  else
    counter1++;
}

void changeUp2A() {
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB))
    counter2++;
  else
    counter2--;
}

void changeUp2B() {
  if (digitalRead(Motor2EPinA) == digitalRead(Motor2EPinB))
    counter2--;
  else
    counter2++;
}

//**
// ultrasonic sensor distance measuring function
void measureDistance() {
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