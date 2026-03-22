#include <DualMAX14870MotorShield.h>
#include <Pixy2.h>
#include <BNO055_support.h>
#include <Wire.h>

DualMAX14870MotorShield Motors;
Pixy2 pixy;

unsigned long refreshRate = 100; // Refresh rate in milliseconds ESTIMATE

// PID constants
double Kp = 2.0; // Proportional gain
double Ki = 0.0625 ; // Integral gain
double Kd = 0; // Derivative gain

bool isTurning = false;
double turnStartYaw = 0;
double targetYaw = 0;
const double TURN_TOLERANCE = 5; // degrees


// Target position
double setpoint = 0;

// Variables for PID control
double input, output;
double integral = 0;
double derivative = 0;
double previous_error = 0;

// Ultrasonic sensor code
int signal = 26;
int distance;
int gap = 15;
unsigned long pulseduration = 0;

//ir sensors
#define irLeft A8
#define irRight A9
const float DISTANCE_CUTOFF = 2.8; //NORMAL: 2.8

//IMU configs

struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
 double yaw;
double pitch;
 double roll;
//

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
float COUNTS_PER_REV = 12.0 * 4.0 * 9.75;
float TURN_CIRC = 59.69;
float WHEEL_CIRC_CM = 22;
float FULL_ROTATION_COUNTS = (TURN_CIRC / WHEEL_CIRC_CM) * COUNTS_PER_REV;

int TURN_SPEED = 150;
int STRAIGHT_SPEED = 90;
int CORRECT_SPEED = 85;

int CORRECTION_COUNT = 20;
float correctStartTime = 0;
float correctDelay = 5000;

int SEARCH_COOLDOWN = 50000; //NORMAL: 5000
bool isSearching = false;
double WALL_CUTOFF = 1.5;

long startTime = millis();



enum States {
  initialize,
  main,
  correctLeft,
  correctRight,
  left,
  right,
  back
};

States myState = initialize;

void setup() {

  // Initialize I2C
Wire.begin();

// Initialize the BNO055
BNO_Init(&myBNO);

// Set operation mode
bno055_set_operation_mode(OPERATION_MODE_NDOF);

  // ultrasonic sensor setup
  delay(5000);
  pinMode(signal, OUTPUT);
  setpoint = readPosition();

  //motor setup
  Motors.enableDrivers();

  //pixy setup
  Serial.begin(9600);
  pixy.init();
  Serial.println("Pixy Initialized...");

  //initializing motor pins & attaching interrupts
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

  Serial.println(myState);
  //Get the raw distance measurement value
  measureDistance();
  distance = (pulseduration / 2) * 0.0343;
  delay(50);

  //retrieve pixy info  
  pixy.ccc.getBlocks();

  switch (myState) {
    case initialize:
      myState = main;
      break;

    /*
    Main Case: Arduino drives forwards and monitors sensors
    After cooldown, activates sensing mode
    */
    case main: {
      //Serial.println("main!");
      //check IR Sensors
  
      float leftVolts = analogRead(irLeft) * 0.0048828125;  // value from sensor * (5/1024)
      float rightVolts = analogRead(irRight) * 0.0048828125;

      if (leftVolts > DISTANCE_CUTOFF && millis() - correctStartTime > correctDelay) {
        //Serial.print("ABOVE CUT");
        //myState = correctRight;
        //Serial.print("SET STATE -> ");
        //Serial.println(myState);
        //break;
      }

      if (rightVolts > DISTANCE_CUTOFF && millis() - correctStartTime > correctDelay) {
        //Serial.print("ABOVE CUT");
        //myState = correctLeft;
        //break;
      }
      //Serial.println("setting speeds");
      //Move Forwards
      PIDSetSpeeds();
      //Motors.setSpeeds(STRAIGHT_SPEED, STRAIGHT_SPEED + 1);

      //check Pixy
      if (pixy.ccc.numBlocks) {
        isSearching = false;
        startTime = millis();
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
              Serial.println("got to state set");
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
              delay(200);      // stability
              myState = left;  // left turn
            }
          break;
            
          }
        }
        //if Robot is in searching mode
      } else if (isSearching) {
          //if right wall too far away, turn right
          if (leftVolts < WALL_CUTOFF) {
            myState = right;
            break;
          }
          //if left wall too far away, turn left
          if (rightVolts < WALL_CUTOFF) {
            myState = left;
            break;
          }
        
      }
      //activate search after cooldown
      //Serial.println(millis()-startTime);
      if (millis()-startTime > SEARCH_COOLDOWN) {
        Serial.println("SEARCHING");
        isSearching = true;
      }

      break;
    }

    /*
    correctLeft case: robot corrects to the right
    */
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

    /*
    correctRight case: robot corrects to the left
    */
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

    /*
    Left case: robot turns left
    */
case left: {
  if (!isTurning) {
    Serial.println("turning left");

    turnStartYaw = readPosition();
    setpoint = turnStartYaw - 90;

    if (setpoint > 180) setpoint -= 360;

    isTurning = true;

    integral = 0;
    previous_error = 0;

    startTime = millis();
    isSearching = false;
  }

  // run PID continuously
  PIDSetSpeeds();

  double currentYaw = readPosition();
  double error = setpoint - currentYaw;
  Serial.println(error);

  // wrap error
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (abs(error) < 5) {
    Motors.setSpeeds(0, 0);

    isTurning = false;
    myState = main;

    setpoint = currentYaw; // lock heading
    delay(100);
  }

  break;
}



    /*
    Right case: robot turns right
    */
case right: {
Serial.println("gets to case");
if (!isTurning) {
    Serial.println("turning right");

    turnStartYaw = readPosition();
    setpoint = turnStartYaw + 90;

    if (setpoint > 180) setpoint -= 360;

    isTurning = true;

    integral = 0;
    previous_error = 0;

    startTime = millis();
    isSearching = false;
  }

  // run PID continuously
  PIDSetSpeeds();

  double currentYaw = readPosition();
  double error = setpoint - currentYaw;
  Serial.println(error);

  // wrap error
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (abs(error) < 5) {
    Motors.setSpeeds(0, 0);

    isTurning = false;
    myState = main;

    setpoint = currentYaw; // lock heading
    delay(100);
  }

  break;
}


    /*
    Back case: robot turns around
    */
    case back: {

    if (!isTurning) {
    Serial.println("turning left");

    turnStartYaw = readPosition();
    setpoint = turnStartYaw + 180;

    if (setpoint > 180) setpoint -= 360;

    isTurning = true;

    integral = 0;
    previous_error = 0;

    startTime = millis();
    isSearching = false;
  }

  // run PID continuously
  PIDSetSpeeds();

  double currentYaw = readPosition();
  double error = setpoint - currentYaw;
  Serial.println(error);

  // wrap error
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  if (abs(error) < 5) {
    Motors.setSpeeds(0, 0);

    isTurning = false;
    myState = main;

    setpoint = currentYaw; // lock heading
    delay(100);
  }

  break;
    }



    default:
      Serial.println("Error!");
      break;
  }
}

void PIDSetSpeeds() {
      // Read current position
    //Serial.println("SETTING SPEED");
    input = readPosition(); // Placeholder for position reading function

    // Calculate PID
    double error = setpoint - input;
    integral += error * (refreshRate / 1000.0);
    derivative = (error - previous_error) / (refreshRate / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Apply output to your system
    applyOutput(output); // Placeholder for output application function
    Serial.println(error);
}

// Placeholder function to read your position sensor
double readPosition() {
  // Implement sensor reading

// Changed from 100ms to 20ms for a 50Hz refresh rate
 
    // Update Euler data into the structure
    bno055_read_euler_hrp(&myEulerData);			

    // Convert raw data to degrees (Divide by 16.0 as per Bosch API)
 yaw   = (double)myEulerData.h / 16.00;
 pitch  = (double)myEulerData.r / 16.00;
 roll = (double)myEulerData.p / 16.00;

    // Print values in a single line for easier reading in Serial Monitor
    //Serial.println("Yaw (Z-axis): "); Serial.print(yaw);
    //Serial.print(" Roll(Y-axis): "); Serial.print(roll);
    //Serial.print(" Pitch(X-axis): "); Serial.println(pitch);
    //
  if (yaw > 180){
    yaw = yaw - 360;
  }
  
  //Serial.print("yaw: ");
  //Serial.println(yaw);
    return yaw; // Return the actual sensor value
}


 // Changed from 100ms to 20ms for a 50Hz refresh rate
// Placeholder function to apply output
void applyOutput(double output) {
  int outputInt = (int)output; 
  int outputSpeed = constrain(outputInt, -200, 200);

  if (isTurning) {
    // PURE ROTATION
    if (abs(outputSpeed) < 60){
      if (outputSpeed < 0){
        outputSpeed = outputSpeed - 60;
      }
      else if (outputSpeed > 0){
        outputSpeed = outputSpeed + 60;
      }
    }
    Motors.setSpeeds(outputSpeed, -outputSpeed);
    Serial.println(outputSpeed);
  } else {
    // NORMAL DRIVING
    Motors.setSpeeds(outputSpeed + 100, -outputSpeed + 100);
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