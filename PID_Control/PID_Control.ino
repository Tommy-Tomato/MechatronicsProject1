#include <DualMAX14870MotorShield.h>
#include <BNO055_support.h>
#include <Wire.h>

DualMAX14870MotorShield Motors;

// PID constants
double Kp = 2.0; // Proportional gain
double Ki = 0.0625 ; // Integral gain
double Kd = 0; // Derivative gain

// Target position
double setpoint = 0;

// Variables for PID control
double input, output;
double integral = 0;
double derivative = 0;
double previous_error = 0;

// Refresh rate
unsigned long refreshRate = 100; // Refresh rate in milliseconds
unsigned long lastTime = 0;

//IMU configs

struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data
 double yaw;
double pitch;
 double roll;
//


void setup() {
  Serial.begin(9600);

  Motors.enableDrivers();

//IMU configs
 //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(5);
//
  setpoint = readPosition();
}

void loop() {
  unsigned long now = millis();
  if (now - lastTime >= refreshRate) {
    lastTime = now;

    // Read current position
    input = readPosition(); // Placeholder for position reading function

    // Calculate PID
    double error = setpoint - input;
    integral += error * (refreshRate / 1000.0);
    derivative = (error - previous_error) / (refreshRate / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;

    // Apply output to your system
    applyOutput(output); // Placeholder for output application function
  }
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
  Serial.println(yaw);
    return yaw; // Return the actual sensor value
}


 // Changed from 100ms to 20ms for a 50Hz refresh rate
// Placeholder function to apply output
void applyOutput(double output) {
  //Serial.println(output);
  int outputInt = (int)output; 
  int outputSpeed = constrain(outputInt,-400,400);
  //if (outputSpeed < 75 && outputSpeed > 0) {
    //outputSpeed = 75;
  //} else if (outputSpeed < 0 && abs(outputSpeed) < 75) {
    //outputSpeed = -75;
  //}
  //Serial.println(outputSpeed);
  Motors.setSpeeds(outputSpeed + 100, -outputSpeed + 100);
  
}




