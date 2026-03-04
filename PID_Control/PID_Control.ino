#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield Motors;

// PID constants
double Kp = 2.0; // Proportional gain
double Ki = 0.5; // Integral gain
double Kd = 1.0; // Derivative gain

// Target position
double setpoint = 100;

// Variables for PID control
double input, output;
double integral = 0;
double derivative;
double previous_error = 0;

// Refresh rate
unsigned long refreshRate = 100; // Refresh rate in milliseconds
unsigned long lastTime = 0;



void setup() {
  Serial.begin(9600);
  
  Motors.enableDrivers();
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
  return 0; // Return the actual sensor value
}

// Placeholder function to apply output
void applyOutput(double output) {
  int outputSpeed = map(output,0,360,-400,400);
  Motors.setSpeeds(outputSpeed, -outputSpeed);
  
}
 
