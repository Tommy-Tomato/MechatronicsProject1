
#include "BNO055_support.h"		//Contains the bridge code between the API and Arduino
#include <Wire.h>

//The device address is set to BNO055_I2C_ADDR2 in this example. You can change this in the BNO055.h file in the code segment shown below.
// /* bno055 I2C Address */
// #define BNO055_I2C_ADDR1                0x28
// #define BNO055_I2C_ADDR2                0x29
// #define BNO055_I2C_ADDR                 BNO055_I2C_ADDR2

//Pin assignments as tested on the Arduino Due.
//Vdd,Vddio : 3.3V
//GND : GND
//SDA/SCL : SDA/SCL
//PSO/PS1 : GND/GND (I2C mode)

//This structure contains the details of the BNO055 device that is connected. (Updated after initialization)
struct bno055_t myBNO;
struct bno055_euler myEulerData; //Structure to hold the Euler data

unsigned long lastTime = 0;

void setup() //This code is executed once
{
  //Initialize I2C communication
  Wire.begin();

  //Initialization of the BNO055
  BNO_Init(&myBNO); //Assigning the structure to hold information about the device

  //Configuration to NDoF mode
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

  delay(1);

  //Initialize the Serial Port to view information on the Serial Monitor
  Serial.begin(9600);
}

void loop() 
{
  // Changed from 100ms to 20ms for a 50Hz refresh rate
  if ((millis() - lastTime) >= 20) 
  {
    lastTime = millis();

    // Update Euler data into the structure
    bno055_read_euler_hrp(&myEulerData);			

    // Convert raw data to degrees (Divide by 16.0 as per Bosch API)
    float yaw   = (float)myEulerData.h / 16.00;
    float pitch  = (float)myEulerData.r / 16.00;
    float roll = (float)myEulerData.p / 16.00;

    // Print values in a single line for easier reading in Serial Monitor
    Serial.print("(Yaw(Z-axis)): "); Serial.print(yaw);
    Serial.print(" Roll(Y-axis): "); Serial.print(roll);
    Serial.print(" Pitch(X-axis): "); Serial.println(pitch);
    Serial.println();

  }
  
}