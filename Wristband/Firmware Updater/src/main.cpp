/*
 This program is utilized to flash firmware to the MAX32664 from a host microcontroller
 (in my case, the ESP32-MINI-1) via i2c communication. This is my first crack at writing
 firmware to flash other firmware, so use at your own risk big dawg. This code also utilizes
 Sparkfun's bio sensor hub library they created for the MAX32664A. However, the type of sensor
 shouldn't matter, just make sure you have the correct firmware version and change the filePath
 to the name of the .msbl file.

 Author: Frankie Ketchum
 Date: 4/2024

 If you run into an error code check the following table to help diagnose your
 problem:
 1 = Unavailable Command
 2 = Unavailable Function
 3 = Data Format Error
 4 = Input Value Error
 5 = Try Again
 255 = Error Unknown
*/

#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

#define SDA 21
#define SCL 22

// Reset pin, MFIO pin
int resPin = 4;
int mfioPin = 2;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);

bioData body;

void setup()
{
  Serial.begin(9600);

  Wire.begin(SDA, SCL);
  int result = bioHub.beginBootloader();
  if (result == 8) // Zero errors!
    Serial.println("Bootloader Started!");
  else
    Serial.println("Could not communicate with the sensor!");
  delay(4000);

  // Start bootloader functions
  bioHub.flashApplication();
}

void loop()
{
  delay(10000);
}