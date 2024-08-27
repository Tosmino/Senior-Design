#include <Arduino.h>
#include <SensorHub.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <chrono>
#include <stdlib.h>
#include <string>
#include <Wire.h>

// Reset pin, MFIO pin
int resPin = 4;
int mfioPin = 2;

#define SDA 21
#define SCL 22

// To activate hardware test mode, make this 1
#define HWTESTMODE 0

uint8_t userMode = 0x01;

bool configFlag = 0;

// Takes address, reset pin, and MFIO pin.
Sensor_Hub bioHub(resPin, mfioPin);

// struct to hold vitals
bioData body;

// struct to hold raw data
rawData data;

HardwareSerial SerialPort(2);

// Initializes I2C, biohub configuration, sensor configuration, outputs results to serial
void initializeDevices()
{
  int result = bioHub.begin(Wire, resPin, mfioPin);
  Serial.println("bioHub Started");
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");

  bioHub.printSensorHubVersion();
  delay(2000);

  Serial.println("Configuring Sensor...");
  int error = bioHub.configAGC(userMode); // Configuring just the BPM settings.
  if (error == 0)
  { // Zero errors!
    Serial.println("Standard Mode Enabled");
  }
  else
  {
    Serial.println("Error enabling Standard Mode");
    Serial.print("Error: ");
    Serial.println(error);
    configFlag = 1;
  }

  while (configFlag)
  {
  }

  // Delay due to the data lagging slightly behind the sensor
  Serial.println("Loading up the buffer with data...");
  delay(4000);
}

// Initializes wire communication, configures devices to only read raw sensor data
void initializeHardwareTest()
{
  Serial.println("HARDWARE TEST MODE DETECTED");
  int result = bioHub.begin(Wire, resPin, mfioPin);
  Serial.println("bioHub Started");
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");

  bioHub.printSensorHubVersion();
  delay(2000);

  Serial.println("Configuring Sensor...");
  int error = bioHub.rawDataMode(); // Configuring just the BPM settings.
  if (error == 0)
  { // Zero errors!
    Serial.println("Hardware Test Mode Enabled");
  }
  else
  {
    Serial.println("Error enabling Hardware Test");
    Serial.print("Error: ");
    Serial.println(error);
    configFlag = 1;
  }

  while (configFlag)
  {
  }
  Serial.println("Checking Configuration Settings...");

  delay(500);

  bioHub.printConfigSettings();

  delay(500);

  // Delay due to the data lagging slightly behind the sensor
  Serial.println("Loading up the buffer with data...");
  delay(4000);
}

rawData getRawData()
{
  size_t vectorSize;
  std::vector<int32_t> dataVector;
  dataVector = bioHub.readRawData(vectorSize);
  return bioHub.parseSample(dataVector);
}

void printRawData()
{
  Serial.print("greenCount1:");
  Serial.println(data.greenCount1);

  Serial.print("greenCount2:");
  Serial.println(data.greenCount2);

  Serial.print("IRCount:");
  Serial.println(data.IRCount);

  Serial.print("redCount:");
  Serial.println(data.redCount);

  Serial.print("accelX:");
  Serial.println(data.accelX);

  Serial.print("accelY:");
  Serial.println(data.accelY);

  Serial.print("accelZ:");
  Serial.println(data.accelZ);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Serial Start");

  Wire.begin(SDA, SCL);

  delay(2000);

  // Initialize device in intended mode
  HWTESTMODE ? initializeHardwareTest() : initializeDevices();
}

void loop()
{
  // Check if in HWTestMode
  while (HWTESTMODE)
  {
    data = getRawData();
    printRawData();
    delay(2000); // Sleep for 2 seconds
  }
}
