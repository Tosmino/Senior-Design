#include <Arduino.h>
#include <HardwareSerial.h>
#include <BluetoothSerial.h>
#include <chrono>
#include <stdlib.h>
#include <string>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

// Check if Bluetooth configurations are enabled in the SDK
// If not, then you have to recompile the SDK
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Reset pin, MFIO pin
int resPin = 4;
int mfioPin = 2;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin);

// struct to hold vitals
bioData body;

HardwareSerial SerialPort(2);
BluetoothSerial SerialBT;

// define RX/TX pins for BT communication
#define RXD2 13
#define TXD2 12
#define SDA 21
#define SCL 22

// Easy to adjust bounds for heart rate
#define HRLOW 40
#define HRHIGH 200

// Compares vitals to thresholds. Returns true if in distress, false if not
bool checkState()
{
  // Check if user is in distress, return true (distress) or false (okay)
  if (body.status == 0 && body.confidence >= 80)
  {
    if (body.heartRate < HRLOW || body.heartRate > HRHIGH)
    {
      return true;
    }

    return false;
  }
  // return no detect (ask Jared about this one)
  return true;
}

// Retrieves current bio data and stores in "body"
void getBioData()
{
  // Repopulate body with new bio info
  body = bioHub.readBpm();
  Serial.print("Heartrate: ");
  Serial.println(body.heartRate);
  Serial.print("Confidence: ");
  Serial.println(body.confidence);
  Serial.print("Oxygen: ");
  Serial.println(body.oxygen);
  Serial.print("Status: ");
  Serial.println(body.status);
}

// Takes input of second serial connection
// If connected, prints MAC address of Bluetooth Device
void printBTMAC(bool serial2Status)
{
  // Check if serial connection is made
  if (serial2Status == true)
  {
    // Get the MAC address of the Bluetooth interface
    uint8_t baseMac[6];
    esp_read_mac(baseMac, ESP_MAC_BT);
    Serial2.print("Bluetooth MAC: ");
    for (int i = 0; i < 5; i++)
    {
      Serial2.printf("%02X:", baseMac[i]);
    }
    Serial2.printf("%02X\n\n", baseMac[5]);
  }
  return;
}

// Utilized to attempt reconnection to Transmitter pack via Bluetooth
void reconnectBluetooth(bool serial2Status = true)
{
  // Loop through sleeping then connecting while the connection isn't there
  while (!SerialBT.connected())
  {
    if (serial2Status == true)
    {
      Serial2.write("Connecting to bluetooth: Transmitter\n");
    }
    delay(200);
    SerialBT.connect("Transmitter");
  }

  // Print the connection success to the serial 2 if available
  if (serial2Status == true)
  {
    Serial2.write("Connected to Transmitter\n");
  }
}

// Send desired message to Bluetooth Module then sleep for 1 second
void BTSend(bool status)
{
  if (status)
  {
    Serial2.write("User Status: DISTRESSED");
    SerialBT.write(body.heartRate);
  }
  else
  {
    Serial2.write("User Status: OKAY");
    SerialBT.write((uint8_t)0);
  }
  delay(1000);
}

bool startSerial2()
{
  // Setup the UART 2 signal
  Serial.begin(115200);
  Serial.println("Serial Start");
  // Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  return true;
}

void setup()
{
  bool serial2Status = startSerial2();

  /*
  printBTMAC(serial2Status);

  // Setup the Bluetooth serial connection
  SerialBT.begin("Wristband", false);
  SerialBT.connect("Transmitter");

  reconnectBluetooth(serial2Status);
  */

  Wire.begin(SDA, SCL);
  int result = bioHub.begin(Wire, resPin, mfioPin);
  Serial.println("bioHub Started");
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");

  Serial.println("Configuring Sensor....");
  int error = bioHub.configSensorBpm(MODE_ONE); // Configuring just the BPM settings.
  if (error == 0)
  { // Zero errors!
    Serial.println("Sensor configured.");
  }
  else
  {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: ");
    Serial.println(error);
  }

  // Delay due to the data lagging slightly behind the sensor
  Serial.println("Loading up the buffer with data....");
  delay(4000);
}

void loop()
{

  // When a Bluetooth Connection is detected
  // while (SerialBT.connected())
  while (true)
  {
    // Update Bio Data
    getBioData();

    // Check if user is in distress and send info to Bluetooth
    checkState() ? BTSend(true) : BTSend(false);

    // Sleep for 10 seconds
    delay(10000);
  }

  /*
  // If the bluetooth is disconnected try to reconnect
  if (!SerialBT.connected())
  {
    reconnectBluetooth();
  }
  */
}