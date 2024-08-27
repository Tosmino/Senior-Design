/* Title: Senior Design Team 4 Demo Day Interactive Demo
   Code Author: Frankie Ketchum (fjk12@uakron.edu)
                *Utilizes code from Sparkfun's MAX3010x library*
   Purpose: Interactive demonstration for use in explaining functionality of Emergency
            Automatic Personal Locator Beacon
   License: No License, feel free to use for whatever project
*/

#include "heartRate.h"
#include "main.h"
#include "MAX30105.h"

// Initialize Objects
BluetoothSerial SerialBT;
MAX30105 particleSensor;

// Define Flags for Events
volatile bool int1_trigger = false;
volatile bool HR_trigger = false;
volatile bool manual_trigger = false;

const byte RATE_SIZE = 4; // Array used to calculate avg Heart Rate
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;                 // Time at which the last beat occurred
float beatsPerMinute;              // Calculated Heart Rate
float previousBeatsPerMinute = 80; // Used to eliminate outliers
int beatAvg = 80;                  // Calculated average BPM

int blockDelay = 0;  // Delay used to not spam Okay messages
int btResetFlag = 0; // Flag received from BT device

void setup()
{
    // Initialize communication
    Serial.begin(115200);
    Wire.begin(SDA, SCL);

    PrintBTMAC(true); // Print Bluetooth MAC Address to terminal

    // Setup the Bluetooth serial connection
    SerialBT.begin("Wristband", false);
    SerialBT.connect("Transmitter");

    ReconnectBluetooth(true); // Connect Bluetooth initially

    InitializeAccel();  // Initialize Accelerometer
    InitializeSensor(); // Initialize HR Sensor

    // Attach interrupt to INT1 pin
    attachInterrupt(digitalPinToInterrupt(INT1_PIN), Int1_ISR, RISING);

    pinMode(MANUAL_HR_PIN, INPUT); // Define Manual HR Pin as an input
    pinMode(RESET_PIN, INPUT);     // Define Reset Pin as an input
    pinMode(BTDISCONNECTPIN, INPUT);

    // Assign RGB pins
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);

    // Check that red works
    SetColor(255, 0, 0);
    delay(500);

    // Check that green works
    SetColor(0, 255, 0);
    delay(500);

    // Check that blue works
    SetColor(0, 0, 255);
    delay(500);

    Serial.println("Demonstration Initialized.");                                  // End of setup
    Serial.println("Place your index finger on the sensor with steady pressure."); // Call to action
}

void loop()
{
    // Continue to send sample info to the transmitter
    while (SerialBT.connected())
    {
        if (digitalRead(MANUAL_HR_PIN) == LOW)
        {
            beatAvg = 20;        // Set average below threshold
            beatsPerMinute = 20; // Set BPM below threshold

            // Wait for button to be released
            while (digitalRead(MANUAL_HR_PIN) == LOW)
            {
            }
        }

        // If HR wasn't manually set
        if (!HR_trigger)
        {
            CheckHR(); // Read Sensor Samples
        }

        // If cardiac event detected. Range: 40 - 240 BPM
        if (((beatAvg < 40 || beatAvg > 240) && beatsPerMinute != 0))
        {
            SetColor(255, 0, 0); // Set LED to Red
            Serial.println("Cardiac Event Detected!");
            Serial.print("HR: ");
            Serial.println(beatsPerMinute);
            Serial.print("Avg HR: ");
            Serial.println(beatAvg);
            HR_trigger = true; // Set Flag to active

            while (HR_trigger)
            {
                if (true) // SerialBT.connected())
                {
                    SendWarn(); // Send warning message for Heart Rate
                    Serial.println("Warning Sent");
                    delay(1000);
                    if (SerialBT.available())
                    {
                        btResetFlag = SerialBT.read();
                    }
                    if (digitalRead(RESET_PIN) == LOW || btResetFlag == 1)
                    {
                        // Print reset message
                        Serial.println("Reset Pin Detected...");
                        Serial.println("Returning to Okay State.");

                        HR_trigger = false; // Reset Flag
                        beatAvg = 80;       // Reset average to something standard
                        beatsPerMinute = 0; // Reset BPM to 0

                        btResetFlag = 0; // Clear Bluetooth Reset Flag
                    }
                }
                else
                {
                    ReconnectBluetooth(true); // Re-establish Bluetooth Connection
                }
            }
        }

        // If free fall event detected
        if (digitalRead(INT1_PIN) == HIGH)
        {
            SetColor(0, 255, 0);
            Serial.println("Freefall Detected!");
            while (digitalRead(INT1_PIN) == HIGH)
            {
                if (SerialBT.connected())
                {
                    SendWarn();
                    Serial.println("Warning Sent");
                    delay(1000);
                    if (SerialBT.available())
                    {
                        btResetFlag = SerialBT.read();
                    }
                    if (digitalRead(RESET_PIN) == LOW || btResetFlag == 1)
                    {
                        Serial.println("Reset Pin Detected...");
                        Serial.println("Returning to Okay State.");
                        int1_trigger = false;

                        // Read the interrupt release register to unlatch interrupt
                        ReadByte(INT_REL);

                        btResetFlag = 0; // Clear Bluetooth Reset Flag
                    }
                }
                else
                {
                    ReconnectBluetooth(true);
                }
            }
        }

        if (digitalRead(BTDISCONNECTPIN) == HIGH)
        {
            // Disconnect bluetooth and wait
            SerialBT.disconnect();
            SetColor(0, 0, 0);
            while (digitalRead(BTDISCONNECTPIN) == HIGH)
            {
                if (SerialBT.connected())
                {
                    SerialBT.disconnect();
                }
                else
                {
                    Serial.println("Bluetooth Disconnected");
                }
                delay(1000);
            }
        }

        // Otherwise, send okay!
        if (blockDelay == 50)
        {
            blockDelay = 0; // Reset delay
            SendOK();       // Send Okay message
        }
        else
            SetColor(255, 255, 255); // Return to default color

        blockDelay++;
    }

    // If the bluetooth is disconnected try to reconnect
    ReconnectBluetooth(true);
    delay(500);
}

// Function to read data from MAX30101, stores in global variables
void CheckHR()
{
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 30)
        {
            if (abs((beatsPerMinute - previousBeatsPerMinute)) < 10)
            {
                previousBeatsPerMinute = beatsPerMinute;

                rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
                rateSpot %= RATE_SIZE;                    // Wrap variable

                // Take average of readings
                beatAvg = 0;
                for (byte x = 0; x < RATE_SIZE; x++)
                    beatAvg += rates[x];
                beatAvg /= RATE_SIZE;

                Serial.print("IR=");
                Serial.print(irValue);
                Serial.print(", BPM=");
                Serial.print(beatsPerMinute);
                Serial.print(", Avg BPM=");
                Serial.println(beatAvg);
            }
        }
    }

    if (irValue < 50000)
    {
        Serial.println(" No Finger Detected");
        delay(1000);
    }
}

// Setup commands for KX122 Accelerometer
void InitializeAccel()
{
    WriteByte(CNTL1, 0x40);
    WriteByte(FFCNTL, 0x80);
    WriteByte(FFC, 0X01);
    WriteByte(FFTH, 0x08);
    WriteByte(INC1, 0x30);
    WriteByte(INC4, 0x80);
    WriteByte(CNTL1, 0xC0);
}

// Initialize MAX30101 to communicate and operate with default settings
void InitializeSensor()
{
    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
    {
        Serial.println("MAX30105 was not found. Please check wiring/power. ");
        while (1)
            ;
    }

    particleSensor.setup();                    // Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
}

// Interrupt Service Routine for Accelerometer Interrupt Pin
void Int1_ISR()
{
    int1_trigger = true;
}

// Function to print Bluetooth MAC Address to Terminal
void PrintBTMAC(bool serialStatus)
{
    if (serialStatus == true)
    {
        // Get the MAC address of the Bluetooth interface
        uint8_t baseMac[6];
        esp_read_mac(baseMac, ESP_MAC_BT);
        Serial.print("Bluetooth MAC: ");
        for (int i = 0; i < 5; i++)
        {
            Serial.printf("%02X:", baseMac[i]);
        }
        Serial.printf("%02X\n\n", baseMac[5]);
    }
}

// Given a Register, Reads a Byte From KX122 Accelerometer
void ReadByte(uint8_t reg)
{
    Wire.beginTransmission(KX122_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false);

    delay(20);
    Wire.requestFrom(KX122_ADDRESS, 1); // Request 1 byte

    if (Wire.available())
    {
        uint8_t value = Wire.read();
    }
}

// Function to Re-establish Bluetooth Connection
void ReconnectBluetooth(bool serialStatus)
{
    // Loop through sleeping then connecting while the connection isn't there
    SetColor(0, 0, 0);
    while (!SerialBT.connected())
    {
        SerialBT.connect("Transmitter");
        delay(1000);
    }

    // Print the Connection Success to the Terminal if Available
    if (serialStatus == true)
    {
        SetColor(255, 255, 255);
        Serial.println("Connected to Transmitter");
    }
}

// Function to Send Okay Message
void SendOK()
{
    SetColor(0, 0, 255);
    SerialBT.write((uint8_t)0);
    delay(1000);
}

// Function to Send Warning Message Based on Flag Tripped
void SendWarn()
{
    if (int1_trigger)
    {
        // Send a warning message then sleep for one second
        SerialBT.write((uint8_t)100);
    }
    if (HR_trigger)
    {
        // Send a warning message then sleep for one second
        SerialBT.write((uint8_t)beatAvg);
    }
    delay(1000);
}

// Set Color of RGB LED for Visual Demonstration
void SetColor(int red, int green, int blue)
{
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}

// Given a Register and Value, Writes the Value to the Register of the KX122 Accelerometer
void WriteByte(uint8_t reg, uint8_t value)
{
    // Initialize KX122 accelerometer
    Wire.beginTransmission(KX122_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
    delay(2);
}