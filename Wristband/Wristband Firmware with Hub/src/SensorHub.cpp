/*
  This is an Arduino Library written for the MAXIM 32664 Biometric Sensor Hub
  The MAX32664 Biometric Sensor Hub is in actuality a small Cortex M4 microcontroller
  with pre-loaded firmware and algorithms used to interact with the a number of MAXIM
  sensors; specifically the MAX86141 Pulse Oximter and Heart Rate Monitor and
  the KX122 Accelerometer. With that in mind, this library is built to
  communicate with a middle-person and so has a unique method of communication
  (family, index, and write bytes) that is more simplistic than writing and reading to
  registers, but includes a larger set of definable values.

  SparkFun Electronics
  Date: June, 2019
  Author: Elias Santistevan
kk
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
*/

#include "SensorHub.h"

Sensor_Hub::Sensor_Hub(int resetPin, int mfioPin, uint8_t address)
{

    _resetPin = resetPin;
    if (resetPin >= 0)
        pinMode(_resetPin, OUTPUT); // Set the pin as output

    _mfioPin = mfioPin;
    if (mfioPin >= 0)
        pinMode(_mfioPin, OUTPUT); // Set the pin as output

    _address = address;
}

// THIS FUNCTION WORKS
// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled HIGH while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
uint8_t Sensor_Hub::begin(TwoWire &wirePort, int resetPin, int mfioPin)
{

    _i2cPort = &wirePort;
    //  _i2cPort->begin(); A call to Wire.begin should occur in sketch
    //  to avoid multiple begins with other sketches.

    if (resetPin >= 0)
    {
        _resetPin = resetPin;
        pinMode(_resetPin, OUTPUT); // Set the pin as output
    }

    if (mfioPin >= 0)
    {
        _mfioPin = mfioPin;
        pinMode(_mfioPin, OUTPUT); // Set the pin as output
    }

    if ((_resetPin < 0) || (_mfioPin < 0)) // Bail if the pins have still not been defined
        return 0xFF;                       // Return ERR_UNKNOWN

    digitalWrite(_mfioPin, HIGH);
    digitalWrite(_resetPin, LOW);
    delay(10);
    digitalWrite(_resetPin, HIGH);
    delay(1000);
    pinMode(_mfioPin, INPUT_PULLUP); // To be used as an interrupt later

    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte.
    return responseByte;
}

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
uint8_t Sensor_Hub::readSensorHubStatus()
{
    uint8_t status = readByte(0x00, 0x00); // Just family and index byte.
    return status;                         // Will return 0x00
}

uint8_t Sensor_Hub::configBPM(uint8_t mode)
{

    uint8_t statusChauf = 0;
    if (mode == MODE_ONE || mode == MODE_TWO)
    {
    }
    else
        return INCORR_PARAM;

    statusChauf = setOutputMode(ALGO_DATA); // Just the data
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("Output mode set!");

    statusChauf = setFifoThreshold(0x01); // One sample before interrupt is fired.
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("FIFO threshold set!");

    statusChauf = aecAlgoControl(MODE_ONE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("AEC algo controlled!");

    statusChauf = max86141Control(ENABLE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("MAX86141 Enabled!");

    statusChauf = maximFastAlgoControl(mode);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("WHRM algo enabled!");

    _userSelectedMode = mode;
    _sampleRate = readAlgoSamples();

    delay(1000);
    return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::configAGC(uint8_t mode)
{
    uint8_t statusChauf = 0;
    if (mode == MODE_ONE || mode == MODE_TWO)
    {
    }
    else
        return INCORR_PARAM;

    // 1.2 Set mode to sensor + algorithm
    // 10 00 03
    statusChauf = setOutputMode(SENSOR_AND_ALGORITHM); // Just the data
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("Output Mode Set!");
    delay(1000);

    // 1.3 Set interrupt threshold
    // 10 01 01
    statusChauf = setFifoThreshold(0X01);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("FIFO Threshold Set!");
    delay(1000);

    // 1.4 Set report rate
    // 10 02 01
    // statusChauf = setReportRate(0X01);
    // if (statusChauf != SFE_BIO_SUCCESS)
    //     return statusChauf;

    // Serial.println("Report Rate Set!");
    // delay(1000);

    // 1.5 Enable sensor hub accelerometer
    // 44 04 01 00
    statusChauf = enableAccelerometer();
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("Accelerometer Enabled!");
    delay(1000);

    // 1.6 Set algorithm operation mode
    // 50 07 0A 00
    statusChauf = setAlgoOpMode(0X00);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("Algo Operation Mode Set!");
    delay(1000);

    // 1.7 Enable AEC
    // 50 07 0B 01
    statusChauf = aecEnable(ENABLE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("AEC Enabled!");
    delay(1000);

    // 1.8 Disable Auto PD Current Calculation
    // 50 07 12 00
    statusChauf = autoPDEnable(DISABLE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("Auto PD Disabled!");

    // 1.9 Disable SCD (Skin Contact Detection)
    // 50 07 0C 00
    statusChauf = SCDEnable(DISABLE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("SCD Disabled!");
    delay(1000);

    // 1.10 Set Target PD Current (10uA)
    // 50 07 11 00 64
    statusChauf = setAGCTarget(0x0064);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("AGC Target Set!");
    delay(1000);

    // 1.11 All algo config setting must be before here

    // 1.12 Enable WHRM and SpO2 Algorithm
    // 52 07 01 (normal report)
    statusChauf = agcAlgoControl(MODE_ONE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    Serial.println("AGC Algorithm Enabled!");

    delay(1000);
    return SFE_BIO_SUCCESS;
}

// This function sets very basic settings to get sensor and biometric data.
uint8_t Sensor_Hub::rawDataMode()
{
    uint8_t statusChauf = 0;

    // Set output mode to Sensor Only
    // 10 00 01
    statusChauf = setOutputMode(SENSOR_DATA); // Just the data
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Set the sensor hub interrupt threshold
    // 10 01 01
    statusChauf = setFifoThreshold(0X01);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Enable accelerometer
    // 44 04 01 00
    statusChauf = enableAccelerometer();
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Enable AFE
    // 44 00 01 00
    statusChauf = max86141Control(ENABLE);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Set sample rate
    // 40 00 12 18
    statusChauf = setSampleRate();
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Set LED1 current to half scale
    // 40 00 23 7F
    statusChauf = setLEDtoHalfScale(0x23);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Set LED2 current to half scale
    // 40 00 24 7F
    statusChauf = setLEDtoHalfScale(0x24);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    // Set LED3 current to half scale
    // 40 00 25 7F
    statusChauf = setLEDtoHalfScale(0x25);
    if (statusChauf != SFE_BIO_SUCCESS)
        return statusChauf;

    return SFE_BIO_SUCCESS;
}

std::vector<int32_t> Sensor_Hub::readRawData(size_t &vectorSize)
{
    rawData rawDataFifo;
    uint8_t byteData = readByte(HUB_STATUS, 0x00);
    bool DataRdyInt = (byteData >> 3) & 0x01;

    // Wait until data is ready
    while (!DataRdyInt)
    {
        delay(10000);
        byteData = readByte(HUB_STATUS, 0x00);
        DataRdyInt = (byteData >> 3) & 0x01;
    }

    // Get number of samples in FIFO
    vectorSize = readByte(0x12, 0x00);
    std::vector<int32_t> dataVector;

    // Populate array with data stored in FIFO
    readMultipleBytes(0x12, 0x01, vectorSize, dataVector);

    return dataVector;
}

uint8_t Sensor_Hub::enableAccelerometer()
{
    uint8_t value = 0x00;
    uint8_t statusByte = writeByte(ENABLE_SENSOR, 0x04, 0x01, value);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::setSampleRate()
{
    uint8_t value = 0x18;
    uint8_t statusByte = writeByte(0x40, 0x00, 0x12, value);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::setLEDtoHalfScale(uint8_t address)
{
    uint8_t value = 0x7F;
    uint8_t statusByte = writeByte(0x40, 0x00, address, value);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

rawData Sensor_Hub::parseSample(const std::vector<int32_t> &sample)
{
    rawData data;

    // Extract greenCount1 (3 bytes)
    data.greenCount1 = sample[0];

    // Discard 6 bytes (from index 1 to 6)

    // Extract greenCount2 (3 bytes)
    data.greenCount2 = sample[7];

    // Extract IRCount (3 bytes)
    data.IRCount = sample[8];

    // Extract redCount (3 bytes)
    data.redCount = sample[9];

    // Extract accelX (2 bytes)
    data.accelX = static_cast<int16_t>(sample[10]);

    // Extract accelY (2 bytes)
    data.accelY = static_cast<int16_t>(sample[11]);

    // Extract accelZ (2 bytes)
    data.accelZ = static_cast<int16_t>(sample[12]);

    return data;
}

void Sensor_Hub::printConfigSettings()
{
    Serial.print("FIFO Mode: ");
    Serial.println(readByte(0x11, 0x00), HEX);

    delay(2000);

    Serial.print("Interrupt Thresh: ");
    Serial.println(readByte(0x11, 0x01), HEX);

    delay(2000);

    int32_t userArray[2];
    Serial.print("Accelerometer Enable: ");
    readMultipleBytes(0x45, 0x04, 1, userArray);
    Serial.println(userArray[0], HEX);
    Serial.println(userArray[1], HEX);

    // Serial.print("Accelerometer Enable: ");
    // Serial.println(readByte(0x45, 0x04), HEX);

    delay(2000);

    Serial.print("MAX86141 Enable: ");
    Serial.println(readByte(0x45, 0x00), HEX);

    delay(2000);

    Serial.print("Sample Rate: ");
    Serial.println(readRegister(0x41, 0x00, 0x12), HEX);

    delay(2000);

    Serial.print("LED1 Current: ");
    Serial.println(readByte(0x41, 0x00, 0x23), HEX);

    delay(2000);

    Serial.print("LED2 Current: ");
    Serial.println(readByte(0x41, 0x00, 0x24), HEX);

    delay(2000);

    Serial.print("LED3 Current: ");
    Serial.println(readByte(0x41, 0x00, 0x25), HEX);
}

uint8_t Sensor_Hub::setOutputMode(uint8_t outputType)
{

    if (outputType > SENSOR_ALGO_COUNTER) // Bytes between 0x00 and 0x07
        return INCORR_PARAM;

    // Check that communication was successful, not that the IC is outputting
    // correct format.
    uint8_t statusByte = writeByte(OUTPUT_MODE, SET_FORMAT, outputType);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::setFifoThreshold(uint8_t intThresh)
{
    // Checks that there was succesful communcation, not that the threshold was
    // set correctly.
    uint8_t statusByte = writeByte(OUTPUT_MODE, WRITE_SET_THRESHOLD, intThresh);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::setReportRate(uint8_t reportRate)
{
    // Checks that there was succesful communcation, not that the threshold was
    // set correctly.
    uint8_t statusByte = writeByte(OUTPUT_MODE, WRITE_SET_REPORT_RATE, reportRate);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::aecAlgoControl(uint8_t mode)
{
    if (mode == 0 || mode == 1 || mode == 2)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_AEC_ALGO, mode);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::agcAlgoControl(uint8_t mode)
{
    if (mode == 0 || mode == 1 || mode == 2)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::max86141Control(uint8_t senSwitch)
{
    if (senSwitch == 0 || senSwitch == 1)
    {
    }
    else
        return INCORR_PARAM;

    // Check that communication was successful, not that the sensor is enabled.
    uint8_t statusByte = enableWrite(ENABLE_SENSOR, ENABLE_MAX86141, senSwitch);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::maximFastAlgoControl(uint8_t mode)
{

    if (mode == 0 || mode == 1 || mode == 2)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = enableWrite(ENABLE_ALGORITHM, ENABLE_WHRM_ALGO, mode);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::readAlgoSamples()
{
    uint8_t samples = readByte(READ_ALGORITHM_CONFIG, READ_WHRM_NUM_SAMPLES, READ_WHRM_NUM_SAMPLES_ID);
    return samples;
}

uint8_t Sensor_Hub::setAlgoOpMode(uint8_t mode)
{
    if (mode >= 0 || mode <= 7)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = writeByte(ALGO_CONFIGURATION, CONFIG_WHRM, WHRM_RUN_MODE, mode);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::aecEnable(uint8_t mode)
{
    if (mode == 0 || mode == 1)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = writeByte(ALGO_CONFIGURATION, CONFIG_WHRM, WHRM_AEC_ENABLE, mode);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::autoPDEnable(uint8_t enable)
{
    if (enable == 0 || enable == 1)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = writeByte(ALGO_CONFIGURATION, CONFIG_WHRM, WHRM_AUTO_PD_ENABLE, enable);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::SCDEnable(uint8_t enable)
{
    if (enable == 0 || enable == 1)
    {
    }
    else
        return INCORR_PARAM;

    uint8_t statusByte = writeByte(ALGO_CONFIGURATION, CONFIG_WHRM, WHRM_SCD_ENABLE, enable);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

uint8_t Sensor_Hub::setAGCTarget(uint16_t current)
{
    uint8_t statusByte = writeByte(ALGO_CONFIGURATION, CONFIG_WHRM, SET_TARGET_PD, current);
    if (statusByte != SFE_BIO_SUCCESS)
        return statusByte;
    else
        return SFE_BIO_SUCCESS;
}

void Sensor_Hub::printSensorHubVersion()
{
    uint8_t versionArray[3];
    getSensorHubVersion(0xFF, 0X03, versionArray);

    Serial.print("Sensor Hub Version: ");
    Serial.println(versionArray[0]);
    Serial.println(versionArray[1]);
    Serial.println(versionArray[2]);
}

//-------------------Private Functions-----------------------

// This function uses the given family, index, and write byte to enable
// the given sensor.
uint8_t
Sensor_Hub::enableWrite(uint8_t _familyByte, uint8_t _indexByte,
                        uint8_t _enableByte)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_enableByte);
    _i2cPort->endTransmission();

    if (_familyByte == ENABLE_SENSOR && _indexByte == ENABLE_MAX86141)
        delay(ENABLE_CMD_DELAY);
    if (_familyByte == ENABLE_ALGORITHM && _indexByte == ENABLE_WHRM_ALGO)
        _enableByte == 0 ? delay(ALGO_CMD_DELAY_SHORT) : delay(ALGO_CMD_DELAY_LONG);

    // Status Byte, success or no? 0x00 is a successful transmit
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

uint8_t Sensor_Hub::enableWrite(uint8_t _familyByte, uint8_t _indexByte,
                                uint8_t _enableByte, uint8_t _writeByte)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_enableByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();

    if (_familyByte == ENABLE_SENSOR && _indexByte == ENABLE_MAX86141)
        delay(ENABLE_CMD_DELAY);
    else
        delay(ENABLE_CMD_DELAY);

    // Status Byte, success or no? 0x00 is a successful transmit
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
                              uint8_t _writeByte)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    // Status Byte, success or no? 0x00 is a successful transmit
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function is the same as the function above and uses the given family,
// index, and write byte, but also takes a 16 bit integer as a paramter to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
                              uint8_t _writeByte, uint16_t _val)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->write((_val >> 8)); // MSB
    _i2cPort->write(_val);        // LSB
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    // Status Byte, success or no? 0x00 is a successful transmit
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
                              uint8_t _writeByte, uint8_t _writeVal)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->write(_writeVal);
    _i2cPort->endTransmission();

    _familyByte == 0x44 ? delay(20) : delay(CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Sensor_Hub::writeLongBytes(uint8_t _familyByte, uint8_t _indexByte,
                                   uint8_t _writeByte, int32_t _writeVal[], const size_t _size)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);

    for (size_t i = 0; i < _size; i++)
    {
        _i2cPort->write(_writeVal[i] >> 24);
        _i2cPort->write(_writeVal[i] >> 16);
        _i2cPort->write(_writeVal[i] >> 8);
        _i2cPort->write(_writeVal[i]);
    }

    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function sends information to the MAX32664 to specifically write values
// to the registers of downward sensors and so also requires a
// register address and register value as parameters. Again there is the write
// of the specific bytes followed by a read to confirm positive transmission.
uint8_t Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte,
                               uint8_t _writeByte, uint8_t _writeVal[], size_t _size)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);

    for (size_t i = 0; i < _size; i++)
    {
        _i2cPort->write(_writeVal[i]);
    }

    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

uint8_t Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte,
                               uint8_t _writeByte, uint8_t _writeVal, uint8_t cmdDelay)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->write(_writeVal);
    _i2cPort->endTransmission();
    delay(cmdDelay);

    // Status Byte, 0x00 is a successful transmit.
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte an index byte, and
// then delays 60 microseconds, during which the MAX32664 retrieves the requested
// information. An I-squared-C request is then issued, and the information is read.
uint8_t Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte)
{

    uint8_t returnByte;
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _i2cPort->read();
    return returnByte; // If good then return the actual byte.
}

void Sensor_Hub::getSensorHubVersion(uint8_t _familyByte, uint8_t _indexByte, uint8_t *versionArray)
{
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(3) + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
    }

    versionArray[0] = _i2cPort->read();
    versionArray[1] = _i2cPort->read();
    versionArray[2] = _i2cPort->read();
}

// This function is exactly as the one above except it accepts also receives a
// Write Byte as a parameter. It starts a request by writing the family byte, index byte, and
// write byte to the MAX32664 and then delays 60 microseconds, during which
// the MAX32664 retrieves the requested information. A I-squared-C request is
// then issued, and the information is read.
uint8_t Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte,
                             uint8_t _writeByte)
{

    uint8_t returnByte;
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _i2cPort->read();
    return returnByte; // If good then return the actual byte.
}

uint8_t Sensor_Hub::readRegister(uint8_t _familyByte, uint8_t _indexByte,
                                 uint8_t _registerByte)
{

    uint8_t returnByte;
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_registerByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = _i2cPort->read();
    return returnByte; // If good then return the actual byte.
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This differs from the above read commands in
// that it returns a 16 bit integer instead of a single byte.
uint16_t Sensor_Hub::readIntByte(uint8_t _familyByte, uint8_t _indexByte,
                                 uint8_t _writeByte)
{

    uint16_t returnByte;
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(returnByte) + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }

    returnByte = (_i2cPort->read() << 8);
    returnByte |= _i2cPort->read();

    return returnByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, const size_t _numOfReads,
                                      int32_t userArray[])
{

    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(int32_t) * _numOfReads + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }
    else
    {
        for (size_t i = 0; i < (sizeof(int32_t) * _numOfReads); i++)
        {
            userArray[i] = _i2cPort->read() << 24;
            userArray[i] |= _i2cPort->read() << 16;
            userArray[i] |= _i2cPort->read() << 8;
            userArray[i] |= _i2cPort->read();
        }
        return statusByte;
    }
}

uint8_t Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte, const size_t _numOfReads,
                                      std::vector<int32_t> &userVector)
{
    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    size_t bytesToRead = sizeof(int32_t) * _numOfReads + sizeof(statusByte);
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(bytesToRead));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }
    else
    {
        userVector.clear();              // Clear the vector before populating it
        userVector.reserve(_numOfReads); // Reserve space for efficiency
        for (size_t i = 0; i < _numOfReads; i++)
        {
            int32_t value = 0;
            value |= static_cast<int32_t>(_i2cPort->read()) << 24;
            value |= static_cast<int32_t>(_i2cPort->read()) << 16;
            value |= static_cast<int32_t>(_i2cPort->read()) << 8;
            value |= static_cast<int32_t>(_i2cPort->read());
            userVector.push_back(value); // Add the parsed value to the vector
        }
        return statusByte;
    }
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte,
                                      uint8_t _writeByte, const size_t _numOfReads,
                                      uint8_t userArray[])
{

    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(_numOfReads + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // SFE_BIO_SUCCESS (0x00)
    {
        Serial.print("READ ERROR: ");
        Serial.println(statusByte);
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE
    }
    else
    {
        for (size_t i = 0; i < _numOfReads; i++)
        {
            userArray[i] = _i2cPort->read();
        }
        return statusByte;
    }
}