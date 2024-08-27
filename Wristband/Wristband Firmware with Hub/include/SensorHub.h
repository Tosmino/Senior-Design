#ifndef _SensorHub_H
#define _SensorHub_H

#include <Wire.h>
#include <Arduino.h>
#include <vector>

#define DISABLE 0x00
#define ENABLE 0x01
#define INCORR_PARAM 0xEE

#define ENABLE_CMD_DELAY 250     // Milliseconds
#define ENABLE_ACC_DELAY 20      // Milliseconds
#define ALGO_CMD_DELAY_SHORT 120 // Milliseconds
#define ALGO_CMD_DELAY_LONG 320  // Milliseconds
#define CMD_DELAY 2              // Milliseconds
#define SAMPLE_DELAY 10          // Milliseconds

#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define SET_FORMAT 0x00
#define WRITE_SET_THRESHOLD 0x01 // Index Byte for WRITE_INPUT(0x14)
#define WRITE_SET_REPORT_RATE 0X02

const uint8_t BIO_ADDRESS = 0x55;

struct bioData
{
    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate; // LSB = 0.1bpm
    uint8_t confidence; // 0-100% LSB = 1%
    uint16_t oxygen;    // 0-100% LSB = 1%
    uint8_t status;     // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
};

struct rawData
{
    uint32_t greenCount1;
    uint32_t greenCount2;
    uint32_t IRCount;
    uint32_t redCount;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
};

// Status Bytes are communicated back after every I-squared-C transmission and
// are indicators of success or failure of the previous transmission.
enum READ_STATUS_BYTE_VALUE
{

    SFE_BIO_SUCCESS = 0x00,
    ERR_UNAVAIL_CMD,
    ERR_UNAVAIL_FUNC,
    ERR_DATA_FORMAT,
    ERR_INPUT_VALUE,
    ERR_TRY_AGAIN,
    ERR_BTLDR_GENERAL = 0x80,
    ERR_BTLDR_CHECKSUM,
    ERR_BTLDR_AUTH,
    ERR_BTLDR_INVALID_APP,
    ERR_UNKNOWN = 0xFF

};

// The family register bytes are the larger umbrella for all the Index and
// Write Bytes listed below. You can not reference a nestled byte without first
// referencing it's larger category: Family Register Byte.
enum FAMILY_REGISTER_BYTES
{
    HUB_STATUS = 0x00,
    WRITE_DEVICE_MODE,
    READ_DEVICE_MODE,
    OUTPUT_MODE = 0X10,
    READ_OUTPUT_MODE,
    ENABLE_SENSOR = 0x44,
    READ_SENSOR_MODE,
    ALGO_CONFIGURATION = 0x50,
    READ_ALGORITHM_CONFIG,
    ENABLE_ALGORITHM
};

// All the defines below are: 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes nestled even
// farther under their Index Bytes.

// Write Bytes under Family Byte: OUTPUT_MODE (0x10) and Index byte: SET_FORMAT
// (0x00)
enum OUTPUT_MODE_WRITE_BYTE
{

    PAUSE = 0x00,
    SENSOR_DATA,
    ALGO_DATA,
    SENSOR_AND_ALGORITHM,
    PAUSE_TWO,
    SENSOR_COUNTER_BYTE,
    ALGO_COUNTER_BYTE,
    SENSOR_ALGO_COUNTER

};

enum SENSOR_ENABLE_BYTE
{

    ENABLE_MAX86141,
    ENABLE_ACCELEROMETER = 0x04

};

// Index Byte under the Family Byte: ENABLE_ALGORITHM (0x52).
enum ALGORITHM_MODE_ENABLE_INDEX_BYTE
{

    ENABLE_AGC_ALGO = 0x00,
    ENABLE_AEC_ALGO,
    ENABLE_WHRM_ALGO = 0X07

};

enum READ_ALGORITHM_INDEX_BYTE
{

    READ_WHRM_NUM_SAMPLES = 0x07,
    READ_WHRM_NUM_SAMPLES_ID = 0x14

};

enum READ_AGC_ALGO_WRITE_BYTE
{

    READ_AGC_PERC_ID = 0x00,
    READ_AGC_STEP_SIZE_ID,
    READ_AGC_SENSITIVITY_ID,
    READ_AGC_NUM_SAMPLES_ID,
    READ_MAX_FAST_COEF_ID = 0x0B

};

enum WHRM_CONFIG_BYTE
{
    CONFIG_WHRM = 0x07,
    WHRM_RUN_MODE = 0x0A,
    WHRM_AEC_ENABLE,
    WHRM_SCD_ENABLE,
    SET_TARGET_PD = 0x11,
    WHRM_AUTO_PD_ENABLE
};

class Sensor_Hub
{
public:
    // Constructor ----------
    Sensor_Hub(int resetPin = -1, int mfioPin = -1, uint8_t address = 0x55);

    // Functions ------------

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function initializes the sensor. To place the MAX32664 into
    // application mode, the MFIO pin must be pulled HIGH while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in application mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x00 which is the byte indicating
    // which mode the IC is in.
    uint8_t begin(TwoWire &wirePort = Wire, int resetPin = -1, int mfioPin = -1);

    // Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
    // The following function checks the status of the FIFO.
    uint8_t readSensorHubStatus();

    // This function sets very basic settings to get sensor and biometric data.
    uint8_t rawDataMode();

    uint8_t configAGC(uint8_t);

    std::vector<int32_t> readRawData(size_t &);

    uint8_t enableAccelerometer();

    uint8_t setSampleRate();

    uint8_t setLEDtoHalfScale(uint8_t address);

    rawData parseSample(const std::vector<int32_t> &);

    void printConfigSettings();

    uint8_t setOutputMode(uint8_t);

    uint8_t configBPM(uint8_t);

    uint8_t setFifoThreshold(uint8_t);

    uint8_t setReportRate(uint8_t);

    uint8_t aecAlgoControl(uint8_t);

    uint8_t agcAlgoControl(uint8_t);

    uint8_t max86141Control(uint8_t);

    uint8_t maximFastAlgoControl(uint8_t);

    uint8_t readAlgoSamples();

    uint8_t setAlgoOpMode(uint8_t);

    uint8_t aecEnable(uint8_t);

    uint8_t autoPDEnable(uint8_t);

    uint8_t SCDEnable(uint8_t);

    uint8_t setAGCTarget(uint16_t);

    void printSensorHubVersion();

private:
    // Variables -----------
    int _resetPin;
    int _mfioPin;
    uint8_t _address;
    uint8_t _userSelectedMode;
    uint8_t _sampleRate;

    // I-squared-C Class----
    TwoWire *_i2cPort;

    // Functions------------

    // This function uses the given family, index, and write byte to enable
    // the given sensor.
    uint8_t enableWrite(uint8_t, uint8_t, uint8_t);

    uint8_t enableWrite(uint8_t, uint8_t, uint8_t, uint8_t);

    uint8_t writeByte(uint8_t, uint8_t, uint8_t);

    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint8_t);

    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint16_t);

    uint8_t writeBytes(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);

    uint8_t writeLongBytes(uint8_t, uint8_t, uint8_t, int32_t _writeVal[], const size_t);

    uint8_t writeBytes(uint8_t, uint8_t, uint8_t, uint8_t _writeVal[], const size_t);

    uint8_t readByte(uint8_t, uint8_t);

    void getSensorHubVersion(uint8_t, uint8_t, uint8_t *);

    uint8_t readByte(uint8_t, uint8_t, uint8_t);

    uint8_t readRegister(uint8_t, uint8_t, uint8_t);

    uint16_t readIntByte(uint8_t, uint8_t, uint8_t);

    uint8_t readMultipleBytes(uint8_t, uint8_t, const size_t, int32_t userArray[]);

    uint8_t readMultipleBytes(uint8_t, uint8_t, const size_t,
                              std::vector<int32_t> &);

    uint8_t readMultipleBytes(uint8_t, uint8_t, uint8_t, const size_t, uint8_t userArray[]);
};
#endif