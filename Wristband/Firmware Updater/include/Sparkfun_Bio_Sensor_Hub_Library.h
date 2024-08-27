#ifndef _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_
#define _SPARKFUN_BIO_SENSOR_HUB_LIBRARY_H_

#include <Wire.h>
#include <Arduino.h>
#include <vector>
#include <fstream>
#include <msbl_data.h>

#define WRITE_FIFO_INPUT_BYTE 0x04
#define DISABLE 0x00
#define ENABLE 0x01
#define MODE_ONE 0x01
#define MODE_TWO 0x02
#define APP_MODE 0x00
#define BOOTLOADER_MODE 0x08
#define NO_WRITE 0x00
#define INCORR_PARAM 0xEE

#define CONFIGURATION_REGISTER 0x0A
#define PULSE_MASK 0xFC
#define READ_PULSE_MASK 0x03
#define SAMP_MASK 0xE3
#define READ_SAMP_MASK 0x1C
#define ADC_MASK 0x9F
#define READ_ADC_MASK 0x60

#define ENABLE_CMD_DELAY 45     // Milliseconds
#define ALGO_CMD_DELAY_SHORT 45 // Milliseconds
#define ALGO_CMD_DELAY_LONG 45  // Milliseconds
#define CMD_DELAY 700           // Milliseconds
#define MAXFAST_ARRAY_SIZE 6    // Number of bytes....
#define MAXFAST_EXTENDED_DATA 5
#define MAX30101_LED_ARRAY 12 // 4 values of 24 bit (3 byte) LED values

#define SET_FORMAT 0x00
#define READ_FORMAT 0x01         // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD 0x01 // Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 0x00

const uint8_t BIO_ADDRESS = 0x55;

struct bioData
{

    uint32_t irLed;
    uint32_t redLed;
    uint16_t heartRate;  // LSB = 0.1bpm
    uint8_t confidence;  // 0-100% LSB = 1%
    uint16_t oxygen;     // 0-100% LSB = 1%
    uint8_t status;      // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
    float rValue;        // -- Algorithm Mode 2 vv
    int8_t extStatus;    // --
    uint8_t reserveOne;  // --
    uint8_t resserveTwo; // -- Algorithm Mode 2 ^^
};

struct version
{
    // 3 bytes total
    uint8_t major;
    uint8_t minor;
    uint8_t revision;
};

struct sensorAttr
{

    uint8_t byteWord;
    uint8_t availRegisters;
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
    SET_DEVICE_MODE,
    READ_DEVICE_MODE,
    OUTPUT_MODE = 0x10,
    READ_OUTPUT_MODE,
    READ_DATA_OUTPUT,
    READ_DATA_INPUT,
    WRITE_INPUT,
    WRITE_REGISTER = 0x40,
    READ_REGISTER,
    READ_ATTRIBUTES_AFE,
    DUMP_REGISTERS,
    ENABLE_SENSOR,
    READ_SENSOR_MODE,
    CHANGE_ALGORITHM_CONFIG = 0x50,
    READ_ALGORITHM_CONFIG,
    ENABLE_ALGORITHM,
    BOOTLOADER_FLASH = 0x80,
    BOOTLOADER_INFO,
    IDENTITY = 0xFF

};

// All the defines below are: 1. Index Bytes nestled in the larger category of the
// family registry bytes listed above and 2. The Write Bytes nestled even
// farther under their Index Bytes.

// Write Bytes under Family Byte: SET_DEVICE_MODE (0x01) and Index
// Byte: 0x00.
enum DEVICE_MODE_WRITE_BYTES
{

    EXIT_BOOTLOADER = 0x00,
    SFE_BIO_RESET = 0x02,
    ENTER_BOOTLOADER = 0x08

};

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

// Index Byte under the Family Byte: BOOTLOADER_FLASH (0x80).
enum BOOTLOADER_FLASH_INDEX_BYTE
{

    SET_INIT_VECTOR_BYTES = 0x00,
    SET_AUTH_BYTES,
    SET_NUM_PAGES,
    ERASE_FLASH,
    SEND_PAGE_VALUE

};

// Index Byte under the Family Byte: BOOTLOADER_INFO (0x81).
enum BOOTLOADER_INFO_INDEX_BYTE
{

    BOOTLOADER_VERS = 0x00,
    PAGE_SIZE

};

// Index Byte under the Family Byte: IDENTITY (0xFF).
enum IDENTITY_INDEX_BYTES
{

    READ_MCU_TYPE = 0x00,
    READ_SENSOR_HUB_VERS = 0x03,
    READ_ALGO_VERS = 0x07

};

class SparkFun_Bio_Sensor_Hub
{
public:
    // Variables ------------
    uint8_t numPages;
    uint16_t pageSize;
    std::vector<uint8_t> initVectorBytes;
    std::vector<uint8_t> authBytes;
    std::vector<std::vector<uint8_t>> pages;

    // Constructor ----------
    SparkFun_Bio_Sensor_Hub(int resetPin = -1, int mfioPin = -1, uint8_t address = 0x55);

    // Functions ------------

    // Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
    // The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
    // bootloader mode, the MFIO pin must be pulled LOW while the board is held
    // in reset for 10ms. After 50 addtional ms have elapsed the board should be
    // in bootloader mode and will return two bytes, the first 0x00 is a
    // successful communcation byte, followed by 0x08 which is the byte indicating
    // that the board is in bootloader mode.
    uint8_t beginBootloader(TwoWire &wirePort = Wire, int resetPin = -1, int mfioPin = -1);

    // Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
    // The following function checks the status of the FIFO.
    uint8_t readSensorHubStatus();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
    version readBootloaderVers();

    // Family Byte: BOOTLOADER_INFO (0x81), Index Byte: PAGE_SIZE (0x01)
    // Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
    version readSensorHubVersion();

    // Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
    version readAlgorithmVersion();

    uint8_t readBootloaderMode();

    uint8_t setBootloaderMode(uint8_t);

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
    // Returns true on successful communication.
    bool eraseFlash();

    uint8_t readPageSize();

    bool getNumPages();

    // Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
    // Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
    uint8_t setNumPages();

    // Get initial bytes
    void getInitBytes();

    // Set initial bytes
    uint8_t setInitBytes();

    // Get authentication bytes
    void getAuthBytes();

    // Set authentication bytes
    uint8_t setAuthBytes();

    // Populate pages with bytes
    void populatePageBytes();

    // Send page bytes
    uint8_t sendPageBytes();

    // Get data from specified file locations
    std::vector<uint8_t> getData(std::streampos, std::streampos);

    uint8_t getValueAtAddress(uint8_t);

    uint8_t getValueAtAddress16(uint16_t);

    // Sequence to flash firmware to MAX32664
    uint8_t flashApplication();

private:
    // Variables -----------
    int _resetPin;
    int _mfioPin;
    uint8_t _address;

    // I-squared-C Class----
    TwoWire *_i2cPort;

    // Functions------------

    // This function uses the given family, index, and write byte to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t);

    // This function sends is simliar to the one above and sends info to the MAX32664
    // but takes an additional uint8_t as a paramter. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint8_t);

    // This function is the same as the function above and uses the given family,
    // index, and write byte, but also takes a 16 bit integer as a paramter to communicate
    // with the MAX32664 which in turn communicates with downward sensors. There
    // are two steps demonstrated in this function. First a write to the MCU
    // indicating what you want to do, a delay, and then a read to confirm positive
    // transmission.
    uint8_t writeByte(uint8_t, uint8_t, uint8_t, uint16_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeLongBytes(uint8_t, uint8_t, uint8_t, int32_t _writeVal[], const size_t);

    // This function sends information to the MAX32664 to specifically write values
    // to the registers of downward sensors and so also requires a
    // register address and register value as parameters. Again there is the write
    // of the specific bytes followed by a read to confirm positive transmission.
    uint8_t writeBytes(uint8_t, uint8_t, uint8_t, uint8_t _writeVal[], const size_t);

    uint8_t writeBytes(uint8_t, uint8_t, std::vector<uint8_t>);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, index byte, and
    // delays 60 microseconds, during which the MAX32664 retrieves the requested
    // information. An I-squared-C request is then issued, and the information is read and returned.
    uint8_t readByte(uint8_t, uint8_t);

    // This function is exactly as the one above except it accepts a Write Byte as
    // a paramter. It starts a request by writing the family byte, index byte, and
    // write byte to the MAX32664, delays 60 microseconds, during which
    // the MAX32664 retrieves the requested information. A I-squared-C request is
    // then issued, and the information is read and returned.
    uint8_t readByte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This differs from the above read commands in
    // that it returns a 16 bit integer instead of 8.
    uint16_t readIntByte(uint8_t, uint8_t, uint8_t);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns three uint32_t bytes instead of one.
    uint8_t readMultipleBytes(uint8_t, uint8_t, uint8_t, const size_t, int32_t userArray[]);

    // This function handles all read commands or stated another way, all information
    // requests. It starts a request by writing the family byte, an index byte, and
    // a write byte and then then delays 60 microseconds, during which the MAX32664
    // retrieves the requested information. An I-squared-C request is then issued,
    // and the information is read. This function is very similar to the one above
    // except it returns multiple requested bytes.
    uint8_t readMultipleBytes(uint8_t, uint8_t, uint8_t, const size_t, uint8_t userArray[]);

    // Needs comment - INCOMPLETE
    uint8_t readFillArray(uint8_t, uint8_t, uint8_t, uint8_t array[]);
};
#endif