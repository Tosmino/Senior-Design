/*
  This is an Arduino Library written for the MAXIM 32664 Biometric Sensor Hub
  The MAX32664 Biometric Sensor Hub is in actuality a small Cortex M4 microcontroller
  with pre-loaded firmware and algorithms used to interact with the a number of MAXIM
  sensors; specifically the MAX30101 Pulse Oximter and Heart Rate Monitor and
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

#include "SparkFun_Bio_Sensor_Hub_Library.h"

SparkFun_Bio_Sensor_Hub::SparkFun_Bio_Sensor_Hub(int resetPin, int mfioPin, uint8_t address)
{

    _resetPin = resetPin;
    if (resetPin >= 0)
        pinMode(_resetPin, OUTPUT); // Set the pin as output

    _mfioPin = mfioPin;
    if (mfioPin >= 0)
        pinMode(_mfioPin, OUTPUT); // Set the pin as output

    _address = address;
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
uint8_t SparkFun_Bio_Sensor_Hub::beginBootloader(TwoWire &wirePort, int resetPin, int mfioPin)
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

    digitalWrite(_mfioPin, LOW);
    digitalWrite(_resetPin, LOW);
    delay(10);
    digitalWrite(_resetPin, HIGH);
    delay(50); // Bootloader mode is enabled when this ends.
    pinMode(_resetPin, OUTPUT);
    pinMode(_mfioPin, OUTPUT);

    // Let's check to see if the device made it into bootloader mode.
    uint8_t responseByte = readByte(READ_DEVICE_MODE, 0x00); // 0x00 only possible Index Byte
    return responseByte;
}

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
uint8_t SparkFun_Bio_Sensor_Hub::readSensorHubStatus()
{

    uint8_t status = readByte(0x00, 0x00); // Just family and index byte.
    return status;                         // Will return 0x00
}

// Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
version SparkFun_Bio_Sensor_Hub::readBootloaderVers()
{

    version booVers; // BOO!
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(BOOTLOADER_INFO);
    _i2cPort->write(BOOTLOADER_VERS);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t statusByte = _i2cPort->read();
    if (statusByte)
    { // Pass through if SFE_BIO_SUCCESS (0x00).
        booVers.major = 0;
        booVers.minor = 0;
        booVers.revision = 0;
        return booVers;
    }

    booVers.major = _i2cPort->read();
    booVers.minor = _i2cPort->read();
    booVers.revision = _i2cPort->read();

    return booVers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
version SparkFun_Bio_Sensor_Hub::readSensorHubVersion()
{

    version bioHubVers;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(IDENTITY);
    _i2cPort->write(READ_SENSOR_HUB_VERS);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t statusByte = _i2cPort->read();
    if (statusByte)
    { // Pass through if SFE_BIO_SUCCESS (0x00).
        bioHubVers.major = 0;
        bioHubVers.minor = 0;
        bioHubVers.revision = 0;
        return bioHubVers;
    }

    bioHubVers.major = _i2cPort->read();
    bioHubVers.minor = _i2cPort->read();
    bioHubVers.revision = _i2cPort->read();

    return bioHubVers;
}

// Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
version SparkFun_Bio_Sensor_Hub::readAlgorithmVersion()
{

    version libAlgoVers;
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(IDENTITY);
    _i2cPort->write(READ_ALGO_VERS);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(4));
    uint8_t statusByte = _i2cPort->read();
    if (statusByte)
    { // Pass through if SFE_BIO_SUCCESS (0x00).
        libAlgoVers.major = 0;
        libAlgoVers.minor = 0;
        libAlgoVers.revision = 0;
        return libAlgoVers;
    }

    libAlgoVers.major = _i2cPort->read();
    libAlgoVers.minor = _i2cPort->read();
    libAlgoVers.revision = _i2cPort->read();

    return libAlgoVers;
}

// Read device mode and outputs to Serial
uint8_t SparkFun_Bio_Sensor_Hub::readBootloaderMode()
{
    uint8_t statusByte = readByte(READ_DEVICE_MODE, 0x00);
    Serial.print("Device mode: ");
    Serial.println(statusByte, HEX);
    return statusByte;
}

// Set the Bootloader mode to desired mode
uint8_t SparkFun_Bio_Sensor_Hub::setBootloaderMode(uint8_t mode)
{
    uint8_t statusByte = writeByte(SET_DEVICE_MODE, 0x00, mode);
    return statusByte;
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
// Returns true on successful communication.
bool SparkFun_Bio_Sensor_Hub::eraseFlash()
{

    // This is a unique write in that it does not have a relevant write byte.
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(BOOTLOADER_FLASH);
    _i2cPort->write(ERASE_FLASH);
    _i2cPort->endTransmission();
    delay(1400);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    if (!statusByte)
        return true;
    else
        return false;
}

// Reads page size of bio hub, sets pageSize to uint16_t value
uint8_t SparkFun_Bio_Sensor_Hub::readPageSize()
{
    uint8_t userArray[2];
    uint8_t arraySize = 2;
    uint8_t statusByte = readFillArray(BOOTLOADER_INFO, PAGE_SIZE, arraySize, userArray);
    Serial.print("Page size: ");
    pageSize = ((uint16_t)userArray[0] << 8) | userArray[1];
    pageSize += 16;
    Serial.println(pageSize);
    return statusByte;
}

// Gets num pages found at msbl 0x44, writes to numPages, returns true on success
bool SparkFun_Bio_Sensor_Hub::getNumPages()
{
    numPages = getValueAtAddress(0x44);

    Serial.print("Number of Pages: ");
    Serial.println(numPages, HEX);

    return true; // Return true to indicate success
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
// Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
uint8_t SparkFun_Bio_Sensor_Hub::setNumPages()
{

    uint8_t statusByte = writeByte(BOOTLOADER_FLASH, SET_NUM_PAGES, 0x00, numPages);
    return statusByte;
}

void SparkFun_Bio_Sensor_Hub::getInitBytes()
{
    initVectorBytes = getData(0x28, 0x32);
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_INIT_VECTOR_BYTES (0x00)
uint8_t SparkFun_Bio_Sensor_Hub::setInitBytes()
{
    uint8_t statusByte = writeBytes(BOOTLOADER_FLASH, SET_INIT_VECTOR_BYTES, initVectorBytes);
    return statusByte;
}

void SparkFun_Bio_Sensor_Hub::getAuthBytes()
{
    authBytes = getData(0x34, 0x43);
}

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_AUTH_BYTES (0x01)
uint8_t SparkFun_Bio_Sensor_Hub::setAuthBytes()
{
    uint8_t statusByte = writeBytes(BOOTLOADER_FLASH, SET_AUTH_BYTES, authBytes);
    return statusByte;
}

// Populates vector<vector<uint8_t>> with page bytes from .msbl file
void SparkFun_Bio_Sensor_Hub::populatePageBytes()
{
    std::vector<uint8_t> pageData;
    uint8_t statusChauf;
    std::streampos startAddress = 0x004C;
    std::streampos endAddress = 0x004C + pageSize - 1;
    Serial.print("Start Address: ");
    Serial.println(startAddress, HEX);
    Serial.print("End Address: ");
    Serial.println(endAddress, HEX);

    for (int i = 0; i < numPages; ++i)
    {
        pageData = getData((0x004C + i * pageSize), (0x004C + (i + 1) * pageSize - 1));
        statusChauf = writeBytes(BOOTLOADER_FLASH, SEND_PAGE_VALUE, pageData);
        if (statusChauf != 0)
        {
            Serial.print("Page Write Failed, error: ");
            Serial.println(statusChauf);
        }
        // pages.push_back(pageData);
        pageData.clear();
    }
    Serial.println("All pages pushed back successfully");
}

// Sends page bytes in uint8_t vector via i2c to MAX32664
uint8_t SparkFun_Bio_Sensor_Hub::sendPageBytes()
{
    uint8_t statusByte;
    int currentPage = 1;
    for (auto i = pages.begin(); i != pages.end(); i++)
    {
        statusByte = writeBytes(BOOTLOADER_FLASH, SEND_PAGE_VALUE, *i);
        Serial.print("Writing pages... (");
        Serial.print((currentPage / numPages) * 100);
        Serial.println("%)");
        if (statusByte != 0)
            return statusByte;
    }
    return statusByte;
}

// Read data from .msbl file from starting address to end address, stores in uint8_t vector
std::vector<uint8_t> SparkFun_Bio_Sensor_Hub::getData(std::streampos startAddr, std::streampos endAddr)
{
    // Check if the start and end addresses are within bounds
    if (startAddr < 0 || endAddr >= sizeof(msbl_data) || startAddr > endAddr)
    {
        Serial.println("Invalid start or end address.");
        return {}; // Return an empty vector to indicate failure
    }

    // Calculate the number of elements to read
    std::streamsize numElements = endAddr - startAddr + 1;

    // Copy the data from msbl_data to a vector of uint8_t
    std::vector<uint8_t> data(msbl_data + startAddr, msbl_data + endAddr + 1);

    return data;
}

uint8_t SparkFun_Bio_Sensor_Hub::getValueAtAddress(uint8_t address)
{
    uint8_t value;

    // Offset to read the value at 0x44
    const int offset = address;

    // Check if the offset is within bounds of msbl_data array
    if (offset + sizeof(value) > sizeof(msbl_data))
    {
        Serial.println("Offset is out of bounds.");
        return false; // Return false to indicate failure
    }

    // Read the value at offset 0x44 from msbl_data
    memcpy(&value, &msbl_data[offset], sizeof(value));

    return value;
}

uint8_t SparkFun_Bio_Sensor_Hub::getValueAtAddress16(uint16_t address)
{
    uint8_t value;

    // Offset to read the value
    const int offset = address;

    // Check if the offset is within bounds of msbl_data array
    if (offset + sizeof(value) > sizeof(msbl_data))
    {
        Serial.println("Offset is out of bounds.");
        return false; // Return false to indicate failure
    }

    // Read the value at offset 0x44 from msbl_data
    memcpy(&value, &msbl_data[offset], sizeof(value));

    return value;
}

// Sequence function to flash new firmware to MAX32664
uint8_t SparkFun_Bio_Sensor_Hub::flashApplication()
{
    // Initialize variables
    getNumPages();
    getInitBytes();
    Serial.print("Value at 0x205B: ");
    Serial.println(getValueAtAddress16(0x205B), HEX);
    version bootLoader;
    version sensorHub;
    uint8_t statusChauf;

    // Read mode
    statusChauf = readBootloaderMode();

    delay(100);

    // If not in Bootloader mode already, then put in Bootloader mode
    if (statusChauf != 8)
    {
        statusChauf = setBootloaderMode(ENTER_BOOTLOADER);
        if (statusChauf != 0)
        {
            Serial.println("Failed to enter Bootloader");
            return statusChauf;
        }
    }

    Serial.println("Entered Bootloader");

    // Read version
    readBootloaderVers();
    Serial.print("Bootloader version: ");
    Serial.println(bootLoader.major);
    Serial.println(bootLoader.minor);
    Serial.println(bootLoader.revision);

    delay(100);

    // Read bootloader page size
    statusChauf = readPageSize();
    if (statusChauf != 0)
    {
        Serial.println("Failed to Read Page Size");
        return statusChauf;
    }
    Serial.println("Page Size Read");

    delay(100);

    // Set num pages to value found at 0x44
    statusChauf = setNumPages();
    if (statusChauf != 0)
    {
        Serial.println("Failed to Set Number of Pages");
        return statusChauf;
    }
    Serial.println("Number of Pages Set");

    delay(100);

    // Set init vector bytes to values 0x28 to 0x32
    statusChauf = setInitBytes();
    if (statusChauf != 0)
    {
        Serial.println("Failed to Set Initial Bytes");
        return statusChauf;
    }
    Serial.println("Initial Bytes Set");

    delay(100);

    getAuthBytes();

    delay(100);

    // Set auth bytes to values 0x34 to 0x43
    statusChauf = setAuthBytes();
    if (statusChauf != 0)
    {
        Serial.println("Failed to Set Authentication Bytes");
        Serial.println(statusChauf);
        return statusChauf;
    }
    Serial.println("Authentication Bytes Set");

    delay(100);

    // Erase application
    if (!eraseFlash())
    {
        Serial.println("Failed to Erase Flash");
        return 255;
    }
    Serial.println("Flash Erased Successfully");

    delay(5000);

    // Populate pages
    populatePageBytes();

    Serial.println("Page Bytes Populated");

    delay(100);

    // Based on number of pages, enter for loop
    // statusChauf = sendPageBytes();
    // if (statusChauf != 0)
    // {
    //     Serial.println("Failed to Send Page Bytes");
    //     return statusChauf;
    // }
    // Serial.println("Page Bytes Sent");

    delay(100);

    // Exit Bootloader mode and enter application mode
    statusChauf = setBootloaderMode(EXIT_BOOTLOADER);
    if (statusChauf != 0)
    {
        Serial.println("Failed to exit Bootloader");
        return statusChauf;
    }

    sensorHub = readSensorHubVersion();
    Serial.print("Sensor Hub Version: ");
    Serial.print(sensorHub.major);
    Serial.print(".");
    Serial.print(sensorHub.minor);
    Serial.print(".");
    Serial.println(sensorHub.revision);

    Serial.println("Exiting Bootloader Mode...");

    delay(5000);

    Serial.println("Device Now in Application Mode");

    return statusChauf;
}

//-------------------Private Functions-----------------------

// This function uses the given family, index, and write byte to communicate
// with the MAX32664 which in turn communicates with downward sensors. There
// are two steps demonstrated in this function. First a write to the MCU
// indicating what you want to do, a delay, and then a read to confirm positive
// transmission.
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
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
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
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
uint8_t SparkFun_Bio_Sensor_Hub::writeByte(uint8_t _familyByte, uint8_t _indexByte,
                                           uint8_t _writeByte, uint8_t _writeVal)
{

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->write(_writeVal);
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
uint8_t SparkFun_Bio_Sensor_Hub::writeLongBytes(uint8_t _familyByte, uint8_t _indexByte,
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
uint8_t SparkFun_Bio_Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte,
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

// This function writes the contents of a vector to the MAX32664
uint8_t SparkFun_Bio_Sensor_Hub::writeBytes(uint8_t _familyByte, uint8_t _indexByte, std::vector<uint8_t> vect)
{
    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);

    for (auto i = vect.begin(); i != vect.end(); i++)
    {
        _i2cPort->write(*i);
    }

    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    // Status Byte, 0x00 is a successful transmit.
    _i2cPort->requestFrom(_address, static_cast<uint8_t>(1));
    uint8_t statusByte = _i2cPort->read();
    return statusByte;
}

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte an index byte, and
// then delays 60 microseconds, during which the MAX32664 retrieves the requested
// information. An I-squared-C request is then issued, and the information is read.
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte)
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
    if (statusByte)        // SFE_BIO_SUCCESS (0x00) - how do I know its
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE

    returnByte = _i2cPort->read();
    return returnByte; // If good then return the actual byte.
}

// This function is exactly as the one above except it accepts also receives a
// Write Byte as a parameter. It starts a request by writing the family byte, index byte, and
// write byte to the MAX32664 and then delays 60 microseconds, during which
// the MAX32664 retrieves the requested information. A I-squared-C request is
// then issued, and the information is read.
uint8_t SparkFun_Bio_Sensor_Hub::readByte(uint8_t _familyByte, uint8_t _indexByte,
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
    if (statusByte)        // SFE_BIO_SUCCESS (0x00)
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE

    returnByte = _i2cPort->read();
    return returnByte; // If good then return the actual byte.
}

uint8_t SparkFun_Bio_Sensor_Hub::readFillArray(uint8_t _familyByte, uint8_t _indexByte,
                                               uint8_t _numOfReads, uint8_t array[])
{

    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(_numOfReads + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte)
    { // SFE_BIO_SUCCESS: 0x00
        for (size_t i = 0; i < _numOfReads; i++)
        {
            array[i] = 0;
        }
        return statusByte;
    }

    for (size_t i = 0; i < _numOfReads; i++)
    {
        array[i] = _i2cPort->read();
    }
    return statusByte;
}
// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This differs from the above read commands in
// that it returns a 16 bit integer instead of a single byte.
uint16_t SparkFun_Bio_Sensor_Hub::readIntByte(uint8_t _familyByte, uint8_t _indexByte,
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
    if (statusByte)        // Pass through if SFE_BIO_SUCCESS (0x00).
        return statusByte; // Return the error, see: READ_STATUS_BYTE_VALUE

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
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte,
                                                   uint8_t _writeByte, const size_t _numOfReads,
                                                   int32_t userArray[])
{

    uint8_t statusByte;

    _i2cPort->beginTransmission(_address);
    _i2cPort->write(_familyByte);
    _i2cPort->write(_indexByte);
    _i2cPort->write(_writeByte);
    _i2cPort->endTransmission();
    delay(CMD_DELAY);

    _i2cPort->requestFrom(_address, static_cast<uint8_t>(sizeof(int32_t) * _numOfReads + sizeof(statusByte)));
    statusByte = _i2cPort->read();
    if (statusByte) // Pass through if SFE_BIO_SUCCESS (0x00).
        return statusByte;
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

// This function handles all read commands or stated another way, all information
// requests. It starts a request by writing the family byte, an index byte, and
// a write byte and then then delays 60 microseconds, during which the MAX32664
// retrieves the requested information. An I-squared-C request is then issued,
// and the information is read. This function is very similar to the one above
// except it returns three uint32_t bytes instead of one.
uint8_t SparkFun_Bio_Sensor_Hub::readMultipleBytes(uint8_t _familyByte, uint8_t _indexByte,
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
    if (statusByte) // Pass through if SFE_BIO_SUCCESS (0x00).
        return statusByte;
    else
    {
        for (size_t i = 0; i < _numOfReads; i++)
        {
            userArray[i] = _i2cPort->read();
        }
        return statusByte;
    }
}