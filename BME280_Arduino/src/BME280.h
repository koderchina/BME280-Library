#ifndef __CAN_BUS_SOLDERED__
#define __CAN_BUS_SOLDERED__
#include <cstdint>
#include <Wire.h>
#include <Arduino.h>

const uint8_t BME280_T1_REG = 0x88; ///< Declare BME280 registers for the
const uint8_t BME280_T2_REG = 0x8A; ///< calibration data register
const uint8_t BME280_T3_REG = 0x8C; ///< calibration data register
const uint8_t BME280_P1_REG = 0x8E; ///< calibration data register
const uint8_t BME280_P2_REG = 0x90; ///< calibration data register
const uint8_t BME280_P3_REG = 0x92; ///< calibration data register
const uint8_t BME280_P4_REG = 0x94; ///< calibration data register
const uint8_t BME280_P5_REG = 0x96; ///< calibration data register
const uint8_t BME280_P6_REG = 0x98; ///< calibration data register
const uint8_t BME280_P7_REG = 0x9A; ///< calibration data register
const uint8_t BME280_P8_REG = 0x9C; ///< calibration data register
const uint8_t BME280_P9_REG = 0x9E; ///< calibration data register
const uint8_t BME280_H1_REG = 0xA1; ///< calibration data register
const uint8_t BME280_H2_REG = 0xE1; ///< calibration data register
const uint8_t BME280_H3_REG = 0xE3; ///< calibration data register
const uint8_t BME280_H4_REG = 0xE4; ///< calibration data register
const uint8_t BME280_H5_REG = 0xE5; ///< calibration data register
const uint8_t BME280_H6_REG = 0xE7; ///< calibration data register

// Register Memory Map
#define REG_HUM_LSB 0xFE
#define REG_HUM_MSB 0xFD
#define REG_TEMP_XLSB 0xFC
#define REG_TEMP_LSB 0xFB
#define REG_TEMP_MSB 0xFA
#define REG_PRESS_XLSB 0xF9
#define REG_PRESS_LSB 0xF8
#define REG_PRESS_MSB 0xF7
#define REG_CONFIG 0xF5
#define REG_CTRL_MEAS 0xF4
#define REG_STATUS 0xF3
#define REG_CTRL_HUM 0xF2
#define REG_RESET 0xE0
#define REG_ID 0xD0

#define SENSOR_ADDR 0x76
#define DEVICE_ID 0x60
#define I2C_STANDARD_MODE 100000 // 100 kHz

enum operatingMode
{
    SLEEP_MODE,
    FORCED_MODE,
    NORMAL_MODE
};

enum overSamplingType
{
    SKIP_MEASUREMENT,
    OVERSAMPLING1,
    OVERSAMPLING2,
    OVERSAMPLING4,
    OVERSAMPLING8,
    OVERSAMPLING16
};

enum sensorType
{
    HUM,
    PRESS,
    TEMP
};

enum standBySettings
{
    standByHalf,
    standBy63ms,
    standBy125ms,
    standBy250ms,
    standBy500ms,
    standBy1000ms,
    standBy10ms,
    standBy20ms
};

enum filterSettings
{
    filterOff,
    IIR2,
    IIR4,
    IIR8,
    IIR16
};

class BME280
{
public:
    BME280();
    bool begin(int32_t i2cSpeed);
    uint8_t setMode(operatingMode deviceMode);
    bool setOversampling(sensorType sensor, overSamplingType oversampling);
    bool getOversampling();
    uint8_t iirFilter(filterSettings iirSetting);
    uint8_t inactiveTime(standBySettings standByTime);
    uint8_t measurementTime(); // Do this later
    float getTemperature();
    float getPressure();
    float getHumidity();
    void reset();
    void printParams();

private:
    void readByte(byte addr, byte *data);
    uint8_t readByte(byte addr);
    /*********************************************************************************************
    ** Declare the getData methods as template functions. All device I/O is done    **
    ** through these two functions regardless of whether I2C, hardware SPI or software SPI is   **
    ** being used. The two functions are designed so that only the address and a variable are   **
    ** passed in and the functions determine the size of the parameter variable and reads or    **
    ** writes that many bytes. So if a read is called using a character array[10] then 10 bytes **
    ** are read, if called with a int8 then only one byte is read. The return value, if used,   **
    ** is the number of bytes read or written. This is done by using template function          **
    ** definitions which need to be defined in this header file rather than in the c++ program  **
    ** library file.                                                                            **
    *********************************************************************************************/
    template <typename T>
    uint8_t &getData(const byte addr, T &value)
    {
        uint8_t *bytePtr = (uint8_t *)&value;  // Pointer to structure beginning
        static uint8_t structSize = sizeof(T); // Number of bytes in structure
        Wire.beginTransmission(0x76);          // Address the I2C device
        Wire.write(addr);                      // Send register address to read
        Wire.endTransmission();                // Close transmission
        Wire.requestFrom(0x76, sizeof(T));     // Request 1 byte of data
        structSize = Wire.available();         // Use the actual number of bytes
        for (uint8_t i = 0; i < structSize; i++)
            *bytePtr++ = Wire.read(); // loop for each byte to be read
        return structSize;
    }
    void writeByte(byte addr, byte *data);
    void readSensors();
    void getCalibration();
    int8_t combineByte(uint8_t lsb);
    void readCalibrationRegisters();
    void getSensorData();
    uint8_t _cal_dig_H1, _cal_dig_H3;  ///< Calibration variables
    int8_t _cal_dig_H6 = 0;            ///< Calibration variables
    uint16_t _cal_dig_T1, _cal_dig_P1; ///< Calibration variables
    int16_t _cal_dig_T2, _cal_dig_T3, _cal_dig_P2, _cal_dig_P3, _cal_dig_P4, _cal_dig_P5, _cal_dig_P6, _cal_dig_P7,
        _cal_dig_P8, _cal_dig_P9, _cal_dig_H2, _cal_dig_H4,
        _cal_dig_H5; ///< Calibration variables
    int32_t _tfine, _Temperature, _Pressure, _Humidity;

protected:
};

#endif