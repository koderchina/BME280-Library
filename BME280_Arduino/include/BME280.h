#ifndef __CAN_BUS_SOLDERED__
#define __CAN_BUS_SOLDERED__
#include <cstdint>
#include <Wire.h>
#include <Arduino.h>

// Compensation parameter storage
#define REG_DIG_T1_LSB 0x88
#define REG_DIG_T1_MSB 0x89
#define REG_DIG_T2_LSB 0x8A
#define REG_DIG_T2_MSB 0x8B
#define REG_DIG_T3_LSB 0x8C
#define REG_DIG_T3_MSB 0x8D
#define REG_DIG_P1_LSB 0x8E
#define REG_DIG_P1_MSB 0x8F
#define REG_DIG_P2_LSB 0x90
#define REG_DIG_P2_MSB 0x91
#define REG_DIG_P3_LSB 0x92
#define REG_DIG_P3_MSB 0x93
#define REG_DIG_P4_LSB 0x94
#define REG_DIG_P4_MSB 0x95
#define REG_DIG_P5_LSB 0x96
#define REG_DIG_P5_MSB 0x97
#define REG_DIG_P6_LSB 0x98
#define REG_DIG_P6_MSB 0x99
#define REG_DIG_P7_LSB 0x9A
#define REG_DIG_P7_MSB 0x9B
#define REG_DIG_P8_LSB 0x9C
#define REG_DIG_P8_MSB 0x9D
#define REG_DIG_P9_LSB 0x9E
#define REG_DIG_P9_MSB 0x9F
#define REG_DIG_H1 0xA1
#define REG_DIG_H2_LSB 0xE1
#define REG_DIG_H2_MSB 0xE2
#define REG_DIG_H3 0xE3

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
    OVERSAMPLING3,
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

class BME280
{
public:
    BME280();
    bool begin(int32_t i2cSpeed);
    uint8_t setMode(operatingMode deviceMode);
    bool setOversampling(sensorType sensor, overSamplingType oversampling);
    bool getOversampling();
    uint8_t iirFilter();
    uint8_t inactiveTime();
    uint8_t measurementTime();
    void getSensorData();
    void reset();

private:
    void readByte(byte addr, byte *data);
    void writeByte(byte addr, byte *data);
    void readSensors();
    void getCalibration();

protected:
};

#endif