#include "BME280.h"

BME280::BME280()
{
}

/**
 * @brief Read a single byte from register address
 *
 * @param addr Address of the register
 * @param data Pointer to the data buffer where register data is copied to
 */
void BME280::readByte(byte addr, byte *data)
{
    // This initiates communication with the I2C device whose address is defined by SENSOR_ADDR. The I2C address is usually a 7-bit address specific to the device.
    Wire.beginTransmission(SENSOR_ADDR);
    // This sends a byte (addr) to the I2C device. Typically, this byte represents a register address or command that the device should respond to.
    Wire.write(addr);
    // This ends the transmission of the write operation but does not release the I2C bus (i.e., it keeps the connection open).
    // The false argument indicates that a repeated start condition will be sent instead of a stop condition.
    // This is crucial in a read operation where you first specify the register to read from without ending the communication entirely.
    Wire.endTransmission(false);
    // This requests 1 byte of data from the I2C device at address SENSOR_ADDR. The function will wait for the device to send the requested data.
    Wire.requestFrom(SENSOR_ADDR, 1);
    // This reads the byte of data that was requested and stores it in the memory location pointed to by data.
    Wire.readBytes(data, 1);
}

uint8_t BME280::readByte(byte addr)
{
    // This initiates communication with the I2C device whose address is defined by SENSOR_ADDR. The I2C address is usually a 7-bit address specific to the device.
    Wire.beginTransmission(SENSOR_ADDR);
    // This sends a byte (addr) to the I2C device. Typically, this byte represents a register address or command that the device should respond to.
    Wire.write(addr);
    // This ends the transmission of the write operation but does not release the I2C bus (i.e., it keeps the connection open).
    // The false argument indicates that a repeated start condition will be sent instead of a stop condition.
    // This is crucial in a read operation where you first specify the register to read from without ending the communication entirely.
    Wire.endTransmission(false);
    // This requests 1 byte of data from the I2C device at address SENSOR_ADDR. The function will wait for the device to send the requested data.
    Wire.requestFrom(SENSOR_ADDR, 1);
    // This reads the byte of data that was requested and stores it in the memory location pointed to by data.
    uint8_t data;
    Wire.readBytes(&data, 1);
    return data;
}

void BME280::writeByte(byte addr, byte *data)
{
    // Start communication with the slave device
    Wire.beginTransmission(SENSOR_ADDR);
    // Send the register address
    Wire.write(addr);
    // Send the data byte to be written to the register
    Wire.write(*data);
    // End transmission and send the data
    Wire.endTransmission();
}
int8_t BME280::combineByte(uint8_t lsb)
{
    return (int8_t)(readByte(lsb));
}

void BME280::printParams()
{
    Serial.begin(115200);
    Serial.print("_cal_dig_T1: ");
    Serial.println(_cal_dig_T1);
    Serial.print("_cal_dig_T2: ");
    Serial.println(_cal_dig_T2);
    Serial.print("_cal_dig_T3: ");
    Serial.println(_cal_dig_T3);
    Serial.print("_cal_dig_P1: ");
    Serial.println(_cal_dig_P1);
    Serial.print("_cal_dig_P2: ");
    Serial.println(_cal_dig_P2);
    Serial.print("_cal_dig_P3: ");
    Serial.println(_cal_dig_P3);
    Serial.print("_cal_dig_P4: ");
    Serial.println(_cal_dig_P4);
    Serial.print("_cal_dig_P5: ");
    Serial.println(_cal_dig_P5);
    Serial.print("_cal_dig_P6: ");
    Serial.println(_cal_dig_P6);
    Serial.print("_cal_dig_P8: ");
    Serial.println(_cal_dig_P7);
    Serial.print("_cal_dig_P9: ");
    Serial.println(_cal_dig_P9);
    Serial.print("_cal_dig_H1: ");
    Serial.println(_cal_dig_H1);
    Serial.print("_cal_dig_H2: ");
    Serial.println(_cal_dig_H2);
    Serial.print("_cal_dig_H3: ");
    Serial.println(_cal_dig_H3);
    Serial.print("_cal_dig_H4: ");
    Serial.println(_cal_dig_H4);
    Serial.print("_cal_dig_H5: ");
    Serial.println(_cal_dig_H5);
    Serial.print("_cal_dig_H6: ");
    Serial.println(_cal_dig_H6);
}

void BME280::readCalibrationRegisters()
{
    getData(BME280_T1_REG, _cal_dig_T1);
    getData(BME280_T2_REG, _cal_dig_T2);
    getData(BME280_T3_REG, _cal_dig_T3);
    getData(BME280_P1_REG, _cal_dig_P1);
    getData(BME280_P2_REG, _cal_dig_P2);
    getData(BME280_P3_REG, _cal_dig_P3);
    getData(BME280_P4_REG, _cal_dig_P4);
    getData(BME280_P5_REG, _cal_dig_P5);
    getData(BME280_P6_REG, _cal_dig_P6);
    getData(BME280_P7_REG, _cal_dig_P7);
    getData(BME280_P8_REG, _cal_dig_P8);
    getData(BME280_P9_REG, _cal_dig_P9);
    getData(BME280_H1_REG, _cal_dig_H1);
    getData(BME280_H2_REG, _cal_dig_H2);
    getData(BME280_H3_REG, _cal_dig_P3);
    uint8_t tempVar; // Single-Byte temporary variable
    getData(BME280_H4_REG, tempVar);
    _cal_dig_H4 = tempVar << 4;
    getData(BME280_H4_REG + 1, tempVar);
    _cal_dig_H4 |= tempVar & 0xF;
    getData(BME280_H5_REG + 2, tempVar);
    _cal_dig_H5 = tempVar << 4;
    getData(BME280_H5_REG, tempVar);
    _cal_dig_H5 |= tempVar >> 4;
    getData(BME280_H6_REG, _cal_dig_H6);
    _cal_dig_H5 = (readByte(BME280_H5_REG + 1) << 4) | (readByte(BME280_H5_REG) >> 4);
    //_cal_dig_H6 = readByte(BME280_H6_REG);

    printParams();
}

/**
 * @brief Function used to begin operation of the sensor
 *
 * @param i2cSpeed
 * @return true
 * @return false
 */
bool BME280::begin(int32_t i2cSpeed)
{
    // Start I2C communication in master mode
    Wire.begin();
    Wire.setClock(i2cSpeed);
    // Byte for storing the sensor ID
    byte id = 0x00;
    // Read the sensor ID from the register
    readByte(REG_ID, &id);
    // If sensor is working it should return an ID 0x60
    if (id == DEVICE_ID)
    {
        // Sensor is working propertly and I2C is working
        readCalibrationRegisters();
        // Reset the sensor so it goes back to sleep mode and all changes are reset.
        reset();
        return true;
    }
    // Something is not right with sensor or I2C comunication, returning false
    return false;
}

uint8_t BME280::setMode(operatingMode deviceMode)
{
    uint8_t ctrlMeasRegister;
    readByte(REG_CTRL_MEAS, &ctrlMeasRegister);
    uint8_t mode;
    switch (deviceMode)
    {
    case (SLEEP_MODE):
        mode = 0x00;
        break;
    case (FORCED_MODE):
        mode = 0x01;
        break;
    case (NORMAL_MODE):
        mode = 0x03;
        break;
    default:
        mode = 0x03;
    };
    ctrlMeasRegister = (ctrlMeasRegister & 0xFC) | mode;
    Serial.print("Mode is set.\nCTRL_MEAS_REGISTER: ");
    Serial.println(ctrlMeasRegister, BIN);
    writeByte(REG_CTRL_MEAS, &ctrlMeasRegister);
    return ctrlMeasRegister;
}

bool BME280::setOversampling(sensorType sensor, overSamplingType oversampling)
{
    byte osValues[6] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

    if (sensor == HUM)
    {
        byte ctrlHumRegister;
        readByte(REG_CTRL_HUM, &ctrlHumRegister);
        ctrlHumRegister = (ctrlHumRegister & 0xF8) | (osValues[oversampling] & 0x07);
        Serial.print("Humidty oversampling set. REG_CTRL_HUM: ");
        Serial.println(ctrlHumRegister, BIN);
        writeByte(REG_CTRL_HUM, &ctrlHumRegister);
        return true;
    }
    else if (sensor == PRESS)
    {
        byte ctrlMeasRegister;
        readByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        ctrlMeasRegister = (ctrlMeasRegister & 0xE3) | ((osValues[oversampling] & 0x07) << 2);
        Serial.print("Measurement oversampling set. REG_CTRL_MEAS: ");
        Serial.println(ctrlMeasRegister, BIN);
        writeByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        return true;
    }
    else if (sensor == TEMP)
    {
        byte ctrlMeasRegister;
        readByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        ctrlMeasRegister = (ctrlMeasRegister & 0x1F) | ((osValues[oversampling] & 0x07) << 5);
        writeByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        return true;
    }
    return false;
}

uint8_t BME280::iirFilter(filterSettings filterSetting)
{
    byte filterSettingValues[6] = {0x00, 0x01, 0x02, 0x03, 0x04};
    uint8_t configRegister;
    readByte(REG_CONFIG, &configRegister);
    configRegister = (configRegister & 0xE3) | ((filterSettingValues[filterSetting] & 0x7) << 2);
    Serial.print("IIR filter set.\nREG_CONFIG: ");
    Serial.println(configRegister, BIN);
    writeByte(REG_CONFIG, &configRegister);
    return filterSettingValues[filterSetting];
}

uint8_t BME280::inactiveTime(standBySettings standByTime)
{
    byte sbTimeValues[7] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint8_t configRegister;
    readByte(REG_CONFIG, &configRegister);
    configRegister = (configRegister && 0x1F) | ((sbTimeValues[standByTime] & 0x07) << 5);
    Serial.print("Inactive time set.\nREG_CONFIG: ");
    Serial.println(configRegister, BIN);
    writeByte(REG_CONFIG, &configRegister);
    return sbTimeValues[standByTime];
}

void BME280::getSensorData()
{
    while (readByte(REG_STATUS) & 0x09 != 0)
        ;
    byte sensorData[8];
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(REG_PRESS_MSB);
    Wire.endTransmission();
    Wire.requestFrom(SENSOR_ADDR, 8);
    uint8_t structSize = Wire.available();
    for (uint8_t i = 0; i < structSize; ++i)
    {
        sensorData[i] = Wire.read();
        // Serial.print(sensorData[i], BIN);
    }
    // Serial.println();
    int64_t i, j, p;
    _Temperature = (int32_t)sensorData[3] << 12 | (int32_t)sensorData[4] << 4 |
                   (int32_t)sensorData[5] >> 4;
    i = ((((_Temperature >> 3) - ((int32_t)_cal_dig_T1 << 1))) * ((int32_t)_cal_dig_T2)) >> 11;
    j = (((((_Temperature >> 4) - ((int32_t)_cal_dig_T1)) *
           ((_Temperature >> 4) - ((int32_t)_cal_dig_T1))) >>
          12) *
         ((int32_t)_cal_dig_T3)) >>
        14;
    _tfine = i + j;
    _Temperature = (_tfine * 5 + 128) >> 8; // In centi-degrees Celsius
    //*******************************//
    // Now compute the pressure      //
    //*******************************//
    _Pressure = (int32_t)sensorData[0] << 12 | (int32_t)sensorData[1] << 4 |
                (int32_t)sensorData[2] >> 4;
    i = ((int64_t)_tfine) - 128000;
    j = i * i * (int64_t)_cal_dig_P6;
    j = j + ((i * (int64_t)_cal_dig_P5) << 17);
    j = j + (((int64_t)_cal_dig_P4) << 35);
    i = ((i * i * (int64_t)_cal_dig_P3) >> 8) + ((i * (int64_t)_cal_dig_P2) << 12);
    i = (((((int64_t)1) << 47) + i)) * ((int64_t)_cal_dig_P1) >> 33;
    if (i == 0)
        _Pressure = 0; // avoid division by 0 exception
    else
    {
        p = 1048576 - _Pressure;
        p = (((p << 31) - j) * 3125) / i;
        i = (((int64_t)_cal_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        j = (((int64_t)_cal_dig_P8) * p) >> 19;
        p = ((p + i + j) >> 8) + (((int64_t)_cal_dig_P7) << 4);
        _Pressure = p >> 8; // in pascals
    } // of if pressure would cause error
      //**********************************//
      // And finally compute the humidity //
      //**********************************//
    _Humidity = (int32_t)sensorData[6] << 8 | (int32_t)sensorData[7];
    i = (_tfine - ((int32_t)76800));
    i = (((((_Humidity << 14) - (((int32_t)_cal_dig_H4) << 20) - (((int32_t)_cal_dig_H5) * i)) +
           ((int32_t)16384)) >>
          15) *
         (((((((i * ((int32_t)_cal_dig_H6)) >> 10) *
              (((i * ((int32_t)_cal_dig_H3)) >> 11) + ((int32_t)32768))) >>
             10) +
            ((int32_t)2097152)) *
               ((int32_t)_cal_dig_H2) +
           8192) >>
          14));
    i = (i - (((((i >> 15) * (i >> 15)) >> 7) * ((int32_t)_cal_dig_H1)) >> 4));
    i = (i < 0) ? 0 : i;
    i = (i > 419430400) ? 419430400 : i;
    _Humidity = (uint32_t)(i >> 12) * 100 / 1024; // in percent * 100
}

float BME280::getTemperature()
{
    getSensorData();
    return _Temperature / 100.0;
}

float BME280::getPressure()
{
    getSensorData();
    return _Pressure / 100.0;
}

float BME280::getHumidity()
{
    getSensorData();
    return _Humidity / 100.0;
}

void BME280::reset()
{
    byte resetRegister = readByte(REG_RESET);
    resetRegister |= 0xB6;
    Serial.print("Reset register set.\nREG_RESET: ");
    Serial.println(resetRegister, BIN);
    writeByte(REG_RESET, &resetRegister);
}