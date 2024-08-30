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
        ctrlHumRegister = (ctrlHumRegister & 0xF8) | osValues[oversampling];
        writeByte(REG_CTRL_HUM, &ctrlHumRegister);
        return true;
    }
    else if (sensor == PRESS)
    {
        byte ctrlMeasRegister;
        readByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        ctrlMeasRegister = (ctrlMeasRegister & 0xE3) | (osValues[oversampling] << 2);
        writeByte(REG_CTRL_MEAS, &ctrlMeasRegister);
        return true;
    }
    return false;
}

uint8_t BME280::iirFilter()
{
}