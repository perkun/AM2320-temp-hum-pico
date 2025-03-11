#include "AM2320.h"
#include <cmath> // for NAN

// #define AM2320_ADDR 0xdd
#define AM2320_ADDR 0x5c
#define AM2320_CMD_READREG 0x03 ///< read register command
#define AM2320_REG_TEMP_H 0x02  ///< temp register address
#define AM2320_REG_HUM_H 0x00   ///< humidity register address

namespace AM2320 {
Sensor::Sensor(i2c_inst_t *i2c) : _i2c(i2c) {}

/**************************************************************************/
/*!
    @brief  perfor a CRC check to verify data
    @param buffer the pointer to the data to check
    @param nbytes the number of bytes to calculate the CRC over
    @return the calculated CRC
*/
/**************************************************************************/
uint16_t Sensor::crc16(uint8_t *buffer, uint8_t nbytes)
{
    uint16_t crc = 0xffff;
    for (int i = 0; i < nbytes; i++) {
        uint8_t b = buffer[i];
        crc ^= b;
        for (int x = 0; x < 8; x++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

uint32_t Sensor::readRegister32(uint8_t reg)
{
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    bool written = false;

    // wake up
    for (int i = 0; i < 3; i++) {
        auto res = i2c_write_timeout_us(_i2c, AM2320_ADDR, buffer, 1, false, 2000);
        // printf("write result: %d\n", res);
        written = res != PICO_ERROR_GENERIC;
        // written = i2c_dev->write(buffer, 1);
        if (written)
            break;
        sleep_ms(100);  // 50-70 lead to 2 errors each time
    }
    if (! written)
        return 0xFFFFFFFF;  // no ACK!
    sleep_ms(10);           // wait 10 ms

    // send a command to read register
    buffer[0] = AM2320_CMD_READREG;
    buffer[1] = reg;
    buffer[2] = 4;  // 4 bytes
    for (int i = 0; i < 3; i++) {
        written =
            i2c_write_timeout_us(_i2c, AM2320_ADDR, buffer, 3, false, 2000) != PICO_ERROR_GENERIC;
        // written = i2c_dev->write(buffer, 3);
        if (written)
            break;
        sleep_ms(5);
    }
    if (! written)
        return 0xFFFFFFFF;  // read not acknowledged!
    sleep_ms(2);            // wait 2 ms

    // 2 bytes preamble, 4 bytes data, 2 bytes CRC
    i2c_read_timeout_us(_i2c, AM2320_ADDR, buffer, 8, false, 2000);
    // i2c_dev->read(buffer, 8);

    if (buffer[0] != 0x03)
        return 0xFFFFFFFF;  // must be 0x03 modbus reply
    if (buffer[1] != 4)
        return 0xFFFFFFFF;  // must be 4 bytes reply

    uint16_t the_crc = buffer[7];
    the_crc <<= 8;
    the_crc |= buffer[6];
    uint16_t calc_crc = crc16(buffer, 6);  // preamble + data
    // Serial.print("CRC: 0x"); Serial.println(calc_crc, HEX);
    if (the_crc != calc_crc)
        return 0xFFFFFFFF;

    // All good!
    uint32_t ret = uint32_t(buffer[2]) << 24 | uint32_t(buffer[3]) << 16 |
                   uint32_t(buffer[4]) << 8 | uint32_t(buffer[5]);

    return ret;
}

float Sensor::readTemperature()
{
    uint32_t data = readRegister32(AM2320_REG_HUM_H);
    if (data == 0xFFFFFFFF)
        return NAN;
    float ft;
    // check sign bit - the temperature MSB is signed , bit 0-15 are magnitude
    if (data & 0x8000) {
        ft = -(int16_t)(data & 0x7FFF);
    }
    else {
        ft = (int16_t)(data & 0xFFFF);
    }

    temp = ft / 10.0;
    return temp;
}

float Sensor::readHumidity()
{
    uint32_t data = readRegister32(AM2320_REG_HUM_H);
    if (data == 0xFFFFFFFF)
        return NAN;

    humidity = (data >> 16) / 10.0;
    return humidity;
}

}  // namespace AM2320
