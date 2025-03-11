#pragma once

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "hardware/i2c.h"

namespace AM2320 {

class Sensor
{
public:
    // i2c needs to be already initialized
    explicit Sensor(i2c_inst_t *i2c);

    // TODO(GD): add constructor that initializes i2c

    float readTemperature();
    float readHumidity();
    float temp;
    float humidity;

private:
    uint32_t readRegister32(uint8_t reg);
    uint16_t crc16(uint8_t *buffer, uint8_t nbytes);

    i2c_inst_t *_i2c;
};

}  // namespace AM2320
