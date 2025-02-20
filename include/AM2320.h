#pragma once

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

namespace AM2320 {

class Sensor {
public:
    Sensor(i2c_inst_t *i2c);
private:
    i2c_inst_t *_i2c;
};

} // namespace AM2320
