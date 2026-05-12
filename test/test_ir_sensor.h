#pragma once

#include "test_config.h"

#ifdef TEST_IR_SENSOR

#include "mbed.h"
#include "IRSensor.h"

void ir_sensor_init(int loops_per_second);
void ir_sensor_task(DigitalOut& led1);
void ir_sensor_reset(DigitalOut& led1);
void ir_sensor_print();

#endif // TEST_IR_SENSOR
