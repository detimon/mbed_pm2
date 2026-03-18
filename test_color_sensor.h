#pragma once

#include "test_config.h"

#ifdef TEST_COLOR_SENSOR

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "ColorSensor.h"

void color_sensor_init(int loops_per_second);
void color_sensor_task(DigitalOut& led1);
void color_sensor_reset(DigitalOut& led1);
void color_sensor_print();

#endif // TEST_COLOR_SENSOR