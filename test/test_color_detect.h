#pragma once

#include "test_config.h"

#ifdef TEST_COLOR_DETECT

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "ColorSensor.h"

void color_detect_init(int loops_per_second);
void color_detect_task(DigitalOut& led1);
void color_detect_reset(DigitalOut& led1);
void color_detect_print();

#endif // TEST_COLOR_DETECT
