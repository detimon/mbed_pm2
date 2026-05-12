#pragma once

#include "test_config.h"

#ifdef CALIB_COLOR_HUE

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "ColorSensor.h"

void calib_color_hue_init(int loops_per_second);
void calib_color_hue_task(DigitalOut& led1);
void calib_color_hue_reset(DigitalOut& led1);
void calib_color_hue_print();

#endif // CALIB_COLOR_HUE
