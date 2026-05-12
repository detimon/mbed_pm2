#pragma once

#include "test_config.h"

#ifdef TEST_NEOPIXEL_COLOR

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "ColorSensor.h"
#include "NeoPixel.h"

void neopixel_color_init(int loops_per_second);
void neopixel_color_task(DigitalOut& led1);
void neopixel_color_reset(DigitalOut& led1);
void neopixel_color_print();

#endif // TEST_NEOPIXEL_COLOR
