#pragma once

#include "test_config.h"

#ifdef TEST_COLOR_NEOPIXEL

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "ColorSensor.h"
#include "NeoPixel.h"

void color_neopixel_init(int loops_per_second);
void color_neopixel_task(DigitalOut& led1);
void color_neopixel_reset(DigitalOut& led1);
void color_neopixel_print();

#endif // TEST_COLOR_NEOPIXEL
