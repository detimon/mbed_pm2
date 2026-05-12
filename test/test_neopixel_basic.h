#pragma once

#include "test_config.h"

#ifdef TEST_NEOPIXEL_BASIC

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "NeoPixel.h"

void neopixel_basic_init(int loops_per_second);
void neopixel_basic_task(DigitalOut& led1);
void neopixel_basic_reset(DigitalOut& led1);
void neopixel_basic_print();

#endif // TEST_NEOPIXEL_BASIC
