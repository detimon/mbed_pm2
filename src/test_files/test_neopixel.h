#pragma once

#include "test_config.h"

#ifdef TEST_NEOPIXEL

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "NeoPixel.h"

void neopixel_init(int loops_per_second);
void neopixel_task(DigitalOut& led1);
void neopixel_reset(DigitalOut& led1);
void neopixel_print();

#endif // TEST_NEOPIXEL
