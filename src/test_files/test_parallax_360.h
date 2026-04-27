#pragma once

#include "mbed.h"
#include "PESBoardPinMap.h"

void parallax_360_init(int loops_per_second);
void parallax_360_task(DigitalOut& led);
void parallax_360_reset(DigitalOut& led);
void parallax_360_print();
