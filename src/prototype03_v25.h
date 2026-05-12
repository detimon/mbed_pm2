#pragma once

#include "mbed.h"

void roboter_v25_init(int loops_per_second);
void roboter_v25_task(DigitalOut& led);
void roboter_v25_reset(DigitalOut& led);
void roboter_v25_print();
