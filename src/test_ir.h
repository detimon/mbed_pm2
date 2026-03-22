#pragma once

#include "test_config.h"

#ifdef TEST_IR

#include "mbed.h"
#include "IRSensor.h"

void ir_init(int loops_per_second);
void ir_task(DigitalOut& led1);
void ir_reset(DigitalOut& led1);
void ir_print();

#endif // TEST_IR
