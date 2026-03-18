#pragma once

#include "test_config.h"

#ifdef TEST_SERVO

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void servo_init(int loops_per_second);
void servo_task(DigitalOut& led1);
void servo_reset(DigitalOut& led1);
void servo_print();

#endif // TEST_SERVO