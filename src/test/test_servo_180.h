#pragma once

#include "test_config.h"

#ifdef TEST_SERVO_180

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void servo_180_init(int loops_per_second);
void servo_180_task(DigitalOut& led1);
void servo_180_reset(DigitalOut& led1);
void servo_180_print();

#endif // TEST_SERVO_180
