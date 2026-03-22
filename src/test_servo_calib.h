#pragma once

#include "test_config.h"

#ifdef TEST_SERVO_CALIB

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void servo_calib_init(int loops_per_second);
void servo_calib_task(DigitalOut& led1);
void servo_calib_reset(DigitalOut& led1);
void servo_calib_print();

#endif // TEST_SERVO_CALIB
