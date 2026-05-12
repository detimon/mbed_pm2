#pragma once

#include "test_config.h"

#ifdef CALIB_SERVO_180

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void calib_servo_180_init(int loops_per_second);
void calib_servo_180_task(DigitalOut& led1);
void calib_servo_180_reset(DigitalOut& led1);
void calib_servo_180_print();

#endif // CALIB_SERVO_180
