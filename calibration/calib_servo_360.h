#pragma once

#include "mbed.h"
#include "PESBoardPinMap.h"

void calib_servo_360_init(int loops_per_second);
void calib_servo_360_task(DigitalOut& led);
void calib_servo_360_reset(DigitalOut& led);
void calib_servo_360_print();
