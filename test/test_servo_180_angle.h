#pragma once

#include "mbed.h"
#include "PESBoardPinMap.h"

void servo_180_angle_init(int loops_per_second);
void servo_180_angle_task(DigitalOut& led);
void servo_180_angle_reset(DigitalOut& led);
void servo_180_angle_print();
