#pragma once

#include "mbed.h"
#include "Servo.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "ColorSensor.h"
#include "PESBoardPinMap.h"

void system_all_init(int loops_per_second);
void system_all_task(DigitalOut& led1);
void system_all_reset(DigitalOut& led1);
void system_all_print();
