#pragma once

#include "mbed.h"
#include "Servo.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "ColorSensor.h"
#include "PESBoardPinMap.h"

void all_init(int loops_per_second);
void all_task(DigitalOut& led1);
void all_reset(DigitalOut& led1);
void all_print();
