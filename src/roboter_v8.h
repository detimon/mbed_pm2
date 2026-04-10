#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v8_init(int loops_per_second);
void roboter_v8_task(DigitalOut& led1);
void roboter_v8_reset(DigitalOut& led1);
void roboter_v8_print();
