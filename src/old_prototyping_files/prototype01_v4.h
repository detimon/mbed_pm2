#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v4_init(int loops_per_second);
void roboter_v4_task(DigitalOut& led1);
void roboter_v4_reset(DigitalOut& led1);
void roboter_v4_print();
