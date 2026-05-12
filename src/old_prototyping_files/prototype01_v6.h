#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v6_init(int loops_per_second);
void roboter_v6_task(DigitalOut& led1);
void roboter_v6_reset(DigitalOut& led1);
void roboter_v6_print();
