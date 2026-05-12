#pragma once

#include "mbed.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v2_init(int loops_per_second);
void roboter_v2_task(DigitalOut& led1);
void roboter_v2_reset(DigitalOut& led1);
void roboter_v2_print();
