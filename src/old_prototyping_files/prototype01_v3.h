#pragma once

#include "mbed.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v3_init(int loops_per_second);
void roboter_v3_task(DigitalOut& led1);
void roboter_v3_reset(DigitalOut& led1);
void roboter_v3_print();
