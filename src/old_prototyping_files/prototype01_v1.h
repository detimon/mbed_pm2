#pragma once

#include "mbed.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v1_init(int loops_per_second);
void roboter_v1_task(DigitalOut& led1);
void roboter_v1_reset(DigitalOut& led1);
void roboter_v1_print();
