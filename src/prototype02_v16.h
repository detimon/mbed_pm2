// CargoSweep — PROTOTYPE_02 Header (v16)
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v16_init(int loops_per_second);
void roboter_v16_task(DigitalOut& led1);
void roboter_v16_reset(DigitalOut& led1);
void roboter_v16_print();
