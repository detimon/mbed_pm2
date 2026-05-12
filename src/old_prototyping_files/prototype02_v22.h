// CargoSweep — PROTOTYPE_02 Header (v22)
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v22_init(int loops_per_second);
void roboter_v22_task(DigitalOut& led1);
void roboter_v22_reset(DigitalOut& led1);
void roboter_v22_print();
