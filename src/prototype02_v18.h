// CargoSweep — PROTOTYPE_02 Header (v18)
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v18_init(int loops_per_second);
void roboter_v18_task(DigitalOut& led1);
void roboter_v18_reset(DigitalOut& led1);
void roboter_v18_print();
