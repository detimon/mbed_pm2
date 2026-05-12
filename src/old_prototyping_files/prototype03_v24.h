// CargoSweep — PROTOTYPE_03 Header (v24)
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v24_init(int loops_per_second);
void roboter_v24_task(DigitalOut& led1);
void roboter_v24_reset(DigitalOut& led1);
void roboter_v24_print();
