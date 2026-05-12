// CargoSweep — PROTOTYPE_02 Header (v19)
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v19_init(int loops_per_second);
void roboter_v19_task(DigitalOut& led1);
void roboter_v19_reset(DigitalOut& led1);
void roboter_v19_print();
