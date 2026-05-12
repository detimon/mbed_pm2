// CargoSweep — PROTOTYPE_03_V23_2 Header
// Kopie v23 + ServoFeedback360 statt Endschalter-Servo.
#pragma once

#include "mbed.h"
#include "ColorSensor.h"
#include "DCMotor.h"
#include "LineFollower.h"
#include "PESBoardPinMap.h"

void roboter_v23_2_init(int loops_per_second);
void roboter_v23_2_task(DigitalOut& led1);
void roboter_v23_2_reset(DigitalOut& led1);
void roboter_v23_2_print();
