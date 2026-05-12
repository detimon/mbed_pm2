// CargoSweep — PROTOTYPE_03 Header (v28)
// Clean rewrite using ServoFeedback360 for the rotating tray.
// Servo smooth movement via setMaxAcceleration(0.3f).
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v28_init(int loops_per_second);
void roboter_v28_task(DigitalOut& led);
void roboter_v28_reset(DigitalOut& led);
void roboter_v28_print();
