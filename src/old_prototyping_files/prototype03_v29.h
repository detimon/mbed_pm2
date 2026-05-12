// CargoSweep — PROTOTYPE_03 Header (v29)
// Clean rewrite using ServoFeedback360 for the rotating tray.
// Servo smooth movement via setMaxAcceleration(0.3f).
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v29_init(int loops_per_second);
void roboter_v29_task(DigitalOut& led);
void roboter_v29_reset(DigitalOut& led);
void roboter_v29_print();
