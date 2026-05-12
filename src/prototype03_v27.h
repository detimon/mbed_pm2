// CargoSweep — PROTOTYPE_03 Header (v27)
// Clean rewrite using ServoFeedback360 for the rotating tray.
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v27_init(int loops_per_second);
void roboter_v27_task(DigitalOut& led);
void roboter_v27_reset(DigitalOut& led);
void roboter_v27_print();
