// CargoSweep — PROTOTYPE_03 Header (v26)
// Clean rewrite using ServoFeedback360 for the rotating tray.
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v26_init(int loops_per_second);
void roboter_v26_task(DigitalOut& led);
void roboter_v26_reset(DigitalOut& led);
void roboter_v26_print();
