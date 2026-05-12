// CargoSweep — PROTOTYPE_03 Header (v30_1)
// Basiert auf v23_2 (Countdown-Arm-Sequenz, advanceTray +90°).
// ServoFeedback360 für den Drehteller (PB_12 / PC_2).
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v30_1_init(int loops_per_second);
void roboter_v30_1_task(DigitalOut& led);
void roboter_v30_1_reset(DigitalOut& led);
void roboter_v30_1_print();
