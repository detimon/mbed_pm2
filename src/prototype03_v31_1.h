// CargoSweep — PROTOTYPE_03 Header (v31_1)
// Basiert auf v30_1 (tray aktiv während ROT_GELB_DRIVE, Phase-0-Stale-Error-Fix).
// ServoFeedback360 für den Drehteller (PB_12 / PC_2).
// Pin mapping per Drahtzugliste V10.
#pragma once

#include "mbed.h"

void roboter_v31_1_init(int loops_per_second);
void roboter_v31_1_task(DigitalOut& led);
void roboter_v31_1_reset(DigitalOut& led);
void roboter_v31_1_print();
