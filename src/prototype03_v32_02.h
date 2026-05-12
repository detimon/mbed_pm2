// CargoSweep — PROTOTYPE_03 Header (v32_02)
// Kopie von v31_1. Aufnahme-Sequenz (runPickupPhase) durch Ablege-Sequenz
// (runDeliverPhase) ersetzt — an breiten Balken und ROT_GELB_PAUSE.
#pragma once

#include "mbed.h"

void roboter_v32_02_init(int loops_per_second);
void roboter_v32_02_task(DigitalOut& led);
void roboter_v32_02_reset(DigitalOut& led);
void roboter_v32_02_print();
