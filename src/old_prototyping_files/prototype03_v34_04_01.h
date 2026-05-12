// CargoSweep â€” PROTOTYPE_03 Header (v34_04_01)
// Kopie von v31_1. Aufnahme-Sequenz (runPickupPhase) durch Ablege-Sequenz
// (runDeliverPhase) ersetzt â€” an breiten Balken und ROT_GELB_PAUSE.
#pragma once

#include "mbed.h"

void roboter_v34_04_01_init(int loops_per_second);
void roboter_v34_04_01_task(DigitalOut& led);
void roboter_v34_04_01_reset(DigitalOut& led);
void roboter_v34_04_01_print();

