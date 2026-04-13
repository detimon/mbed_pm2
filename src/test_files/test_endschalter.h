#pragma once

#include "test_config.h"

#ifdef TEST_ENDSCHALTER

#include "mbed.h"

// Endschalter: PC_5 (PB_A2 = A2)
// Beschaltung: Schalter gegen GND, interner PullUp aktiv -> offen=1, gedrueckt=0

void endschalter_init(int loops_per_second);
void endschalter_task(DigitalOut& led);
void endschalter_reset(DigitalOut& led);
void endschalter_print();

#endif // TEST_ENDSCHALTER
