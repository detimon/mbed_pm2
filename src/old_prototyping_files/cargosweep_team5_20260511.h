// CargoSweep — Team 5 | 2026-05-11
// Refactored from cargosweep_final_version_05.cpp
// States: 20 → 15 (merged alignment + crossing stop states, renamed for clarity)
// Logic and timing: UNCHANGED from Version 05.
#pragma once

#include "mbed.h"

void cargosweep_team5_20260511_init(int loops_per_second);
void cargosweep_team5_20260511_task(DigitalOut& led);
void cargosweep_team5_20260511_reset(DigitalOut& led);
void cargosweep_team5_20260511_print();
