// CargoSweep — Team 5 | 2026-05-12
// Cleanup: toter Code entfernt (applyJiggleTick, advanceTray, tote Variablen/Konstanten),
//          s_jiggle_ctr static-Reset-Bug behoben.
#pragma once

#include "mbed.h"

void cargosweep_team5_20260512_init(int loops_per_second);
void cargosweep_team5_20260512_task(DigitalOut& led);
void cargosweep_team5_20260512_reset(DigitalOut& led);
void cargosweep_team5_20260512_print();
