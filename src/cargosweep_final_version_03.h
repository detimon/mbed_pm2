// CargoSweep â€” Final Version 03
#pragma once

#include "mbed.h"

void cargosweep_final_v03_init(int loops_per_second);
void cargosweep_final_v03_task(DigitalOut& led);
void cargosweep_final_v03_reset(DigitalOut& led);
void cargosweep_final_v03_print();

