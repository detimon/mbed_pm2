// CargoSweep â€” Final Version 04
#pragma once

#include "mbed.h"

void cargosweep_final_v04_init(int loops_per_second);
void cargosweep_final_v04_task(DigitalOut& led);
void cargosweep_final_v04_reset(DigitalOut& led);
void cargosweep_final_v04_print();


