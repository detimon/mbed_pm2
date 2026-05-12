// CargoSweep — Final Version 05
#pragma once

#include "mbed.h"

void cargosweep_final_v05_init(int loops_per_second);
void cargosweep_final_v05_task(DigitalOut& led);
void cargosweep_final_v05_reset(DigitalOut& led);
void cargosweep_final_v05_print();
