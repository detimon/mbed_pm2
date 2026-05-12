// CargoSweep — Final Version 02
#pragma once

#include "mbed.h"

void cargosweep_final_v02_init(int loops_per_second);
void cargosweep_final_v02_task(DigitalOut& led);
void cargosweep_final_v02_reset(DigitalOut& led);
void cargosweep_final_v02_print();
