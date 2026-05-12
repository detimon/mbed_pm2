// CargoSweep — Final Version 01
#pragma once

#include "mbed.h"

void cargosweep_final_v01_init(int loops_per_second);
void cargosweep_final_v01_task(DigitalOut& led);
void cargosweep_final_v01_reset(DigitalOut& led);
void cargosweep_final_v01_print();
