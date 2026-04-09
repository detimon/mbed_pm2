#pragma once

#include "mbed.h"
#include "PESBoardPinMap.h"

void drehservo_90_init(int loops_per_second);
void drehservo_90_task(DigitalOut& led);
void drehservo_90_reset(DigitalOut& led);
void drehservo_90_print();
