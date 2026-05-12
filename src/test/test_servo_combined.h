#pragma once

#include "test_config.h"

#ifdef TEST_SERVO_COMBINED

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void servo_combined_init(int loops_per_second);
void servo_combined_task(DigitalOut& led1);
void servo_combined_reset(DigitalOut& led1);
void servo_combined_print();

#endif // TEST_SERVO_COMBINED
