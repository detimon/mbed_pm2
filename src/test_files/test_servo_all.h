#pragma once

#include "test_config.h"

#ifdef TEST_SERVO_ALL

#include "mbed.h"
#include "PESBoardPinMap.h"
#include "Servo.h"

void servo_all_init(int loops_per_second);
void servo_all_task(DigitalOut& led1);
void servo_all_reset(DigitalOut& led1);
void servo_all_print();

#endif // TEST_SERVO_ALL
