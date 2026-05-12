#ifndef TEST_DC_MOTOR_H_
#define TEST_DC_MOTOR_H_

#include "mbed.h"

void motor_drive_init(int loops_per_second);
void motor_drive_task(DigitalOut& led);
void motor_drive_reset(DigitalOut& led);
void motor_drive_print();

#endif /* TEST_DC_MOTOR_H_ */
