#ifndef TEST_LOGIC_ARM_STANDARD_H_
#define TEST_LOGIC_ARM_STANDARD_H_

#include "mbed.h"

void logic_arm_standard_init(int loops_per_second);
void logic_arm_standard_task(DigitalOut& led);
void logic_arm_standard_reset(DigitalOut& led);
void logic_arm_standard_print();

#endif /* TEST_LOGIC_ARM_STANDARD_H_ */
