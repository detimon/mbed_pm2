#ifndef TEST_LOGIC_ARM_STANDARD_H_
#define TEST_LOGIC_ARM_STANDARD_H_

#include "mbed.h"

void arm_logic_init(int loops_per_second);
void arm_logic_task(DigitalOut& led);
void arm_logic_reset(DigitalOut& led);
void arm_logic_print();

#endif /* TEST_LOGIC_ARM_STANDARD_H_ */
