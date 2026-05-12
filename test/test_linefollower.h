#ifndef TEST_LINE_FOLLOWER_H_
#define TEST_LINE_FOLLOWER_H_

#include "mbed.h"

void linefollower_init(int loops_per_second);
void linefollower_task(DigitalOut& led);
void linefollower_reset(DigitalOut& led);
void linefollower_print();

#endif /* TEST_LINE_FOLLOWER_H_ */
