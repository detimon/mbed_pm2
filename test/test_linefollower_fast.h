#ifndef TEST_LINE_FOLLOWER_FAST_H_
#define TEST_LINE_FOLLOWER_FAST_H_

#include "mbed.h"

void linefollower_fast_init(int loops_per_second);
void linefollower_fast_task(DigitalOut& led);
void linefollower_fast_reset(DigitalOut& led);
void linefollower_fast_print();

#endif /* TEST_LINE_FOLLOWER_FAST_H_ */
