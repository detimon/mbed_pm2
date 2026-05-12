#ifndef TEST_LINE_FOLLOWER_BACKWARD_H_
#define TEST_LINE_FOLLOWER_BACKWARD_H_

#include "mbed.h"

void linefollower_bwd_init(int loops_per_second);
void linefollower_bwd_task(DigitalOut& led);
void linefollower_bwd_reset(DigitalOut& led);
void linefollower_bwd_print();

#endif /* TEST_LINE_FOLLOWER_BACKWARD_H_ */
