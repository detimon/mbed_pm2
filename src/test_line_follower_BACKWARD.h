#ifndef TEST_LINE_FOLLOWER_BACKWARD_H_
#define TEST_LINE_FOLLOWER_BACKWARD_H_

#include "mbed.h"

void line_follower_backward_init(int loops_per_second);
void line_follower_backward_task(DigitalOut& led);
void line_follower_backward_reset(DigitalOut& led);
void line_follower_backward_print();

#endif /* TEST_LINE_FOLLOWER_BACKWARD_H_ */
