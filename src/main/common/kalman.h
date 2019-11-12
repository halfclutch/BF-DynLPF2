#pragma once

#define MAX_WINDOW_SIZE         300
#define DEF_WINDOW_SIZE         10       //Default value in Butterfilght 3.5.2 is 10@32Khz. So should be 2,5@8Khz but low limit is 6

#define DEFAULT_ROLL_Q         (1500)    //Default value is 1500 in Butterfilght 3.5.2
#define DEFAULT_PITCH_Q        (1500)     
#define DEFAULT_YAW_Q          (1500)     

extern void kalman_init(void);
extern void kalman_update(float *input, float* output);
