#pragma once

#define MAX_WINDOW_SIZE         300

//Default values
#define DEF_WINDOW_SIZE         300      // Set < 200 for Kalman (typic 10), else dyn_pt1 is applied

#define DEFAULT_ROLL_Q         (1500)    //Default value is 1500 in Butterfilght 3.5.2
#define DEFAULT_PITCH_Q        (1500)     
#define DEFAULT_YAW_Q          (1500)  
#define E_MAX                   1.0f
#define E_MIN                   0.001f

#define DYN_E_LIMIT             10.0f  //Value in °/s
#define DYN_E_HYTEREIS          1.0f  //Value in °/s


#define DEFAULT_FC_MAX          500.0f
#define DEFAULT_FC_MIN           30.0f

#define DEFAULT_DYN_LPF_GAIN     10    //Gain x10 (20 = 2.0f)

extern void kalman_init(void);
extern void kalman_update(float *input, float* output);

extern void init_dynLpf(void);

