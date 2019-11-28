#pragma once

#define DEFAULT_DYN_LPF_FC_THRESHOLD   25.0f  //Value in °/s
#define DYN_E_HYTEREIS                  2.0f  //Value in °/s


#define DEFAULT_FC_MAX                1000.0f  //Fmax in Hz
#define DEFAULT_FC_MIN                 40.0f  //Fmin in Hz
#define DEFAULT_DYN_LPF_GAIN           10     //Gain

#define DEFAULT_DYN_LPF_THROTTLE_THRESHOLD  40  //Throttle in %
#define DEFAULT_DYN_LPF_THROTTLE_GAIN       5   // 5Hz / % throrrle over 40%

#define DEFAULT_DYN_LPF_FC_FC          15.0f   //Cut of freq on FC value

extern void init_dynLpf2(void);
extern float dynLpf2Apply(int axis, float input);
