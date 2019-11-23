#pragma once

#define DEFAULT_DYN_LPF_FC_THRESHOLD   10.0f  //Value in °/s
#define DYN_E_HYTEREIS                  1.0f  //Value in °/s


#define DEFAULT_FC_MAX                700.0f  //Fmax in Hz
#define DEFAULT_FC_MIN                 60.0f  //Fmin in Hz
#define DEFAULT_DYN_LPF_GAIN          1000    //Gain in Hz.

#define DEFAULT_DYN_LPF_THROTTLE_THRESHOLD  40  //Throttle in %
#define DEFAULT_DYN_LPF_THROTTLE_GAIN       3   // 3Hz / % throrrle over 40%

#define DEFAULT_DYN_LPF_FC_FC          3.0f   //Cut of freq on FC value

extern void init_dynLpf2(void);
extern float dynLpf2Apply(int axis, float input);
