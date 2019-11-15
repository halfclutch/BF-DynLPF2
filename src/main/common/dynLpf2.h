#pragma once

#define DYN_E_LIMIT             20.0f  //Value in °/s
#define DYN_E_HYTEREIS          1.0f  //Value in °/s


#define DEFAULT_FC_MAX          500.0f
#define DEFAULT_FC_MIN           30.0f

#define DEFAULT_DYN_LPF_GAIN     10    //Gain x10 (20 = 2.0f)

extern void init_dynLpf2(void);
extern float dynLpf2Apply(int axis, float input);
