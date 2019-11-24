
#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynLpf2.h"

#include "fc/rc.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#include "fc/rc_controls.h"

//TYPES
//-----
  typedef struct {
      pt1Filter_t pt1;      //PT1 filter
      float Fc;             //Cutoff freq
      
      pt1Filter_t pt1Fc;    //PT1 on Fc

      bool Dyn_Fc;         //Dynamic E or Fixed E
  } dynLpf_t;


//VARIABLES
//---------
    dynLpf_t dynLpf[3];

    float Fmin_init,Fmax;
    float throttleThreshold;
    float throttleGain;
    float dynFcThreshold;
    float dynGainOnError;     


//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////
void init_dynLpf2(void)
{
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    float gain = pt1FilterGain(gyroConfigMutable()->dynlpf_fmin, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1, gain);
    pt1FilterInit(&dynLpf[1].pt1, gain);
    pt1FilterInit(&dynLpf[2].pt1, gain);

    gain = pt1FilterGain(gyroConfigMutable()->dynlpf_fc_fc, gyroDt);
    pt1FilterInit(&dynLpf[0].pt1Fc, gain);
    pt1FilterInit(&dynLpf[1].pt1Fc, gain);
    pt1FilterInit(&dynLpf[2].pt1Fc, gain);

    dynLpf[0].Fc = gyroConfigMutable()->dynlpf_fmin;
    dynLpf[1].Fc = gyroConfigMutable()->dynlpf_fmin;
    dynLpf[2].Fc = gyroConfigMutable()->dynlpf_fmin;

    dynLpf[0].Dyn_Fc = false;
    dynLpf[1].Dyn_Fc = false;
    dynLpf[2].Dyn_Fc = false;

    Fmax                 = (float)gyroConfigMutable()->dynlpf_fmax;         //PT1 maxFc in Hz
    Fmin_init            = (float)gyroConfigMutable()->dynlpf_fmin;         //PT1 min Fc in Hz
   
    throttleThreshold    = (float)gyroConfigMutable()->dynlpf_throttle_threshold;
    throttleGain         = (float)gyroConfigMutable()->dynlpf_throttle_gain;

    dynFcThreshold       = (float)(gyroConfigMutable()->dynlpf_threshold);   //Min Setpoint & Gyro value to rise PT1 Fc
    dynGainOnError       = (float)(gyroConfigMutable()->dynlpf_gain);        
}

//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//                          //
//////////////////////////////
#pragma GCC push_options
#pragma GCC optimize("O3")

FAST_CODE float dynLpf_process(dynLpf_t* filter, float input, float target) {

float newFc, Fmin;
float throttle;

Fmin = Fmin_init;
throttle  = (rcCommand[THROTTLE] - 1000.0f) * 0.1f;
const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Check if we are in dynamic or fixed "e"
    //---------------------------------------
        if(filter->Dyn_Fc) {
            if (((float)(fabs(target)) <= (dynFcThreshold - DYN_E_HYTEREIS)) && ((float)(fabs(filter->pt1.state)) <= (dynFcThreshold - DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = false;
            }
        }else{
            //Enable Dyn_Fc when stick or Quad move
            if (((float)(fabs(target)) >= (dynFcThreshold + DYN_E_HYTEREIS)) || ((float)(fabs(filter->pt1.state)) >= (dynFcThreshold + DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = true;
            }
        }

    //Rise Fmin according to Throttle;
    //--------------------------------
        if(throttle > throttleThreshold){
            Fmin += (throttle - throttleThreshold) * throttleGain;
        }


    //Compute e & Fc
    //--------------
        if(filter->Dyn_Fc) {
            //Avoid division by 0.0f
                if(target == 0.0f)              { target = 0.00001f; }
                if(filter->pt1.state == 0.0f)   { filter->pt1.state = 0.0001f; }

            //Compute e factor
                float Average, Error;
                Average  = (target + input) * 0.5f;
                Error = ((float)fabs( target - input )) / Average;
                Error *= 10.0f;

            //New freq  
                newFc = Fmin + dynGainOnError * (Error * Error);

        } else {
                newFc  = Fmin;
        }

    //Limit & Filter newFc
    //---------------------
        //Low Limit
        if(newFc < Fmin)  { newFc  = Fmin; }

        //High Limit
        if(newFc > Fmax)  { newFc  = Fmax; }

        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);

    //Update PT1 filter
    //------------------
        pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
        filter->Fc = newFc;

    //Apply filter
    //------------
        float output = pt1FilterApply(&filter->pt1, input);

 return output;
}


//////////////////////////////
//                          //
//      DYN LPF2 APPLY      //
//                          //
//////////////////////////////

FAST_CODE float dynLpf2Apply(int axis, float input) {

float output;
float target = getSetpointRate(axis);


  //Apply filter if filter is enable.
    if(gyroConfigMutable()->dynlpf_gain > 0) {
      output = dynLpf_process(&dynLpf[axis], input, target);
    }else{
      output = input;
    }


  //Blackbox
    if(axis == ROLL) {
        DEBUG_SET(DEBUG_DYN_LPF2, 0, (int16_t)(lrintf(input)));
        DEBUG_SET(DEBUG_DYN_LPF2, 1, (int16_t)(lrintf(output)));
        DEBUG_SET(DEBUG_DYN_LPF2, 2, (int16_t)(lrintf(dynLpf[axis].Fc)));
        DEBUG_SET(DEBUG_DYN_LPF2, 3, (int16_t)(lrintf(target)));
    }

return output;
}

#pragma GCC pop_options
