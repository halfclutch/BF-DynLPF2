
#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "dynLpf2.h"

#include "fc/rc.h"

#include "build/debug.h"

#include "common/filter.h"

#include "sensors/gyro.h"

#include "fc/rc.h"

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
}

//////////////////////////////
//                          //
//      DYN LPF PROCESS     //
//                          //
//////////////////////////////
#pragma GCC push_options
#pragma GCC optimize("O3")

FAST_CODE float dynLpf_process(dynLpf_t* filter, float input, float target) {

float newFc;
float Fmin              = (float)gyroConfigMutable()->dynlpf_fmin;
float Fmax              = (float)gyroConfigMutable()->dynlpf_fmax;
float DynGain           = (float)(gyroConfigMutable()->dynlpf_gain) * 0.1f;
float DynFcThreshold    = (float)(gyroConfigMutable()->dynlpf_threshold);

const float gyroDt = gyro.targetLooptime * 1e-6f;

    //Check if we are in dynamic or fixed "e"
    //---------------------------------------
        if(filter->Dyn_Fc) {
            if (((float)(fabs(target)) <= (DynFcThreshold - DYN_E_HYTEREIS)) && ((float)(fabs(filter->pt1.state)) <= (DynFcThreshold - DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = false;
            }
        }else{
            //Enable Dyn_Fc when stick or Quad move
            if (((float)(fabs(target)) >= (DynFcThreshold + DYN_E_HYTEREIS)) || ((float)(fabs(filter->pt1.state)) >= (DynFcThreshold + DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = true;
            }
        }

    //Compute e & Fc
    //--------------
        if(getRcDeflection(THROTTLE) > 0.0f){
            //If throttle > 50%, Fc=Fmax
                newFc = Fmax;

        }else if(filter->Dyn_Fc) {
            //Avoid division by 0.0f
                if(target == 0.0f)              { target = 0.00001f; }
                if(filter->pt1.state == 0.0f)   { filter->pt1.state = 0.00001f; }

            //Compute e factor : e = Gain * Error / Avg(target;filtered)
                float e, Average, Error, signalInput;

                //signalInput = filter->pt1.state;
                signalInput = input;

                Average  = (target + signalInput) * 0.5f;
                Error = (float)(fabs( target - signalInput ));
            
            
                e = (DynGain * Error / Average );

            //New freq
                newFc = Fmax * e;

        } else {
                newFc  = Fmin;
        }

    //Limit & Filter newFc
    //---------------------
        //Low Limit (Limit dysmetry is volonter. Higher rise time, lower fall time)
        if(newFc < Fmin)  { newFc  = Fmin; }

        //Filter the cut-off freq ;)
        newFc = pt1FilterApply(&filter->pt1Fc, newFc);

        //High Limit
        if(newFc > Fmax)  { newFc  = Fmax; }



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
  //Apply filter
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
