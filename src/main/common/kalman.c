#include <stdbool.h>

#include "platform.h"
#include "math.h"

#include "kalman.h"

#include "fc/rc.h"

#include "build/debug.h"

#include "sensors/gyro.h"

// #define VARIANCE_SCALE 0.001
#define VARIANCE_SCALE 0.3333333f

typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float lastX; //previous state
    float e;
    bool Dyn_e;
} kalman_t;

typedef struct variance
{
    float xVar;
    float yVar;
    float zVar;
    float xyCoVar;
    float xzCoVar;
    float yzCoVar;

    uint32_t windex;
    float xWindow[MAX_WINDOW_SIZE];
    float yWindow[MAX_WINDOW_SIZE];
    float zWindow[MAX_WINDOW_SIZE];

    float xSumMean;
    float ySumMean;
    float zSumMean;

    float xMean;
    float yMean;
    float zMean;

    float xSumVar;
    float ySumVar;
    float zSumVar;
    float xySumCoVar;
    float xzSumCoVar;
    float yzSumCoVar;

    float inverseN;
} variance_t;

typedef struct {
    pt1Filter_t pt1;    //PT1 filter
    float Fc;           //Cutoff freq
    bool Dyn_Fc;         //Dynamic E or Fixed E
} dynLpf_t;


variance_t varStruct;
volatile float setPoint[3];
kalman_t kalmanFilterStateRate[3];


kalman_t kalmanFilterStateRate[3];
dynLpf_t dynLpf[3];

//Prototypes
FAST_CODE float dynLpf_process(dynLpf_t* filter, float input, float target);
void init_dynLpf(void);


void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;      //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->e = 1.0f;
    filter->Dyn_e = false;
}


void kalman_init(void)
{
    setPoint[ROLL] = 0.0f;
    setPoint[PITCH] = 0.0f;
    setPoint[YAW] = 0.0f;

    memset(&varStruct, 0, sizeof(varStruct));

    init_kalman(&kalmanFilterStateRate[ROLL],  gyroConfigMutable()->imuf_roll_q);
    init_kalman(&kalmanFilterStateRate[PITCH], gyroConfigMutable()->imuf_pitch_q);
    init_kalman(&kalmanFilterStateRate[YAW],   gyroConfigMutable()->imuf_yaw_q);

    varStruct.inverseN = 1.0f/gyroConfigMutable()->imuf_w;
}

//////////////////////////////
//                          //
//       DYN PT1 INIT       //
//                          //
//////////////////////////////
void init_dynLpf(void)
{
    const float gyroDt = gyro.targetLooptime * 1e-6f;

    const float gain = pt1FilterGain(gyroConfigMutable()->dynlpf_fmin, gyroDt);

    pt1FilterInit(&dynLpf[0].pt1, gain);
    pt1FilterInit(&dynLpf[1].pt1, gain);
    pt1FilterInit(&dynLpf[2].pt1, gain);

    dynLpf[0].Fc = gyroConfigMutable()->dynlpf_fmin;
    dynLpf[1].Fc = gyroConfigMutable()->dynlpf_fmin;
    dynLpf[2].Fc = gyroConfigMutable()->dynlpf_fmin;

    dynLpf[0].Dyn_Fc = false;
    dynLpf[1].Dyn_Fc = false;
    dynLpf[2].Dyn_Fc = false;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void update_kalman_covariance(volatile float *gyroRateData)
{
     varStruct.xWindow[ varStruct.windex] = gyroRateData[ROLL];
     varStruct.yWindow[ varStruct.windex] = gyroRateData[PITCH];
     varStruct.zWindow[ varStruct.windex] = gyroRateData[YAW];

     varStruct.xSumMean +=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean +=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean +=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar + ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.windex++;
    if ( varStruct.windex >= gyroConfigMutable()->imuf_w)
    {
         varStruct.windex = 0;
    }
     varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar - ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumCoVar =  varStruct.xySumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  fabs(varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean));
     varStruct.yVar =  fabs(varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean));
     varStruct.zVar =  fabs(varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean));
     varStruct.xyCoVar =  fabs(varStruct.xySumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.yMean));
     varStruct.xzCoVar =  fabs(varStruct.xzSumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.zMean));
     varStruct.yzCoVar =  fabs(varStruct.yzSumCoVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.zMean));

    float squirt;
    arm_sqrt_f32(varStruct.xVar +  varStruct.xyCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[ROLL].r = squirt * VARIANCE_SCALE;
    arm_sqrt_f32(varStruct.yVar +  varStruct.xyCoVar +  varStruct.yzCoVar, &squirt);
    kalmanFilterStateRate[PITCH].r = squirt * VARIANCE_SCALE;
    arm_sqrt_f32(varStruct.zVar +  varStruct.yzCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[YAW].r = squirt * VARIANCE_SCALE; 
}

FAST_CODE float kalman_process(kalman_t* kalmanState, volatile float input, volatile float target) {
    //project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);
    
    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;

    //if (target != 0.0f) {

    //Enable/Disable dynamic "e" value according to gyro/setpoint (With hysteresis).
    //Remove noise, like when target=5°/s and gyro=0.1°/s. Ratio doesn't mean much at theses low values.
    if(kalmanState->Dyn_e) {
        //Disable Dyn_e when (target/kalmanState->lastX) doesn't mean much
        if (((float)(fabs(target)) <= (DYN_E_LIMIT - DYN_E_HYTEREIS)) && ((float)(fabs(kalmanState->lastX)) <= (DYN_E_LIMIT - DYN_E_HYTEREIS))) {
            kalmanState->Dyn_e = false;
        }
    }else{
        //Enable Dyn_e when stick or Quad move
        if (((float)(fabs(target)) >= (DYN_E_LIMIT + DYN_E_HYTEREIS)) || ((float)(fabs(kalmanState->lastX)) >= (DYN_E_LIMIT + DYN_E_HYTEREIS))) {
            kalmanState->Dyn_e = true;
        }
    }
    if(kalmanState->Dyn_e) {
        //kalmanState->e = fabs(1.0f - (target/kalmanState->lastX));

        //Avoid division by 0.0f
        if(target == 0.0f)              { target = 0.0001f; }
        if(kalmanState->lastX == 0.0f)  { kalmanState->lastX = 0.0001f; }

        kalmanState->e = (float)fabs(1 - target/kalmanState->lastX);

        //Limit
        if(kalmanState->e > E_MAX)  { kalmanState->e = E_MAX; }
        if(kalmanState->e < E_MIN)  { kalmanState->e = E_MIN; }

    } else {
        kalmanState->e = E_MIN;
    }

    // if (target != 0.0f && kalmanState->lastX != 0.0f) {

    //     kalmanState->e = fabs(1-target/kalmanState->lastX);

    //     //Limit
    //     if(kalmanState->e > 1.0f) {
    //         kalmanState->e = 1.0f;
    //     }

    // } else {
    //     kalmanState->e = 0.1f;
    // }


    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    return kalmanState->x;
}


void kalman_update(float* input, float* output)
{

    //Get setpoints
    //--------------
        setPoint[ROLL]  = getSetpointRate(ROLL);
        setPoint[PITCH] = getSetpointRate(PITCH);
        setPoint[YAW]   = getSetpointRate(YAW);

    //Check which type of filter to apply
    //-----------------------------------
    if(gyroConfigMutable()->imuf_w > 0) {

        if(gyroConfigMutable()->imuf_w < 200) {

        //Kalman
        //------
            update_kalman_covariance(input);
            output[ROLL] = kalman_process(&kalmanFilterStateRate[ROLL], input[ROLL], setPoint[ROLL]);
            output[PITCH] = kalman_process(&kalmanFilterStateRate[PITCH], input[PITCH], setPoint[PITCH]);
            output[YAW] = kalman_process(&kalmanFilterStateRate[YAW], input[YAW], setPoint[YAW]);

            //Blackbox
            DEBUG_SET(DEBUG_KALMAN, 0, (int16_t)(input[ROLL]));
            DEBUG_SET(DEBUG_KALMAN, 1, (int16_t)(output[ROLL]));
            DEBUG_SET(DEBUG_KALMAN, 2, (int16_t)(kalmanFilterStateRate[ROLL].e * 100.0f));
            DEBUG_SET(DEBUG_KALMAN, 3, (int16_t)(kalmanFilterStateRate[ROLL].k * 10000.0f));

        } else {

        //Dyn_PT1
        //-------
            output[ROLL]  = dynLpf_process(&dynLpf[ROLL],   input[ROLL],  setPoint[ROLL]);
            output[PITCH] = dynLpf_process(&dynLpf[PITCH],  input[PITCH], setPoint[PITCH]);
            output[YAW]   = dynLpf_process(&dynLpf[YAW],    input[YAW],  setPoint[YAW]);

            //Blackbox
            DEBUG_SET(DEBUG_KALMAN, 0, (int16_t)(input[ROLL]));
            DEBUG_SET(DEBUG_KALMAN, 1, (int16_t)(output[ROLL]));
            DEBUG_SET(DEBUG_KALMAN, 2, (int16_t)(dynLpf[ROLL].Fc));
            DEBUG_SET(DEBUG_KALMAN, 3, (int16_t)(dynLpf[ROLL].pt1.k * 10000.0f));
        }

    }else{
        //If Kalman is Off copy input to output
        output[ROLL] = input[ROLL];
        output[PITCH]= input[PITCH];
        output[YAW]  = input[YAW];
    }

}

//////////////////////////////
//                          //
//      DYN PT1 PROCESS     //
//                          //
//////////////////////////////
FAST_CODE float dynLpf_process(dynLpf_t* filter, float input, float target) {

float newFc;
float Fmin    = (float)gyroConfigMutable()->dynlpf_fmin;
float Fmax    = (float)gyroConfigMutable()->dynlpf_fmax;
float DynGain = (float)(gyroConfigMutable()->dynlpf_gain) * 0.1f;

    //Check if we are in dynamic or fixed "e"
    //---------------------------------------
        if(filter->Dyn_Fc) {
            //Disable Dyn_Fc when (target/kalmanState->lastX) doesn't mean much
            if (((float)(fabs(target)) <= (DYN_E_LIMIT - DYN_E_HYTEREIS)) && ((float)(fabs(filter->pt1.state)) <= (DYN_E_LIMIT - DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = false;
            }
        }else{
            //Enable Dyn_Fc when stick or Quad move
            if (((float)(fabs(target)) >= (DYN_E_LIMIT + DYN_E_HYTEREIS)) || ((float)(fabs(filter->pt1.state)) >= (DYN_E_LIMIT + DYN_E_HYTEREIS))) {
                filter->Dyn_Fc = true;
            }
        }

    //Compute e & Fc
    //--------------
        if(filter->Dyn_Fc) {
            
        //Avoid division by 0.0f
            if(target == 0.0f)              { target = 0.00001f; }
            if(filter->pt1.state == 0.0f)   { filter->pt1.state = 0.00001f; }

        //Compute e factor
            float e;
            switch(gyroConfigMutable()->imuf_w) {
                case 300 :
                    //e = Gain * Error / Avg(target;filtered)
                    float Mean  = (target + filter->pt1.state) * 0.5f;          //Traget Gyro mean
                    float Error = (float)(fabs( target - filter->pt1.state ));  //Target Gyro error 
                    e = (DynGain * Error / Mean );
                    break;
                default :
                    //IMU-F style
                    e = fabs(1-target/filter->pt1.state);
                    break;
            }

        //New freq
            newFc = Fmax * e;

        //Limit
            if(newFc > Fmax)  { newFc  = Fmax; }
            if(newFc < Fmin)  { newFc  = Fmin; }

        } else {
            newFc  = Fmin;
        }

    //Update PT1 filter
    //------------------
        const float gyroDt = gyro.targetLooptime * 1e-6f;
        pt1FilterUpdateCutoff(&filter->pt1, pt1FilterGain(newFc, gyroDt));
        filter->Fc = newFc;

    //Apply filter
    //------------
        float output = pt1FilterApply(&filter->pt1, input);

 return output;
}


#pragma GCC pop_options
