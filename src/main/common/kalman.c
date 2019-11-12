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


variance_t varStruct;
volatile float setPoint[3];
kalman_t kalmanFilterStateRate[3];


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

inline float kalman_process(kalman_t* kalmanState, volatile float input, volatile float target) {
    //project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);
    
    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;



    //if (target != 0.0f) {

    //Enable/Disable dynamic "e" value according to gyro/setpoint (With hysteresis).
    //Remove noise, like when target=5°/s and gyro=0.1°/s. Ratio doesn't mean much at theses low values.
    #define DYN_E_LIMIT     10.0f  //Value in °/s
    #define DYN_E_HYTEREIS  2.0f  //Value in °/s

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
        kalmanState->e = fabs(1.0f - (target/kalmanState->lastX));
    } else {
        kalmanState->e = 1.0f;
    }
    
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
    if(gyroConfigMutable()->imuf_w > 0) {
        setPoint[ROLL] = getSetpointRate(ROLL);
        setPoint[PITCH] = getSetpointRate(PITCH);
        setPoint[YAW] = getSetpointRate(YAW);

        update_kalman_covariance(input);
        output[ROLL] = kalman_process(&kalmanFilterStateRate[ROLL], input[ROLL], setPoint[ROLL]);
        output[PITCH] = kalman_process(&kalmanFilterStateRate[PITCH], input[PITCH], setPoint[PITCH]);
        output[YAW] = kalman_process(&kalmanFilterStateRate[YAW], input[YAW], setPoint[YAW]);
    }else{
        //If Kalman is Off copy input to output
        output[ROLL] = input[ROLL];
        output[PITCH]= input[PITCH];
        output[YAW]  = input[YAW];
    }

    //Blackbox
    DEBUG_SET(DEBUG_KALMAN, 0, (int16_t)(input[ROLL]));
    DEBUG_SET(DEBUG_KALMAN, 1, (int16_t)(output[ROLL]));
    DEBUG_SET(DEBUG_KALMAN, 2, (int16_t)(kalmanFilterStateRate[ROLL].e * 100.0f));
    DEBUG_SET(DEBUG_KALMAN, 3, (int16_t)(kalmanFilterStateRate[ROLL].k * 10000.0f));

    //For simply test if Kalman is called
    //static uint16_t KalmanAliveCounter = 0;
    //KalmanAliveCounter++;
    //DEBUG_SET(DEBUG_KALMAN, 3, (uint16_t)KalmanAliveCounter);
}

#pragma GCC pop_options
