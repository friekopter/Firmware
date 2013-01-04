#ifndef PID_H_
#define PID_H_
#include "inttypes.h"

typedef struct {
    float setPoint;		// Last setpoint
    float dState;		// Last position input
    float iState;		// Integrator state
    float *iGain;		// integral gain
    float *pGain;		// proportional gain
    float *dGain;		// derivative gain
    float *fGain;		// low pass filter factor (1 - pole) for derivative gain
    float *pMax, *iMax, *dMax, *oMax;
    int *pTrim, *iTrim, *dTrim, *fTrim;	// pointers to radio trim channels (or NULL)
    float pv_1, pv_2;
    float co_1;
    float pTerm_1;
    float iTerm_1;
    float dTerm_1;
    float sp_1;
} pidStruct_t;

extern pidStruct_t *pidInit(float *p, float *i, float *d, float *f, float *pMax, float *iMax, float *dMax, float *oMax, int *pTrim, int *iTrim, int *dTrim, int *fTrim);
extern float pidUpdate(pidStruct_t *pid, float setpoint, float position);
extern float pidUpdateTest(pidStruct_t *pid, float setpoint, float position);
void pidZeroIntegral(pidStruct_t *pid, float pv, float iState);


#endif /* PID_H_ */
