#ifndef PID_H_
#define PID_H_
#include "inttypes.h"
#include <systemlib/visibility.h>

__BEGIN_DECLS

typedef struct {
    //float setPoint;		// Last setpoint
    float dState;		// Last position input
    float iState;		// Integrator state
    const float *iGain;		// integral gain
    const float *pGain;		// proportional gain
    const float *dGain;		// derivative gain
    const float *fGain;		// low pass filter factor (1 - pole) for derivative gain
    const float *pMax, *iMax, *dMax, *oMax;
    const int *pTrim, *iTrim, *dTrim, *fTrim;	// pointers to radio trim channels (or NULL)
    float pv_1;//, pv_2;
    float co_1;
    float pTerm_1;
    float iTerm_1;
    float dTerm_1;
    //float sp_1;
} pidStruct_t;

__EXPORT  pidStruct_t *pidInit(const float *p,const float *i,const float *d,const float *f,const float *pMax,const float *iMax,const float *dMax,const float *oMax,const int *pTrim,const int *iTrim,const int *dTrim,const int *fTrim);
__EXPORT  float pidUpdate(pidStruct_t *pid, float setpoint, float position);
__EXPORT  float pidUpdateTest(pidStruct_t *pid, float setpoint, float position);
__EXPORT  void pidZeroIntegral(pidStruct_t *pid, float pv, float iState);


__END_DECLS

#endif /* PID_H_ */
