
 

#include "pid.h"
#include "util.h"
/**
 * @file
 *   @brief Controls the quadrotor attitude
 */
 

#ifndef QUAT_ATT_CONTROL_H_
#define QUAT_ATT_CONTROL_H_

inline void quat_att_control_reset(void);

void quat_att_control_init(void);

void quat_att_control_cyclic(void);

typedef struct {
    unsigned long loops;
    unsigned char flying;

    float userPitchTarget;
    float userRollTarget;

    float yawRateTarget;

    float navPitchTarget;
    float navRollTarget;

    utilFilter_t userPitchFilter[3];
    utilFilter_t userRollFilter[3];

    utilFilter_t navPitchFilter[3];
    utilFilter_t navRollFilter[3];

    pidStruct_t *rollRate;
    pidStruct_t *pitchRate;
    pidStruct_t *yawRate;

    pidStruct_t *rollAngle;
    pidStruct_t *pitchAngle;
    pidStruct_t *yawAngle;

    unsigned long lastUpdate;		// time of raw data that this structure is based on
} controlStruct_t;

extern controlStruct_t controlData;


typedef struct {
	//float *paramPIDMultiplier;
	float *paramControlThrottleF;
	float *paramControlYawF;
	float *paramControlPitchF;
	float *paramControlRollF;
	float *paramControlDeadBand;
} controlParameter_t;

extern controlParameter_t controlParameter;


#endif /* QUAT_ATT_CONTROL_H_ */


