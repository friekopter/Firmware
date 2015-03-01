
 

#include <quat/utils/pid.h>
#include <quat/utils/util.h>
#include "quat_att_control_params.h"
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
/**
 * @file
 *   @brief Controls the quadrotor attitude
 */
 

#ifndef QUAT_ATT_CONTROL_H_
#define QUAT_ATT_CONTROL_H_

void control_quadrotor_attitude_reset(void);

void control_initFilter(void);

void control_quadrotor_attitude_init(	const struct attitude_pid_quat_params *tilt_rate,
		const struct attitude_pid_quat_params *tilt_angle,
		const struct attitude_pid_quat_params *yaw_rate,
		const struct attitude_pid_quat_params *yaw_angle,
		const struct attitude_control_quat_params *control);

void control_quadrotor_attitude(
		const struct vehicle_attitude_setpoint_s *att_sp,
		const struct vehicle_attitude_s *att,
		const struct vehicle_rates_setpoint_s *rate_sp,
		const struct attitude_control_quat_params *control,
		struct actuator_controls_s *actuators);

void control_quadrotor_set_yaw(float yaw);

float control_quadrotor_get_yaw(void);

typedef struct {
    float yawSetpoint;

    float navPitchTarget;
    float navRollTarget;

    utilFilter_t pitchFilter[3];
    utilFilter_t rollFilter[3];

    pidStruct_t *rollRate;
    pidStruct_t *pitchRate;
    pidStruct_t *yawRate;

    pidStruct_t *rollAngle;
    pidStruct_t *pitchAngle;
    pidStruct_t *yawAngle;

    unsigned long lastUpdate;		// time of raw data that this structure is based on
} controlStruct_t;

extern controlStruct_t controlData;


#endif /* QUAT_ATT_CONTROL_H_ */


