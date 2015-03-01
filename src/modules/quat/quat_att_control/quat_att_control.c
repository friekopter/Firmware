#include "quat_att_control.h"
#include "inttypes.h"
#include "debug.h"
#include <float.h>
#include <uORB/topics/actuator_controls.h>

#include <quat/utils/pid.h>
#include <quat/utils/compass_utils.h>
#include <quat/utils/util.h>


#include <math.h>
#include <stdbool.h>


controlStruct_t controlData;


uint32_t controller_counter = 0;

void control_quadrotor_set_yaw(float yaw)
{
	controlData.yawSetpoint = yaw;
}

float control_quadrotor_get_yaw()
{
	return controlData.yawSetpoint;
}

void control_quadrotor_attitude_reset()
{
	pidZeroIntegral(controlData.pitchRate,0.0f,0.0f);
	pidZeroIntegral(controlData.rollRate,0.0f,0.0f);
	pidZeroIntegral(controlData.yawRate,0.0f,0.0f);
	pidZeroIntegral(controlData.pitchAngle,0.0f,0.0f);
	pidZeroIntegral(controlData.rollAngle,0.0f,0.0f);
	pidZeroIntegral(controlData.yawAngle,0.0f,0.0f);
}

void control_initFilter() {
    int i;
    for (i = 0; i < 3; i++)
    {
    	utilFilterInit(&controlData.pitchFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    	utilFilterInit(&controlData.rollFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    }
}

void control_quadrotor_attitude_init(		const struct attitude_pid_quat_params *tilt_rate,
											const struct attitude_pid_quat_params *tilt_angle,
											const struct attitude_pid_quat_params *yaw_rate,
											const struct attitude_pid_quat_params *yaw_angle,
											const struct attitude_control_quat_params *control)
{
	control_initFilter();

    controlData.pitchRate = pidInit(&tilt_rate->p, &tilt_rate->i, &tilt_rate->d, &tilt_rate->f,
    								&tilt_rate->max_p, &tilt_rate->max_i, &tilt_rate->max_d, &tilt_rate->max_o,
    								0, 0, 0, 0);
    controlData.rollRate = pidInit (&tilt_rate->p, &tilt_rate->i, &tilt_rate->d, &tilt_rate->f,
									&tilt_rate->max_p, &tilt_rate->max_i, &tilt_rate->max_d, &tilt_rate->max_o,
									0, 0, 0, 0);
    controlData.yawRate = pidInit  (&yaw_rate->p, &yaw_rate->i, &yaw_rate->d, &yaw_rate->f,
									&yaw_rate->max_p, &yaw_rate->max_i, &yaw_rate->max_d, &yaw_rate->max_o,
									0, 0, 0, 0);
    controlData.pitchAngle = pidInit(&tilt_angle->p, &tilt_angle->i, &tilt_angle->d, &tilt_angle->f,
									&tilt_angle->max_p, &tilt_angle->max_i, &tilt_angle->max_d, &tilt_angle->max_o,
									0, 0, 0, 0);
    controlData.rollAngle = pidInit(&tilt_angle->p, &tilt_angle->i, &tilt_angle->d, &tilt_angle->f,
									&tilt_angle->max_p, &tilt_angle->max_i, &tilt_angle->max_d, &tilt_angle->max_o,
									0, 0, 0, 0);
    controlData.yawAngle = pidInit(&yaw_angle->p, &yaw_angle->i, &yaw_angle->d, &yaw_angle->f,
									&yaw_angle->max_p, &yaw_angle->max_i, &yaw_angle->max_d, &yaw_angle->max_o,
									0, 0, 0, 0);
}

void control_quadrotor_attitude(
		const struct vehicle_attitude_setpoint_s *att_sp,
		const struct vehicle_attitude_s *att,
		const struct vehicle_rates_setpoint_s *rate_sp,
		const struct attitude_control_quat_params *control,
		struct actuator_controls_s *actuators)
{
    float pitchCommand, rollCommand, ruddCommand, throttleCommand;
    if(fabsf(rate_sp->yaw) < FLT_MIN) {
    	// hold heading
    	float yawRateTarget = pidUpdate(controlData.yawAngle, 0.0f, compassDifferenceRad(controlData.yawSetpoint, att->yaw));	// seek a 0 deg difference between hold heading and actual yaw
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, yawRateTarget, att->yawspeed), -control->controlMax, control->controlMax);
    }
    else {
    	// rate controls
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, rate_sp->yaw, att->yawspeed), -control->controlMax, control->controlMax);
    	control_quadrotor_set_yaw(att->yaw);
    }
    // smooth
   	float rollTarget = utilFilter3(controlData.rollFilter, att_sp->roll_body);
    // roll angle
    rollCommand = pidUpdate(controlData.rollAngle, rollTarget, att->roll);
    // rate
    rollCommand += pidUpdate(controlData.rollRate, 0.0f, att->rollspeed);
    rollCommand = constrainFloat(rollCommand, -control->controlMax, control->controlMax);

    // smooth
   	float pitchTarget = utilFilter3(controlData.pitchFilter, att_sp->pitch_body);
    // pitch angle
    pitchCommand = pidUpdate(controlData.pitchAngle, pitchTarget, att->pitch);
    // rate
    pitchCommand += pidUpdate(controlData.pitchRate, 0.0f, att->pitchspeed);
    pitchCommand = constrainFloat(pitchCommand, -control->controlMax, control->controlMax);
    throttleCommand = att_sp->thrust;
    actuators->control[0] = rollCommand;
    actuators->control[1] = pitchCommand;
    actuators->control[2] = ruddCommand;
    actuators->control[3] = throttleCommand;
}

