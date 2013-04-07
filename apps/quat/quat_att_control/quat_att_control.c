#include "quat_att_control.h"
#include "inttypes.h"
#include "debug.h"
#include <uORB/topics/actuator_controls.h>

#include <quat/utils/pid.h>
#include <quat/utils/compass_utils.h>
#include <quat/utils/util.h>


#include <math.h>
#include <stdbool.h>


controlStruct_t controlData;
controlParameter_t controlParameter;


uint32_t controller_counter = 0;

inline void control_quadrotor_attitude_reset()
{
	pidZeroIntegral(controlData.pitchRate,0.0f,0.0f);
	pidZeroIntegral(controlData.rollRate,0.0f,0.0f);
	pidZeroIntegral(controlData.yawRate,0.0f,0.0f);
	pidZeroIntegral(controlData.pitchAngle,0.0f,0.0f);
	pidZeroIntegral(controlData.rollAngle,0.0f,0.0f);
	pidZeroIntegral(controlData.yawAngle,0.0f,0.0f);
}

inline void control_quadrotor_attitude_init(const struct attitude_pid_quat_params *tilt_rate,
											const struct attitude_pid_quat_params *tilt_angle,
											const struct attitude_pid_quat_params *yaw_rate,
											const struct attitude_pid_quat_params *yaw_angle,
											const struct attitude_control_quat_params *control)
{
    int i;
    for (i = 0; i < 3; i++)
    {
    	utilFilterInit(&controlData.pitchFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    	utilFilterInit(&controlData.rollFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    }

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
    controlParameter.paramControlDeadBand 	= &control->controlDeadBand;
    controlParameter.paramControlPitchF 		= &control->controlPitchF;
    controlParameter.paramControlRollF 		= &control->controlRollF;
    controlParameter.paramControlThrottleF 	= &control->controlThrottleF;
    controlParameter.paramControlYawF 			= &control->controlYawF;
}

inline void control_quadrotor_attitude(
		const struct vehicle_attitude_setpoint_s *att_sp,
		const struct vehicle_attitude_s *att,
		const struct vehicle_rates_setpoint_s *rate_sp,
		const struct attitude_control_quat_params *control,
		struct actuator_controls_s *actuators)
{
    float pitchCommand, rollCommand, ruddCommand, throttleCommand;
/*
    throttleCommand = 0;
    userThrottle = control->controlHoverThrottle;
    //First handle throttle that controls altitude
    // are we in altitude hold mode?
    if (navData.status > NAV_STATUS_MANUAL)
    {
    	throttleCommand = pidUpdate(navData.altSpeedPID, navData.holdSpeedAlt, navGetVel('D'));
    }
    else
    {
    	userThrottle = (float)REMOTE_THROT * *controlParameter.paramControlThrottleF;;
    }

    //Second handle direction
    userRudd = REMOTE_RUDD * *controlParameter.paramControlYawF;
    // if motors are not running, use this heading as hold heading
    if (global_data.active_throttle == 0)
    {
    	navData.holdHeading = AQ_YAW;
    }
    // implement rudder dead zone
    if (	REMOTE_RUDD > *controlParameter.paramControlDeadBand ||
    		REMOTE_RUDD < -*controlParameter.paramControlDeadBand)
    {
    	//If we move the rudder stick disable heading lock
    	navData.holdHeading = AQ_YAW;
    	//pidUpdate(controlData.yawAngle, 0.0f, 0.0f);
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, userRudd, AQ_HRATEZ), -configData.controlMax, configData.controlMax);
    }
    else
    {
    	//yaw stick in normal position, do hold heading
    	controlData.yawRateTarget = pidUpdate(controlData.yawAngle, 0.0f, compassDifference(navData.holdHeading, AQ_YAW));	// seek a 0 deg difference between hold heading and actual yaw
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, controlData.yawRateTarget, AQ_HRATEZ), -configData.controlMax, configData.controlMax);
    }

    // DVH overrides direct user pitch / roll requests
    if (navData.status != NAV_STATUS_DVH)
    {
    	controlData.userPitchTarget = REMOTE_PITCH * *controlParameter.paramControlPitchF;
    	controlData.userRollTarget = REMOTE_ROLL * *controlParameter.paramControlRollF;
    }
    else
    {
    	controlData.userPitchTarget = 0.0f;
    	controlData.userRollTarget = 0.0f;
    }

    if(navData.status > NAV_STATUS_ALTHOLD)
    {
    	controlData.navPitchTarget = navData.holdTiltN;
    	controlData.navRollTarget = navData.holdTiltE;
    }
    else
    {
    	controlData.navPitchTarget = 0.0f;
    	controlData.navRollTarget = 0.0f;
    }

    // smooth
   	controlData.userPitchTarget = utilFilter3(controlData.userPitchFilter, controlData.userPitchTarget);
   	controlData.userRollTarget = utilFilter3(controlData.userRollFilter, controlData.userRollTarget);

   	// smooth
   	controlData.navPitchTarget = utilFilter3(controlData.navPitchFilter, controlData.navPitchTarget);
   	controlData.navRollTarget = utilFilter3(controlData.navRollFilter, controlData.navRollTarget);

    // rotate nav's NE rotation to our craft's frame of reference
    pitch = controlData.navPitchTarget * quatData.yawCos - controlData.navRollTarget * quatData.yawSin;
    roll = controlData.navRollTarget *  quatData.yawCos + controlData.navPitchTarget * quatData.yawSin;

	// combine nav & user requests (both are already smoothed)
	pitch = pitch + controlData.userPitchTarget;
	roll = roll + controlData.userRollTarget;
*/
    if(rate_sp->yaw == 0.0f)
    {
    	// hold heading
    	float yawRateTarget = pidUpdate(controlData.yawAngle, 0.0f, compassDifference(controlData.yawSetpoint, att->yaw));	// seek a 0 deg difference between hold heading and actual yaw
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, yawRateTarget, att->yawspeed), -control->controlMax, control->controlMax);
    }
    else
    {
    	// rate controls
    	ruddCommand = constrainFloat(pidUpdate(controlData.yawRate, rate_sp->yaw, att->yawspeed), -control->controlMax, control->controlMax);
    	controlData.yawSetpoint = att->yaw;
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
