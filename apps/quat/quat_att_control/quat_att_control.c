#include "quat_att_control.h"
#include "conf.h"
#include "inttypes.h"
#include "global_data.h"
// Include comm
#include "comm.h"
#include "debug.h"

#include "transformation.h"
#include "pid.h"
#include "control_quadrotor_start_land.h"
#include "remote_control.h"
#include "nav.h"
#include "aq.h"
#include "quat.h"
#include "motors.h"
#include "imu_config.h"
#include "compass_utils.h"
#include "util.h"

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
PARAM_DEFINE_FLOAT(PARAM_PID_TILT_RATE_P, 0.0f);

inline void control_quadrotor_attitude_init()
{
    int i;
    for (i = 0; i < 3; i++)
    {
    	utilFilterInit(&controlData.userPitchFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    	utilFilterInit(&controlData.userRollFilter[i], 1.0f/100.0f, 0.1f, 0.0f);
    	utilFilterInit(&controlData.navPitchFilter[i], 1.0f/100.0f, 0.125f, 0.0f);
    	utilFilterInit(&controlData.navRollFilter[i], 1.0f/100.0f, 0.125f, 0.0f);
    }

    controlData.pitchRate = pidInit(&global_data.param[PARAM_PID_TILT_RATE_P], &global_data.param[PARAM_PID_TILT_RATE_I], &global_data.param[PARAM_PID_TILT_RATE_D], &global_data.param[PARAM_PID_TILT_RATE_F],
    								&global_data.param[PARAM_PID_TILT_RATE_MAX_P], &global_data.param[PARAM_PID_TILT_RATE_MAX_I], &global_data.param[PARAM_PID_TILT_RATE_MAX_D], &global_data.param[PARAM_PID_TILT_RATE_MAX_O],
    								0, 0, 0, 0);
    controlData.rollRate = pidInit( &global_data.param[PARAM_PID_TILT_RATE_P], &global_data.param[PARAM_PID_TILT_RATE_I], &global_data.param[PARAM_PID_TILT_RATE_D], &global_data.param[PARAM_PID_TILT_RATE_F],
									&global_data.param[PARAM_PID_TILT_RATE_MAX_P], &global_data.param[PARAM_PID_TILT_RATE_MAX_I], &global_data.param[PARAM_PID_TILT_RATE_MAX_D], &global_data.param[PARAM_PID_TILT_RATE_MAX_O],
									0, 0, 0, 0);
    controlData.yawRate = pidInit(  &global_data.param[PARAM_PID_YAW_RATE_P], &global_data.param[PARAM_PID_YAW_RATE_I], &global_data.param[PARAM_PID_YAW_RATE_D], &global_data.param[PARAM_PID_YAW_RATE_F],
									&global_data.param[PARAM_PID_YAW_RATE_MAX_P], &global_data.param[PARAM_PID_YAW_RATE_MAX_I], &global_data.param[PARAM_PID_YAW_RATE_MAX_D], &global_data.param[PARAM_PID_YAW_RATE_MAX_O],
									0, 0, 0, 0);
    controlData.pitchAngle = pidInit(&global_data.param[PARAM_PID_TILT_ANGLE_P], &global_data.param[PARAM_PID_TILT_ANGLE_I], &global_data.param[PARAM_PID_TILT_ANGLE_D], &global_data.param[PARAM_PID_TILT_ANGLE_F],
									 &global_data.param[PARAM_PID_TILT_ANGLE_MAX_P], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_I], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_D], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_O],
									 0, 0, 0, 0);
    controlData.rollAngle = pidInit(&global_data.param[PARAM_PID_TILT_ANGLE_P], &global_data.param[PARAM_PID_TILT_ANGLE_I], &global_data.param[PARAM_PID_TILT_ANGLE_D], &global_data.param[PARAM_PID_TILT_ANGLE_F],
									&global_data.param[PARAM_PID_TILT_ANGLE_MAX_P], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_I], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_D], &global_data.param[PARAM_PID_TILT_ANGLE_MAX_O],
									0, 0, 0, 0);
    controlData.yawAngle = pidInit(&global_data.param[PARAM_PID_YAW_ANGLE_P], &global_data.param[PARAM_PID_YAW_ANGLE_I], &global_data.param[PARAM_PID_YAW_ANGLE_D], &global_data.param[PARAM_PID_YAW_ANGLE_F],
								   &global_data.param[PARAM_PID_YAW_ANGLE_MAX_P], &global_data.param[PARAM_PID_YAW_ANGLE_MAX_I], &global_data.param[PARAM_PID_YAW_ANGLE_MAX_D], &global_data.param[PARAM_PID_YAW_ANGLE_MAX_O],
								   0, 0, 0, 0);
    controlParameter.paramControlDeadBand 	= &global_data.param[PARAM_CONTROL_DEAD_BAND_YAW];
    controlParameter.paramControlPitchF 		= &global_data.param[PARAM_CONTROL_PITCH_F];
    controlParameter.paramControlRollF 		= &global_data.param[PARAM_CONTROL_ROLL_F];
    controlParameter.paramControlThrottleF 	= &global_data.param[PARAM_CONTROL_THROTTLE_F];
    controlParameter.paramControlYawF 			= &global_data.param[PARAM_CONTROL_YAW_F];
}

inline void control_quadrotor_attitude()
{
    float pitch, roll, userThrottle, userRudd;
    float pitchCommand, rollCommand, ruddCommand, throttleCommand;

    throttleCommand = 0;
    userThrottle = configData.motorsHoverThrot;
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

    // roll angle
    rollCommand = pidUpdate(controlData.rollAngle, roll, AQ_ROLL);
    // rate
    rollCommand += pidUpdate(controlData.rollRate, 0.0f, AQ_HRATEX);
    rollCommand = constrainFloat(rollCommand, -configData.controlMax, configData.controlMax);

    // pitch angle
    pitchCommand = pidUpdate(controlData.pitchAngle, pitch, AQ_PITCH);
    // rate
    pitchCommand += pidUpdate(controlData.pitchRate, 0.0f, AQ_HRATEY);
    pitchCommand = constrainFloat(pitchCommand, -configData.controlMax, configData.controlMax);


    //now transmit commands to the motor
    motorsCommands(throttleCommand + userThrottle, pitchCommand, rollCommand, ruddCommand);

	if ((	global_data.state.mav_mode == MAV_MODE_MANUAL ||
			global_data.state.mav_mode == MAV_MODE_GUIDED 	) &&
		(	global_data.state.status == MAV_STATE_ACTIVE  ||
			global_data.state.status == MAV_STATE_CRITICAL||
			global_data.state.status == MAV_STATE_EMERGENCY	))
	{
		// Set MOTORS
#ifndef SCISIM
		motor_i2c_set_11bit_pwm(MOT1_I2C_SLAVE_ADDRESS, *motorsData.m1p);
		motor_i2c_set_11bit_pwm(MOT2_I2C_SLAVE_ADDRESS, *motorsData.m2p);
		motor_i2c_set_11bit_pwm(MOT3_I2C_SLAVE_ADDRESS, *motorsData.m3p);
		motor_i2c_set_11bit_pwm(MOT4_I2C_SLAVE_ADDRESS, *motorsData.m4p);
#endif
		global_data.active_throttle = motorsData.throttle;
	}
	else
	{
			// Set MOTORS to 0 thrust
#ifndef SCISIM
		motor_i2c_set_pwm(MOT1_I2C_SLAVE_ADDRESS, 0);
		motor_i2c_set_pwm(MOT2_I2C_SLAVE_ADDRESS, 0);
		motor_i2c_set_pwm(MOT3_I2C_SLAVE_ADDRESS, 0);
		motor_i2c_set_pwm(MOT4_I2C_SLAVE_ADDRESS, 0);
#endif
		global_data.active_throttle = 0;
	}
#ifndef SCISIM
	//DEBUGGING
	if (controller_counter++ == 200)
	{
		controller_counter = 0;

		if (global_data.param[PARAM_SEND_SLOT_DEBUG_3] == 1)
		{
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 21, REMOTE_ROLL);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 22, REMOTE_PITCH);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 23, REMOTE_RUDD);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 24, REMOTE_THROT);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 25, roll);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 26, pitch);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 27, controlData.yawRateTarget);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 28, compassDifference(navData.holdHeading, AQ_YAW));
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 29, navData.holdHeading);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 30, rollCommand);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 31, pitchCommand);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 32, ruddCommand);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 33, throttleCommand + userThrottle);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 34, motorsData.throttle);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 35, global_data.active_throttle);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 36, global_data.temperature_bctrl1_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 37, global_data.temperature_bctrl2_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 38, global_data.temperature_bctrl3_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 39, global_data.temperature_bctrl4_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 40, global_data.current_bctrl1_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 41, global_data.current_bctrl2_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 42, global_data.current_bctrl3_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 43, global_data.current_bctrl4_si);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 44, global_data.temperature_gyros);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 45, navData.holdSpeedAlt);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 46, navData.holdAlt);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 47, REMOTE_HOME);
			//mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 48, AQ_HRATEY);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 49, AQ_YAW);
			mavlink_msg_debug_send(global_data.param[PARAM_SEND_DEBUGCHAN], 50, AQ_HRATEZ);
		}
	}
#endif
}
