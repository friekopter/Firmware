/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Friedemann Ludwig
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file attitude_control_quat_params.c
 * 
 * Parameters attitude control
 */

#include "quat_att_control_params.h"

// Range actuators old: 0...2047 new 0...1 => divided by 2000
// Angel old degree, new rad => multiply by 57
// => divide by 53
PARAM_DEFINE_FLOAT(QUA_TILT_R_P, 0.0f);//0
PARAM_DEFINE_FLOAT(QUA_TILT_R_I, 0.0f);//0
PARAM_DEFINE_FLOAT(QUA_TILT_R_D, 1.5f);//7180.0f
PARAM_DEFINE_FLOAT(QUA_TILT_R_F, 35e-5f);//0.25f
PARAM_DEFINE_FLOAT(QUA_TILT_R_MA_P, 0.0f);//0.0
PARAM_DEFINE_FLOAT(QUA_TILT_R_MA_I, 0.0f);//0.0
PARAM_DEFINE_FLOAT(QUA_TILT_R_MA_D, 0.24f);//999.0f
PARAM_DEFINE_FLOAT(QUA_TILT_R_MA_O, 0.24f);//250.0f

PARAM_DEFINE_FLOAT(QUA_TILT_A_P, 0.4f);//60
PARAM_DEFINE_FLOAT(QUA_TILT_A_I, 15e-7f);//0.0005
PARAM_DEFINE_FLOAT(QUA_TILT_A_D, 10.0f);//1744
PARAM_DEFINE_FLOAT(QUA_TILT_A_F, 35e-5f);//0.25
PARAM_DEFINE_FLOAT(QUA_TILT_A_MA_P, 0.24f);//150
PARAM_DEFINE_FLOAT(QUA_TILT_A_MA_I, 62e-3f);//75
PARAM_DEFINE_FLOAT(QUA_TILT_A_MA_D, 0.24f);//150
PARAM_DEFINE_FLOAT(QUA_TILT_A_MA_O, 0.24f);//250

PARAM_DEFINE_FLOAT(QUA_YAW_R_P, 57e-1f);//300
PARAM_DEFINE_FLOAT(QUA_YAW_R_I, 28e-4f);//0.15
PARAM_DEFINE_FLOAT(QUA_YAW_R_D, 1.0f);//50
PARAM_DEFINE_FLOAT(QUA_YAW_R_F, 47e-2f);//0.25
PARAM_DEFINE_FLOAT(QUA_YAW_R_MA_P, 0.04f);//80
PARAM_DEFINE_FLOAT(QUA_YAW_R_MA_I, 0.04f);//80
PARAM_DEFINE_FLOAT(QUA_YAW_R_MA_D, 0.04f);//80
PARAM_DEFINE_FLOAT(QUA_YAW_R_MA_O, 0.1f);//180

PARAM_DEFINE_FLOAT(QUA_YAW_A_P, 5e-2f);//0.05
PARAM_DEFINE_FLOAT(QUA_YAW_A_I, 2e-5f);//0.00002
PARAM_DEFINE_FLOAT(QUA_YAW_A_D, 0.0f);//0
PARAM_DEFINE_FLOAT(QUA_YAW_A_F, 0.0f);//0
PARAM_DEFINE_FLOAT(QUA_YAW_A_MA_P, 2e-2f);//1.25
PARAM_DEFINE_FLOAT(QUA_YAW_A_MA_I, 7e-4f);//0.04
PARAM_DEFINE_FLOAT(QUA_YAW_A_MA_D, 0.0f);//0
PARAM_DEFINE_FLOAT(QUA_YAW_A_MA_O, 22e-2f);//1.25

// Range remote old -500...500 and 0...1000
// Range remote new -1...1 and 0...1
PARAM_DEFINE_FLOAT(QUA_CO_MAX, 1.0f);//1.0
// => multiply by 500
PARAM_DEFINE_FLOAT(QUA_CO_PITCHF, 1.5f);//0.003
PARAM_DEFINE_FLOAT(QUA_CO_ROLLF, 1.5f);//0.003
PARAM_DEFINE_FLOAT(QUA_CO_YAWF, 1.5f);//0.02
// => divide by 500
PARAM_DEFINE_FLOAT(QUA_CO_DB, 0.02f);//10

// => just forward to actuator
PARAM_DEFINE_FLOAT(QUA_CO_THROF, 1.0f);//2.4

int pid_update(	struct attitude_pid_quat_param_handles *source,
				struct attitude_pid_quat_params *destination);

int parameters_init(struct attitude_pid_quat_param_handles *tiltRate,
					struct attitude_pid_quat_param_handles *tiltAngle,
					struct attitude_pid_quat_param_handles *yawRate,
					struct attitude_pid_quat_param_handles *yawAngle,
					struct attitude_control_quat_param_handles *control)
{
	tiltRate->p 		=	param_find("QUA_TILT_R_P");
	tiltRate->i 		=	param_find("QUA_TILT_R_I");
	tiltRate->d 		=	param_find("QUA_TILT_R_D");
	tiltRate->f 		=	param_find("QUA_TILT_R_F");
	tiltRate->max_p 	=	param_find("QUA_TILT_R_MA_P");
	tiltRate->max_i 	=	param_find("QUA_TILT_R_MA_I");
	tiltRate->max_d 	=	param_find("QUA_TILT_R_MA_D");
	tiltRate->max_o 	=	param_find("QUA_TILT_R_MA_O");

	tiltAngle->p 		=	param_find("QUA_TILT_A_P");
	tiltAngle->i 		=	param_find("QUA_TILT_A_I");
	tiltAngle->d 		=	param_find("QUA_TILT_A_D");
	tiltAngle->f 		=	param_find("QUA_TILT_A_F");
	tiltAngle->max_p 	=	param_find("QUA_TILT_A_MA_P");
	tiltAngle->max_i 	=	param_find("QUA_TILT_A_MA_I");
	tiltAngle->max_d 	=	param_find("QUA_TILT_A_MA_D");
	tiltAngle->max_o 	=	param_find("QUA_TILT_A_MA_O");

	yawRate->p 		=	param_find("QUA_YAW_R_P");
	yawRate->i 		=	param_find("QUA_YAW_R_I");
	yawRate->d 		=	param_find("QUA_YAW_R_D");
	yawRate->f 		=	param_find("QUA_YAW_R_F");
	yawRate->max_p 	=	param_find("QUA_YAW_R_MA_P");
	yawRate->max_i 	=	param_find("QUA_YAW_R_MA_I");
	yawRate->max_d 	=	param_find("QUA_YAW_R_MA_D");
	yawRate->max_o 	=	param_find("QUA_YAW_R_MA_O");

	yawAngle->p 		=	param_find("QUA_YAW_A_P");
	yawAngle->i 		=	param_find("QUA_YAW_A_I");
	yawAngle->d 		=	param_find("QUA_YAW_A_D");
	yawAngle->f 		=	param_find("QUA_YAW_A_F");
	yawAngle->max_p 	=	param_find("QUA_YAW_A_MA_P");
	yawAngle->max_i 	=	param_find("QUA_YAW_A_MA_I");
	yawAngle->max_d 	=	param_find("QUA_YAW_A_MA_D");
	yawAngle->max_o 	=	param_find("QUA_YAW_A_MA_O");

	control->controlDeadBand 	= param_find("QUA_CO_DB");
	control->controlPitchF 		= param_find("QUA_CO_PITCHF");
	control->controlRollF 		= param_find("QUA_CO_ROLLF");
	control->controlThrottleF 	= param_find("QUA_CO_THROF");
	control->controlYawF 		= param_find("QUA_CO_YAWF");
	control->controlMax 		= param_find("QUA_CO_MAX");
	return OK;
}

int pid_update(	struct attitude_pid_quat_param_handles *source,
				struct attitude_pid_quat_params *destination)
{
	param_get(source->p, &(destination->p));
	param_get(source->i, &(destination->i));
	param_get(source->d, &(destination->d));
	param_get(source->f, &(destination->f));
	param_get(source->max_p, &(destination->max_p));
	param_get(source->max_i, &(destination->max_i));
	param_get(source->max_d, &(destination->max_d));
	param_get(source->max_o, &(destination->max_o));
	return OK;
}

int parameters_update(	struct attitude_pid_quat_param_handles *tiltRate,
						struct attitude_pid_quat_param_handles *tiltAngle,
						struct attitude_pid_quat_param_handles *yawRate,
						struct attitude_pid_quat_param_handles *yawAngle,
						struct attitude_control_quat_param_handles *control,
						struct attitude_pid_quat_params *tiltRateDest,
						struct attitude_pid_quat_params *tiltAngleDest,
						struct attitude_pid_quat_params *yawRateDest,
						struct attitude_pid_quat_params *yawAngleDest,
						struct attitude_control_quat_params *controlDest)
{
	pid_update(tiltRate, tiltRateDest);
	pid_update(tiltAngle, tiltAngleDest);
	pid_update(yawRate, yawRateDest);
	pid_update(yawAngle, yawAngleDest);

	param_get(control->controlDeadBand, &(controlDest->controlDeadBand));
	param_get(control->controlPitchF, &(controlDest->controlPitchF));
	param_get(control->controlRollF, &(controlDest->controlRollF));
	param_get(control->controlThrottleF, &(controlDest->controlThrottleF));
	param_get(control->controlYawF, &(controlDest->controlYawF));
	param_get(control->controlMax, &(controlDest->controlMax));
	return OK;
}


