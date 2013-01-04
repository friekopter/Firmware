/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author:
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
 * @file quat_att_control_params.h
 * 
 * Parameters for Quat filter
 */
#ifndef _quat_att_control_params_h_
#define _quat_att_control_params_h_

#include <systemlib/param/param.h>

struct attitude_control_quat_param_handles {
	param_t
    controlDeadBand,
    controlPitchF,
    controlRollF,
    controlThrottleF,
    controlYawF,
    controlMax;
};

struct attitude_control_quat_params {
	float
    controlDeadBand,
    controlPitchF,
    controlRollF,
    controlThrottleF,
    controlYawF,
    controlMax;
};

struct attitude_pid_quat_param_handles {
	param_t
	p,
	i,
	d,
	f,
	max_p,
	max_i,
	max_d,
	max_o;
};

struct attitude_pid_quat_params {
	float
	p,
	i,
	d,
	f,
	max_p,
	max_i,
	max_d,
	max_o;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct attitude_pid_quat_param_handles *tiltRate,
					struct attitude_pid_quat_param_handles *tiltAngle,
					struct attitude_pid_quat_param_handles *yawRate,
					struct attitude_pid_quat_param_handles *yawAngle,
					struct attitude_control_quat_param_handles *control);

/**
 * Update all parameters
 *
 */
int parameters_update(	struct attitude_pid_quat_param_handles *tiltRate,
						struct attitude_pid_quat_param_handles *tiltAngle,
						struct attitude_pid_quat_param_handles *yawRate,
						struct attitude_pid_quat_param_handles *yawAngle,
						struct attitude_control_quat_param_handles *control,
						struct attitude_pid_quat_params *tiltRateDest,
						struct attitude_pid_quat_params *tiltAngleDest,
						struct attitude_pid_quat_params *yawRateDest,
						struct attitude_pid_quat_params *yawAngleDest,
						struct attitude_control_quat_params *controlDest);

#endif //#define _quat_att_control_params_h_
