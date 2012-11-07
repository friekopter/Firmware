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

struct attitude_estimator_quat_params {
	float
	tilt_rate_p,
	tilt_rate_i,
	tilt_rate_d,
	tilt_rate_f,
	tilt_rate_max_p,
	tilt_rate_max_i,
	tilt_rate_max_d,
	tilt_rate_max_f,
	tilt_angle_p,
	tilt_angle_i,
	tilt_angle_d,
	tilt_angle_f,
	tilt_angle_max_p,
	tilt_angle_max_i,
	tilt_angle_max_d,
	tilt_angle_max_f
	;
};

struct attitude_estimator_quat_param_handles {
	param_t accdist, kp, ki, ka, km1, km2;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct attitude_estimator_quat_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct attitude_estimator_quat_param_handles *source, struct attitude_estimator_quat_params *destination);

#endif //#define _quat_att_control_params_h_
