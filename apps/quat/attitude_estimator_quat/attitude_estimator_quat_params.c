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
 * @file attitude_estimator_ekf_params.c
 * 
 * Parameters for EKF filter
 */

#include "attitude_estimator_quat_params.h"

/* Extended Kalman Filter covariances */


/* Acceleration suppression factor */
PARAM_DEFINE_FLOAT(QUA_ATT_ACCDIST, 20.0f);
/* Gyro proportional */
PARAM_DEFINE_FLOAT(QUA_ATT_KP, 0.5f);
/* Gyro integration */
PARAM_DEFINE_FLOAT(QUA_ATT_KI, 0.05f);
/* Acceleration proportional */
PARAM_DEFINE_FLOAT(QUA_ATT_KA, 0.01f);
/* Magnetic x,y direction proportional */
PARAM_DEFINE_FLOAT(QUA_ATT_KM1, 0.0003f);
/* Magnetic z direction proportional */
PARAM_DEFINE_FLOAT(QUA_ATT_KM2, 0.004f);

int parameters_init(struct attitude_estimator_quat_param_handles *h)
{
	/* PID parameters */
	h->accdist 	=	param_find("QUA_ATT_ACCDIST");
	h->kp 	=	param_find("QUA_ATT_KP");
	h->ki 	=	param_find("QUA_ATT_KI");
	h->ka 	=	param_find("QUA_ATT_KA");
	h->km1 	=	param_find("QUA_ATT_KM1");
	h->km2 	=	param_find("QUA_ATT_KM2");
	return h->accdist == PARAM_INVALID |
			h->kp == PARAM_INVALID |
			h->ki == PARAM_INVALID |
			h->ka == PARAM_INVALID |
			h->km1 == PARAM_INVALID |
			h->km2 == PARAM_INVALID;
}


int parameters_update(const struct attitude_estimator_quat_param_handles *source, struct attitude_estimator_quat_params *destination)
{
	int result = 0;
	result = result | param_get(source->accdist, &(destination->accdist));
	result = result | param_get(source->kp, &(destination->kp));
	result = result | param_get(source->ki, &(destination->ki));
	result = result | param_get(source->ka, &(destination->ka));
	result = result | param_get(source->km1, &(destination->km1));
	result = result | param_get(source->km2, &(destination->km2));
	return result;
}
