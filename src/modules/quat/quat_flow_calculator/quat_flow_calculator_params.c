/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Samuel Zihlmann <samuezih@ee.ethz.ch>
 *   		 Lorenz Meier <lm@inf.ethz.ch>
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
 * @file quat_flow_calculator_params.c
 * 
 * Parameters for flow calculator */

#include "quat_flow_calculator_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(QFC_LO_THRUST, 0.2f);
PARAM_DEFINE_FLOAT(QFC_SONAR_LP_U, 0.5f);
PARAM_DEFINE_FLOAT(QFC_SONAR_LP_L, 0.2f);
PARAM_DEFINE_INT32(QFC_MIN_QUA, 30);
PARAM_DEFINE_INT32(QFC_DEBUG, 0);
PARAM_DEFINE_FLOAT(QFC_MAX_VEL, 1.0f);
PARAM_DEFINE_FLOAT(QFC_FLOW_K, 0.0165f);


int parameters_init(struct quat_flow_calculator_param_handles *h)
{
	/* PID parameters */
	h->minimum_liftoff_thrust	=	param_find("QFC_LO_THRUST");
	h->sonar_upper_lp_threshold	=	param_find("QFC_SONAR_LP_U");
	h->sonar_lower_lp_threshold	=	param_find("QFC_SONAR_LP_L");
	h->debug					=	param_find("QFC_DEBUG");
	h->minimum_quality 			= 	param_find("QFC_MIN_QUA");
	h->max_velocity				= 	param_find("QFC_MAX_VEL");
	h->flow_k					= 	param_find("QFC_FLOW_K");
	return OK;
}

int parameters_update(const struct quat_flow_calculator_param_handles *h, struct quat_flow_calculator_params *p)
{
	param_get(h->minimum_liftoff_thrust, &(p->minimum_liftoff_thrust));
	param_get(h->sonar_upper_lp_threshold, &(p->sonar_upper_lp_threshold));
	param_get(h->sonar_lower_lp_threshold, &(p->sonar_lower_lp_threshold));
	param_get(h->debug, &(p->debug));
	param_get(h->minimum_quality, &(p->minimum_quality));
	param_get(h->max_velocity, &(p->max_velocity));
	param_get(h->flow_k, &(p->flow_k));

	return OK;
}
