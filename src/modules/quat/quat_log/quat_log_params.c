/*
 * quat_log_params.c
 *
 *  Created on: Oct 9, 2013
 *      Author: fludwig
 */

#include "quat_log_params.h"

PARAM_DEFINE_INT32(Q_LOG_INTV, 100);

int quat_log_parameters_init(struct quat_log_param_handles *paramHandles)
{
	paramHandles->q_log_interval = param_find("Q_LOG_INTV");
	return OK;
}

int quat_log_parameters_update(const struct quat_log_param_handles *handles,
		struct quat_log_params *params)
{
	param_get(handles->q_log_interval, &(params->q_log_interval));
	if(params->q_log_interval < 1) params->q_log_interval = 1;
	return OK;
}


