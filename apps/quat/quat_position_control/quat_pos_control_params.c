
/*
 * @file quat_position_control_params.c
 * 
 * Parameters for EKF filter
 */

#include "quat_pos_control_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
PARAM_DEFINE_FLOAT(MC_POS_P, 0.2f);

int parameters_init(struct quat_position_control_param_handles *h)
{
	/* PID parameters */
	h->p 	=	param_find("MC_POS_P");

	return OK;
}

int parameters_update(const struct quat_position_control_param_handles *h, struct quat_position_control_params *p)
{
	param_get(h->p, &(p->p));

	return OK;
}
