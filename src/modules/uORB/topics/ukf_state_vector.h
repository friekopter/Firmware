/*
 * state_vector.h
 *
 *  Created on: Sep 23, 2013
 *      Author: fludwig
 */

#ifndef UKF_STATE_VECTOR_H_
#define UKF_STATE_VECTOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

struct ukf_state_vector_s {
	float vel_x;
	float vel_y;
	float vel_d;
	float acc_bias_x;
	float acc_bias_y;
	float acc_bias_z;
	float gyo_bias_x;
	float gyo_bias_y;
	float gyo_bias_z;
	float pos_x;
	float pos_y;
	float pos_d;
	float pres_alt;


};

/* register this as object request broker structure */
ORB_DECLARE(ukf_state_vector);

#endif /* UKF_STATE_VECTOR_H_ */
