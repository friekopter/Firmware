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

/**
 * @file filtered_bottom_flow.h
 * Definition of the filtered bottom flow uORB topic.
 */

#ifndef TOPIC_FILTERED_BOTTOM_FLOW_H_
#define TOPIC_FILTERED_BOTTOM_FLOW_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Filtered bottom flow in bodyframe.
 */
struct filtered_bottom_flow_s
{
	uint64_t timestamp;		/**< time of this estimate, in microseconds since system start */

	float sumx;				/**< Integrated bodyframe x flow in meters					   */
	float sumy;				/**< Integrated bodyframe y flow in meters			   		   */

	float vx; 				/**< Flow bodyframe x speed, m/s							   */
	float vy;				/**< Flow bodyframe y Speed, m/s 				   */
	float ground_distance;  /**< Ground distance in m */
	uint32_t sonar_counter;	/**< Raised by one if new measurement arrived */

	bool ned_xy_valid;			/**< true if x and y are valid */
	bool ned_z_valid;			/**< true if z is valid */
	bool ned_v_xy_valid;		/**< true if vy and vy are valid */
	bool ned_v_z_valid;			/**< true if vz is valid */
	bool landed;
	/* Position in local NED frame */
	float ned_x;				/**< X position in meters in NED earth-fixed frame */
	float ned_y;				/**< X position in meters in NED earth-fixed frame */
	float ned_z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
	/* Velocity in NED frame */
	float ned_vx; 				/**< Ground X Speed (Latitude), m/s in NED */
	float ned_vy;				/**< Ground Y Speed (Longitude), m/s in NED */
	float ned_vz;				/**< Ground Z Speed (Altitude), m/s	in NED */
	/* Heading */
	float yaw;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(filtered_bottom_flow);

#endif
