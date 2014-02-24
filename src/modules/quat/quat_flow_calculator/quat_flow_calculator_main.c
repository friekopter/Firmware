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
 * @file quat_flow_calculator_main.c
 *
 * Optical flow position estimator
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/subsystem_info.h>
#include <quat/utils/util.h>
#include <systemlib/perf_counter.h>
#include <poll.h>

#include "quat_flow_calculator_params.h"

__EXPORT int quat_flow_calculator_main(int argc, char *argv[]);
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
static bool flow_valid = false;
static orb_advert_t flow_subsystem_info_pub = -1;
static bool debug = false;
struct subsystem_info_s flow_control_info = {
	true,
	false,
	true,
	SUBSYSTEM_TYPE_OPTICALFLOW
};
/* rotation matrix for transformation of optical flow speed vectors */
static const int8_t rotM_flow_sensor[3][3] =   {{  0,-1, 0 },
												{  1, 0, 0 },
												{  0, 0, 1 }}; // 90deg rotated

int quat_flow_calculator_thread_main(int argc, char *argv[]);
static void usage(const char *reason);
uint8_t quat_flow_calculate_flow(
		struct quat_flow_calculator_params* params,
		struct vehicle_local_position_s* local_position_data,
		struct optical_flow_s* flow,
		const float filtered_flow_ned_z,
		const uint8_t filtered_flow_ned_z_valid,
		struct vehicle_attitude_s* att);
void quat_flow_calculate_altitude(bool vehicle_liftoff,
		bool armed,
		float sonar_new,
		struct quat_flow_calculator_params* params,
		struct filtered_bottom_flow_s* filtered_flow,
		struct vehicle_attitude_s* att);

static void usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_spawn_cmd().
 */
int quat_flow_calculator_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");
	if (!strcmp(argv[1], "debug")){
			debug = true;
	}
	if (!strcmp(argv[1], "start") || !strcmp(argv[1], "debug"))
	{
		if (thread_running)
		{
			printf("flow position estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn_cmd("quat_flow_calculator",
					 SCHED_RR,
					 SCHED_PRIORITY_MAX - 5,
					 4096,
					 quat_flow_calculator_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop"))
	{
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status"))
	{
		if (thread_running)
			printf("\tflow position estimator is running\n");
		else
			printf("\tflow position estimator not started\n");

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

uint8_t quat_flow_calculate_flow(
		struct quat_flow_calculator_params* params,
		struct vehicle_local_position_s* local_position_data,
		struct optical_flow_s* flow,
		const float filtered_flow_ned_z,
		const uint8_t filtered_flow_ned_z_valid,
		struct vehicle_attitude_s* att)
{
	const float max_flow = params->max_velocity;	// max flow value that can be used, rad/s
	if( filtered_flow_ned_z_valid == 0 ||
			flow->quality < (uint8_t)params->minimum_quality ||
			att->R[2][2] <= 0.7f)
	{
		//invalid position or bad quality
		flow->flow_comp_x_m = 0.0f;
		flow->flow_comp_y_m = 0.0f;
		return 0;
	}
	/* distance to surface */
	float flow_dist = -filtered_flow_ned_z / att->R[2][2];
	/* check if flow is too large for accurate measurements */
	/* calculate estimated velocity in body frame */
	float body_v_est[3] = { 0.0f, 0.0f, 0.0f };

	// Calculate estimated flow velocity from currently estimated velocity
	// Transfer velocity to body frame
	for (int i = 0; i < 3; i++) {
		// world to body
		body_v_est[i] = att->R[0][i] * local_position_data->vx +
						att->R[1][i] * local_position_data->vy +
						att->R[2][i] * local_position_data->vz;
	}

	// Calculate a measure of flow accuracy based on the expecte flow velocity
	float expected_x_flow = fabsf(body_v_est[1] / flow_dist - att->rollspeed);
	float expected_y_flow = fabsf(body_v_est[0] / flow_dist + att->pitchspeed);
	float expected_max_flow = expected_x_flow > expected_y_flow ? expected_x_flow : expected_y_flow;
	uint8_t flow_accuracy = 0;

	// Scale flow accuracy between 0 and 255
	if (expected_max_flow < max_flow){
		flow_accuracy = 255;
	}
	else if (expected_max_flow >  2 * max_flow) {
		flow_accuracy = 0;
	}
	else
	{
		flow_accuracy = (uint8_t)(255.0f * (expected_max_flow - max_flow) / max_flow);
	}

	float flow_ang[3];
	flow_ang[0] = (float)flow->flow_raw_x * params->flow_k;
	flow_ang[1] = (float)flow->flow_raw_y * params->flow_k;
	flow_ang[2] = 0.0f;

	/* flow measurements vector */
	float flow_m[3];
	flow_m[0] = -flow_ang[0] * flow_dist;
	flow_m[1] = -flow_ang[1] * flow_dist;
	flow_m[2] = body_v_est[2];

	/* convert to bodyframe velocity */
	float flow_mb[3];
	for(uint8_t i = 0; i < 3; i++) {
		float sum = 0.0f;
		for(uint8_t j = 0; j < 3; j++) {
			sum = sum + flow_m[j] * rotM_flow_sensor[j][i];
		}
		flow_mb[i] = sum;
	}

	/* velocity in NED */
	float flow_v[3] = { 0.0f, 0.0f, 0.0f };

	/* project measurements vector to NED basis, skip Z component */
    utilRotateVecByMatrix2(flow_v, flow_mb, att->R);

	flow->flow_comp_x_m = flow_v[0];
	flow->flow_comp_y_m = flow_v[1];
	return flow_accuracy;
}

void quat_flow_calculate_altitude(bool vehicle_liftoff,
		bool armed,
		float sonar_new,
		struct quat_flow_calculator_params* params,
		struct filtered_bottom_flow_s* filtered_flow,
		struct vehicle_attitude_s* att)
{
	bool sonar_valid = false;
	static uint64_t time_sonar_validated = 0; // in us
	static uint64_t time_sonar_unvalidated = 0; // in us
	static float sonar_last_measurement = 0.0f;
	static uint64_t sonar_last_timestamp = 0;
	static float sonar_last = 0.0f;
	static float sonar_lp = 0.0f;
	static float sonar_speed = 0.0f;
	static int counter = 0;
	/* filtering ground distance */
	if (!vehicle_liftoff || !armed)
	{
		/* not possible to fly */
		sonar_valid = false;
		filtered_flow->ned_z = 0.0f;
		time_sonar_validated = 0;
		time_sonar_unvalidated = 0;
	}
	else if(sonar_new <= 0.3f) {
		time_sonar_validated = 0;
		if(time_sonar_unvalidated == 0) {
			time_sonar_unvalidated = filtered_flow->timestamp;
		} else if (filtered_flow->timestamp - time_sonar_unvalidated > 200000) {
			sonar_valid = false;
		}
	}
	else{
		time_sonar_unvalidated = 0;
		if(time_sonar_validated == 0) {
			time_sonar_validated = filtered_flow->timestamp;
		} else if (filtered_flow->timestamp - time_sonar_validated > 2000000) {
			sonar_valid = true;
		}
	}

	if ((!sonar_valid || att->R[2][2] <= 0.7f) &&
			!params->debug)
	{
			filtered_flow->ned_vz = 0.0f;
			filtered_flow->ned_v_z_valid = 0;
			filtered_flow->ned_z_valid = 0;
			filtered_flow->sonar_counter++;
	}
	else
	{
		/* simple lowpass sonar filtering */
		/* if new value or with sonar update frequency */
		if (sonar_new != sonar_last || counter++ % 10 == 0)
		{
			sonar_lp = 0.05f * sonar_new + 0.95f * sonar_lp;
			sonar_last = sonar_new;
		}

		float height_diff = sonar_new - sonar_lp;

		/* if over 1/2m spike follow lowpass */
		if (height_diff < -params->sonar_lower_lp_threshold ||
				height_diff > params->sonar_upper_lp_threshold)
		{
			filtered_flow->ned_z = -sonar_lp * att->R[2][2];
			filtered_flow->ground_distance = -sonar_lp;
		}
		else
		{
			filtered_flow->ned_z = -sonar_new * att->R[2][2];
			filtered_flow->ground_distance = -sonar_new;
		}
		// Velocity
		// Only calculate if last measurement was valid
		float time_since_last_sonar = ((float)(filtered_flow->timestamp - sonar_last_timestamp))/1000000.0f;
		float ned_z_lp = -sonar_lp * att->R[2][2];
		if(debug) printf("..m:%8.4f\tv:%8.4f\n", filtered_flow->ned_z, time_since_last_sonar);

		if(time_since_last_sonar > 0.09f &&
				(sonar_last_measurement != ned_z_lp || time_since_last_sonar > 0.11f)) {

			if(filtered_flow->ned_z_valid > 0) {

				float distance = ned_z_lp - sonar_last_measurement;
				sonar_speed = distance/time_since_last_sonar;
				// smooth
				filtered_flow->ned_vz += (sonar_speed - filtered_flow->ned_vz) * 1.0f;
				if(debug) printf("m:%8.4f\tl:%8.4f\td:%8.4f\tt:%8.4f\tv:%8.4f\n",
						filtered_flow->ned_z, sonar_last_measurement, distance,
						time_since_last_sonar, filtered_flow->ned_vz);
				filtered_flow->ned_v_z_valid = 255;
			} else {
				filtered_flow->ned_vz = 0.0f;
				filtered_flow->ned_v_z_valid = 0;
				if(debug) printf("v:%d\tt:%8.4f\n",filtered_flow->ned_z_valid,
						time_since_last_sonar);
			}
			filtered_flow->sonar_counter++;
			sonar_last_timestamp = filtered_flow->timestamp;
			sonar_last_measurement = ned_z_lp;
		}
		filtered_flow->ned_z_valid = 255;
	}
}

int quat_flow_calculator_thread_main(int argc, char *argv[])
{
	/* welcome user */
	thread_running = true;
	printf("[flow position estimator] starting\n");


	const float time_scale = powf(10.0f,-6.0f);
	static float flow_speed[3] = {0.0f, 0.0f, 0.0f};
	static uint64_t time_last_flow = 0; // in ms
	static float dt = 0.0f; // seconds
	static float quality = 0;

	/* subscribe to vehicle status, attitude, sensors and flow*/
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	//struct vehicle_control_mode_s control_mode;
	//memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vehicle_local_position_s local_position_data;
	memset(&local_position_data, 0, sizeof(local_position_data));


	/* subscribe to parameter changes */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));

	/* subscribe to armed topic */
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	/* subscribe to safety topic */
	//int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	/* subscribe to attitude */
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

	/* subscribe to attitude setpoint */
	int vehicle_attitude_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

	/* subscribe to optical flow*/
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));

	/* subscribe to local position*/
	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	static struct filtered_bottom_flow_s filtered_flow = {
			.sumx = 0.0f,
			.sumy = 0.0f,
			.vx = 0.0f,
			.vy = 0.0f,
			.ground_distance = 0.0f,
			.sonar_counter = 0,
			.ned_xy_valid = false,
			.ned_z_valid = false,
			.ned_v_xy_valid = false,
			.ned_v_z_valid = false,
			.landed = false,
			.ned_x = 0.0f,
			.ned_y = 0.0f,
			.ned_z = 0.0f,
			.ned_vx = 0.0f,
			.ned_vy = 0.0f,
			.ned_vz = 0.0f,
			.yaw = 0.0f
	};

	/* advert pub messages */
	orb_advert_t filtered_flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &filtered_flow);
	filtered_flow.landed = true;

	/* vehicle flying status parameters */
	bool vehicle_liftoff = false;
	bool sensors_ready = false;

	/* parameters init*/
	struct quat_flow_calculator_params params;
	struct quat_flow_calculator_param_handles param_handles;
	parameters_init(&param_handles);
	parameters_update(&param_handles, &params);

	flow_subsystem_info_pub = orb_advertise(ORB_ID(subsystem_info), &flow_control_info);

	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "quat_flow_calculator_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "quat_flow_calculator_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "quat_flow_calculator_err");

	while (!thread_should_exit)
	{

		if (sensors_ready)
		{
			/*This runs at the rate of the sensors */
			struct pollfd fds[2] = {
					{ .fd = optical_flow_sub, .events = POLLIN },
					{ .fd = parameter_update_sub,   .events = POLLIN }
			};

			/* wait for a sensor update, check for exit condition every 500 ms */
			int ret = poll(fds, 2, 500);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);

			}
			else if (ret == 0)
			{
				/* no return value, ignore */
//				printf("[flow position estimator] no bottom flow.\n");
			}
			else
			{

				/* parameter update available? */
				if (fds[1].revents & POLLIN)
				{
					/* read from param to clear updated flag */
					struct parameter_update_s update;
					orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);

					parameters_update(&param_handles, &params);
					printf("[flow position estimator] parameters updated.\n");
				}

				/* only if flow data changed */
				if (fds[0].revents & POLLIN)
				{
					perf_begin(mc_loop_perf);
					orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

					bool updated = false;
					/* got flow, updating attitude and status as well */
					orb_check(vehicle_attitude_sub, &updated);
					if(updated) {
						orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
					}
					orb_check(vehicle_attitude_setpoint_sub, &updated);
					if(updated) {
						orb_copy(ORB_ID(vehicle_attitude_setpoint), vehicle_attitude_setpoint_sub, &att_sp);
					}
					orb_check(armed_sub, &updated);
					if(updated) {
						orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					}
					/*orb_check(control_mode_sub, &updated);
					if(updated) {
						orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
					}*/
					orb_check(local_position_sub, &updated);
					if(updated) {
						orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position_data);
					}
					/* vehicle state estimation */
					float sonar_new = flow.ground_distance_m;

					/* set liftoff boolean
					 * -> at bottom sonar sometimes does not work correctly, and has to be calibrated (distance higher than 0.3m)
					 * -> accept sonar measurements after reaching calibration distance (values between 0.3m and 1.0m for some time)
					 * -> minimum sonar value 0.3m
					 */
					if (!vehicle_liftoff)
					{
						if (armed.armed &&
							att_sp.thrust > params.minimum_liftoff_thrust &&
							sonar_new > 0.3f && sonar_new < 1.0f) {
							vehicle_liftoff = true;
							filtered_flow.landed = false;
						}
					}
					else
					{
						//Thrust is not enough to signal landing
						if (!armed.armed) {
							vehicle_liftoff = false;
							filtered_flow.landed = true;
							filtered_flow.ned_x = 0.0f;
							filtered_flow.ned_y = 0.0f;
						}
					}

					//Calculate altitude
					quat_flow_calculate_altitude(vehicle_liftoff, armed.armed, sonar_new, &params, &filtered_flow, &att);

					//Calculate flow velocity
					uint8_t flow_accuracy = quat_flow_calculate_flow(&params,&local_position_data,&flow,filtered_flow.ned_z,filtered_flow.ned_z_valid,&att);

					/* calc dt between flow timestamps */
					/* ignore first flow msg */
					if(time_last_flow == 0)
					{
						time_last_flow = flow.timestamp;
						continue;
					}
					dt = (float)(flow.timestamp - time_last_flow) * time_scale;
					time_last_flow = flow.timestamp;

					/* only make position update if vehicle is lift off or DEBUG is activated*/
					if (vehicle_liftoff || params.debug)
					{
						/* copy flow */
						flow_speed[0] = flow.flow_comp_x_m;
						flow_speed[1] = flow.flow_comp_y_m;
						flow_speed[2] = 0.0f;

						/* update filtered flow */
						//filtered_flow.sumx = filtered_flow.sumx + flow_speed[0] * dt;
						//filtered_flow.sumy = filtered_flow.sumy + flow_speed[1] * dt;
						//filtered_flow.vx = flow_speed[0];
						//filtered_flow.vy = flow_speed[1];
						if(flow_accuracy > 200) {
							filtered_flow.ned_x = filtered_flow.ned_x + flow_speed[0] * dt;
							filtered_flow.ned_y = filtered_flow.ned_y + flow_speed[1] * dt;
						}
						filtered_flow.ned_vx = flow_speed[0];
						filtered_flow.ned_vy = flow_speed[1];
						filtered_flow.ned_xy_valid = flow_accuracy;
						filtered_flow.ned_v_xy_valid = flow_accuracy;
					}
					else
					{
						/* set speed to zero and let position as it is */
						filtered_flow.vx = 0;
						filtered_flow.vy = 0;
						filtered_flow.ned_vx = 0;
						filtered_flow.ned_vy = 0;
						filtered_flow.ned_xy_valid = 0;
						filtered_flow.ned_v_xy_valid = 0;
					}

					// publish subsystem info for flow
					if(!flow_valid &&
						quality >= (float)params.minimum_quality) {
						flow_valid  = true;
						flow_control_info.enabled = true;
						orb_publish(ORB_ID(subsystem_info), flow_subsystem_info_pub, &flow_control_info);
					} else if (flow_valid && quality < (float)params.minimum_quality) {
						flow_valid = false;
						flow_control_info.enabled = false;
						orb_publish(ORB_ID(subsystem_info), flow_subsystem_info_pub, &flow_control_info);
					}

					/* always publish local position */
					filtered_flow.timestamp = hrt_absolute_time();
					filtered_flow.ned_x = !isfinite(filtered_flow.ned_x) ? 0.0f : filtered_flow.ned_x;
					filtered_flow.ned_y = !isfinite(filtered_flow.ned_y) ? 0.0f : filtered_flow.ned_y;
					filtered_flow.ned_z = !isfinite(filtered_flow.ned_z) ? 0.0f : filtered_flow.ned_z;
					filtered_flow.ned_vx = !isfinite(filtered_flow.ned_vx) ? 0.0f : filtered_flow.ned_vx;
					filtered_flow.ned_vy = !isfinite(filtered_flow.ned_vy) ? 0.0f : filtered_flow.ned_vy;
					filtered_flow.ned_vz = !isfinite(filtered_flow.ned_vz) ? 0.0f : filtered_flow.ned_vz;

					orb_publish(ORB_ID(filtered_bottom_flow), filtered_flow_pub, &filtered_flow);

					/* measure in what intervals the position estimator runs */
					perf_count(mc_interval_perf);
					perf_end(mc_loop_perf);
				}
			}

		}
		else
		{
			/* sensors not ready waiting for first attitude msg */

			/* polling */
			struct pollfd fds[1] = {
				{ .fd = vehicle_attitude_sub, .events = POLLIN },
			};

			/* wait for a attitude message, check for exit condition every 5 s */
			int ret = poll(fds, 1, 5000);

			if (ret < 0)
			{
				/* poll error, count it in perf */
				perf_count(mc_err_perf);
			}
			else if (ret == 0)
			{
				/* no return value, ignore */
				printf("[flow position estimator] no attitude received.\n");
			}
			else
			{
				if (fds[0].revents & POLLIN){
					sensors_ready = true;
					printf("[flow position estimator] initialized.\n");
				}
			}
		}
	}

	printf("[flow position estimator] exiting.\n");
	thread_running = false;

	close(vehicle_attitude_setpoint_sub);
	close(vehicle_attitude_sub);
	close(armed_sub);
	//close(control_mode_sub);
	close(parameter_update_sub);
	close(optical_flow_sub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	return 0;
}


