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
 * @file attitude_estimator_quat_main.c
 * 
 * Extended Kalman Filter for Attitude Estimation.
 */

#include "v1.0/common/mavlink.h"
#include <nuttx/config.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/systemlib.h>

#include "attitude_estimator_quat_params.h"
#include "quat.h"


__EXPORT int attitude_estimator_quat_main(int argc, char *argv[]);

static unsigned int loop_interval_alarm = 6500;	// loop interval in microseconds

static float dt = 1.0f;
/* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
static float z_k[9];					/**< Measurement vector */


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int attitude_estimator_quat_task;				/**< Handle of deamon task / thread */
static bool debug = false;
static int mavlink_fd;
static struct vehicle_status_s state;

/**
 * Mainloop of attitude_estimator_quat.
 */
int attitude_estimator_quat_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: attitude_estimator_quat {start|stop|status|debug} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The attitude_estimator_quat app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int attitude_estimator_quat_main(int argc, char *argv[])
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
			printf("attitude_estimator_quat already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		attitude_estimator_quat_task = task_spawn_cmd("attitude_estimator_quat",
							 SCHED_DEFAULT,
							 SCHED_PRIORITY_MAX - 5,
							 20000,
							 attitude_estimator_quat_thread_main,
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
		if (thread_running) {
			printf("\tattitude_estimator_quat app is running\n");
		} else {
			printf("\tattitude_estimator_quat app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

/*
 * QUAT Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
int attitude_estimator_quat_thread_main(int argc, char *argv[])
{
	// print text
	printf("Quat Attitude Estimator initialized..\n\n");
	fflush(stdout);

	int overloadcounter = 19;

	/* Initialize filter */

	/* store start time to guard against too slow update rates */
	uint64_t last_run = hrt_absolute_time();

	struct sensor_combined_s raw;
	struct vehicle_attitude_s att;

	uint64_t last_measurement = 0;

	// Init state struct
	memset(&state, 0, sizeof(state));

	/* advertise attitude */
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);

	usleep(25000);

	int loopcounter = 0;
	int printcounter = 0;

	thread_running = true;

	/* keep track of sensor updates */
	uint64_t sensor_last_timestamp[3] = {0, 0, 0};
	float sensor_update_hz[3] = {0.0f, 0.0f, 0.0f};

	// Track parameter changes
	struct attitude_estimator_quat_params quat_params;
	struct attitude_estimator_quat_param_handles quat_param_handles;

	// Subscribe to data updates
	// Parameter
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* rate-limit parameter updates to 1Hz */
	orb_set_interval(sub_params, 1000);

	// System state
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	/* rate-limit raw data updates to 2Hz */
	orb_set_interval(state_sub, 500);

	// Raw data
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(sub_raw, 4);

	// initialize parameter handles
	int param_init_result = parameters_init(&quat_param_handles);
	if(param_init_result)
	{
		printf("[attitude estimator quat] WARNING: Parameter initialization error?\n");
	}
	int param_update_result = parameters_update(&quat_param_handles, &quat_params);
	if(param_update_result)
	{
		printf("[attitude estimator quat] WARNING: Parameter update error?\n");
	}
	printf("[attitude estimator quat] parameter updated.\n");
	// Call init methods
	quatInit();
	struct pollfd fds[3] = {
		{ .fd = sub_raw,   .events = POLLIN },
		{ .fd = sub_params, .events = POLLIN },
		{ .fd = state_sub, .events = POLLIN }
	};
	/* Main loop*/
	while (!thread_should_exit){
		if(mavlink_fd <= 0){
			mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);
			mavlink_log_info(mavlink_fd,"[attitude estimator quat] estimator starting.\n");
		}

		int ret = poll(fds, 3, 1000);

		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[attitude estimator quat] WARNING: Not getting sensor data - sensor app running?\n");
		}
		else
		{
			/* only update parameters if state changed */
			if (fds[2].revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);
			}
			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN)
			{
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				param_update_result = parameters_update(&quat_param_handles, &quat_params);
				if(param_update_result)
				{
					//printf("[attitude estimator quat] WARNING: Parameter update error!\n");
					mavlink_log_critical(mavlink_fd,"[attitude estimator quat] WARNING: Parameter update error!\n");
				}
				else
				{
					//printf("[attitude estimator quat] parameter updated.\n");
					mavlink_log_info(mavlink_fd,"[attitude estimator quat] parameter updated.\n");
				}
			}
			/* only run filter if sensor values changed */
			if (fds[0].revents & POLLIN)
			{
				/* get latest measurements */
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

				/* Calculate data time difference in seconds */
				dt = ((float)(raw.timestamp - last_measurement)) / 1e6f;
				last_measurement = raw.timestamp;
				uint8_t update_vect[3] = {0, 0, 0};

				/* Fill in gyro measurements */
				if (sensor_last_timestamp[0] != raw.gyro_timestamp) {
					update_vect[0] = 1;
					sensor_last_timestamp[0] = raw.gyro_timestamp;
					uint64_t sensor_dt = raw.timestamp - sensor_last_timestamp[0];
					if(sensor_dt == 0) sensor_update_hz[0] = 0;
					else sensor_update_hz[0] = 1e6f / sensor_dt;
				}
				z_k[0] =  raw.gyro_rad_s[0];
				z_k[1] =  raw.gyro_rad_s[1];
				z_k[2] =  raw.gyro_rad_s[2];

				/* update accelerometer measurements */
				if (sensor_last_timestamp[1] != raw.accelerometer_timestamp) {
					update_vect[1] = 1;
					sensor_last_timestamp[1] = raw.accelerometer_timestamp;
					uint64_t sensor_dt = raw.timestamp - sensor_last_timestamp[1];
					if(sensor_dt == 0) sensor_update_hz[1] = 0;
					else sensor_update_hz[1] = 1e6f / sensor_dt;
				}
				z_k[3] = raw.accelerometer_m_s2[0];
				z_k[4] = raw.accelerometer_m_s2[1];
				z_k[5] = raw.accelerometer_m_s2[2];

				/* update magnetometer measurements */
				if (sensor_last_timestamp[2] != raw.magnetometer_timestamp) {
					update_vect[2] = 1;
					sensor_last_timestamp[2] = raw.magnetometer_timestamp;
					uint64_t sensor_dt = raw.timestamp - sensor_last_timestamp[2];
					if(sensor_dt == 0) sensor_update_hz[2] = 0;
					else sensor_update_hz[2] = 1e6f / sensor_dt;
				}
				z_k[6] = raw.magnetometer_ga[0];
				z_k[7] = raw.magnetometer_ga[1];
				z_k[8] = raw.magnetometer_ga[2];

				uint64_t now = hrt_absolute_time();
				unsigned int time_elapsed = now - last_run;
				last_run = now;

				if (time_elapsed > loop_interval_alarm)
				{
					//TODO: add warning, cpu overload here
					// if (overloadcounter == 20) {
					// 	printf("CPU OVERLOAD DETECTED IN ATTITUDE ESTIMATOR quat (%lu > %lu)\n", time_elapsed, loop_interval_alarm);
					// 	overloadcounter = 0;
					// }
					overloadcounter++;
				}

				uint64_t timing_start = hrt_absolute_time();
				bool isFlying = (!state.condition_landed);
				quatUpdate(update_vect, dt, isFlying, z_k, &quat_params, &att );
				uint64_t timing_diff = hrt_absolute_time() - timing_start;

				// /* print debug information every 500th time */
				if (debug == true && printcounter % 500 == 0)
				{
					printf("sensor inputs: g: %8.4f\t%8.4f\t%8.4f\ta: %8.4f\t%8.4f\t%8.4f\t m: %8.4f\t%8.4f\t%8.4f\n", (double)z_k[0], (double)z_k[1], (double)z_k[2], (double)z_k[3], (double)z_k[4], (double)z_k[5], (double)z_k[6], (double)z_k[7], (double)z_k[8]);
					printf("quat params: accdist: %8.4f\tka: %8.4f\t ki:%8.4f\tkm1: %8.4f\tkm2: %8.4f\t kp: %8.4f\n",(double)quat_params.accdist, (double)quat_params.ka, (double)quat_params.ki, (double)quat_params.km1, (double)quat_params.km2, (double)quat_params.kp);
					printf("quat attitude iteration: %d, runtime: %d us, dt: %d us (%d Hz)\n", loopcounter, (int)timing_diff, (int)(dt * 1000000.0f), (int)(1.0f / dt));
					printf("roll: %8.4f\tpitch: %8.4f\tyaw:%8.4f\n", (double)att.roll, (double)att.pitch, (double)att.yaw);
					printf("update rates gyro: %8.4f\taccel: %8.4f\tmag:%8.4f\n", (double)sensor_update_hz[0], (double)sensor_update_hz[1], (double)sensor_update_hz[2]);
				}
				printcounter++;

				// Broadcast
				orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
			}
		}

		loopcounter++;
	}

	thread_running = false;
	close(state_sub);
	close(sub_params);
	close(sub_raw);

	return 0;
}
