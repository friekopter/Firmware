/****************************************************************************
 *
 *   Copyright (C) 2012 All rights reserved.
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

/**
 * @file quat_att_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Friedemann Ludwig
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <sys/prctl.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

#include "quat_att_control.h"
#include "quat_att_control_params.h"

__EXPORT int quat_att_control_main(int argc, char *argv[]);

static bool debug = false;
static bool thread_should_exit;
static int mc_task;
static bool motor_test_mode = false;

static orb_advert_t actuator_pub;

static struct vehicle_status_s state;

static int
quat_att_control_thread_main(int argc, char *argv[])
{
	// print text
	printf("Quat Attitude Control initialized..\n\n");
	fflush(stdout);

	int printcounter = 0;
	/* declare and safely initialize all structs */
	memset(&state, 0, sizeof(state));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));

	struct actuator_controls_s actuators;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/*
	 * Do not rate-limit the loop to prevent aliasing
	 * if rate-limiting would be desired later, the line below would
	 * enable it.
	 *
	 * rate-limit the attitude subscription to 200Hz to pace our loop
	 * orb_set_interval(att_sub, 5);
	 */
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int params_sub = orb_subscribe(ORB_ID(parameter_update));
/*
	// Track parameter changes
	struct attitude_control_quat_params speed_params;
	struct attitude_control_quat_params distance_params;
	struct attitude_control_quat_params altSpeed_params;
	struct attitude_control_quat_params altDistance_params;
	struct attitude_pid_quat_param_handles speed_param_handles;
	struct attitude_pid_quat_param_handles distance_param_handles;
	struct attitude_pid_quat_param_handles altSpeed_param_handles;
	struct attitude_pid_quat_param_handles altDistance_param_handles;
	// initialize parameter handles
	int param_init_result = parameters_init(&speed_param_handles,
											&distance_param_handles,
											&altSpeed_param_handles,
											&altDistance_param_handles);
	if (param_init_result){
		printf("[attitude control quat] WARNING: Parameter initialization error?\n");
	}
	int param_update_result = parameters_update(&speed_param_handles, &speed_params,
												&distance_param_handles, &distance_params,
												&altSpeed_param_handles, &altSpeed_params,
												&altDistance_param_handles, &altDistance_params);
	if (param_update_result){
		printf("[attitude control quat] WARNING: Parameter update error?\n");
	}
*/
	// Track parameter changes
	struct attitude_pid_quat_params tilt_rate_params;
	struct attitude_pid_quat_params tilt_angle_params;
	struct attitude_pid_quat_params yaw_rate_params;
	struct attitude_pid_quat_params yaw_angle_params;
	struct attitude_control_quat_params control;

	struct attitude_pid_quat_param_handles tilt_rate_param_handles;
	struct attitude_pid_quat_param_handles tilt_angle_param_handles;
	struct attitude_pid_quat_param_handles yaw_rate_param_handles;
	struct attitude_pid_quat_param_handles yaw_angle_param_handles;
	struct attitude_control_quat_param_handles control_handles;

	// initialize parameter handles
	int param_init_result = parameters_init(&tilt_rate_param_handles,
											&tilt_angle_param_handles,
											&yaw_rate_param_handles,
											&yaw_angle_param_handles,
											&control_handles);
	if (param_init_result){
		printf("[attitude control quat] WARNING: Parameter initialization error?\n");
	}
	int param_update_result = parameters_update(&tilt_rate_param_handles,
												&tilt_angle_param_handles,
												&yaw_rate_param_handles,
												&yaw_angle_param_handles,
												&control_handles,
												&tilt_rate_params,
												&tilt_angle_params,
												&yaw_rate_params,
												&yaw_angle_params,
												&control);
	if (param_update_result){
		printf("[attitude control quat] WARNING: Parameter update error?\n");
	}
	struct pollfd fds[1] = {{ .fd = att_sub, .events = POLLIN }};
	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* init control */
	control_quadrotor_attitude_init(&tilt_rate_params,
									&tilt_angle_params,
									&yaw_rate_params,
									&yaw_angle_params,
									&control);
	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "quat_att_control");

	/* welcome user */
	printf("[quat_att_control] starting\n");

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		poll(fds, 1, 500);

		perf_begin(mc_loop_perf);

		/* get a local copy of system state */
		bool updated;
		orb_check(state_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		}
		/* check if params have changed */
		orb_check(params_sub, &updated);
		if (updated) {

			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), params_sub, &update);
			/* update parameters */
			param_update_result = parameters_update(&tilt_rate_param_handles,
													&tilt_angle_param_handles,
													&yaw_rate_param_handles,
													&yaw_angle_param_handles,
													&control_handles,
													&tilt_rate_params,
													&tilt_angle_params,
													&yaw_rate_params,
													&yaw_angle_params,
													&control);
		}
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of attitude setpoint */
		orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);
		/* get a local copy of rates setpoint */
		orb_check(setpoint_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(offboard_control_setpoint), setpoint_sub, &offboard_sp);
		}
		/* get a local copy of the current sensor values */
		orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);


		/** STEP 1: Define which input is the dominating control input */
		if (state.flag_control_offboard_enabled) {
					/* offboard inputs */
					if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_RATES) {
						rates_sp.roll = offboard_sp.p1;
						rates_sp.pitch = offboard_sp.p2;
						rates_sp.yaw = offboard_sp.p3;
						rates_sp.thrust = offboard_sp.p4;
						// printf("thrust_rate=%8.4f\n",offboard_sp.p4);
						rates_sp.timestamp = hrt_absolute_time();
						orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
					} else if (offboard_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE) {
						att_sp.roll_body = offboard_sp.p1;
						att_sp.pitch_body = offboard_sp.p2;
						att_sp.yaw_body = offboard_sp.p3;
						att_sp.thrust = offboard_sp.p4;
						// printf("thrust_att=%8.4f\n",offboard_sp.p4);
						att_sp.timestamp = hrt_absolute_time();
						/* STEP 2: publish the result to the vehicle actuators */
						orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
					}

					/* decide whether we want rate or position input */
		}
		else if (state.flag_control_manual_enabled) {
			/* manual inputs, from RC control or joystick */
			// Always control attitude no rates
				att_sp.roll_body = manual.roll * control.controlRollF;
				att_sp.pitch_body = manual.pitch * control.controlPitchF;
				/* set yaw rate */
				if (manual.yaw < -control.controlDeadBand || manual.yaw > control.controlDeadBand)
				{
					rates_sp.yaw = manual.yaw * control.controlYawF;
				}
				else
				{
					rates_sp.yaw = 0.0f;
				}
				att_sp.thrust = manual.throttle * control.controlThrottleF;
				att_sp.timestamp = hrt_absolute_time();

			/* STEP 2: publish the result to the vehicle actuators */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			if (motor_test_mode) {
				att_sp.roll_body = 0.0f;
				att_sp.pitch_body = 0.0f;
				att_sp.yaw_body = 0.0f;
				att_sp.thrust = 0.1f;
				att_sp.timestamp = hrt_absolute_time();
				/* STEP 2: publish the result to the vehicle actuators */
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			}
		}

		/** STEP 3: Identify the controller setup to run and set up the inputs correctly */

		control_quadrotor_attitude(
				&att_sp,
				&att,
				&rates_sp,
				&control,
				&actuators);

		// /* print debug information every 200th time */
		if (debug == true && printcounter % 2000 == 0)
		{
			printf("attitude_sp: %8.4f\t%8.4f\t%8.4f\t%8.4f\n", (double)att_sp.roll_body, (double)att_sp.pitch_body, (double)att_sp.yaw_body, (double)att_sp.thrust);
			printf("actuators: %8.4f\t%8.4f\t%8.4f\t%8.4f\n", (double)actuators.control[0], (double)actuators.control[1], (double)actuators.control[2], (double)actuators.control[3]);
			printf("state: %i\n", state.flag_control_manual_enabled);
		}
		printcounter++;

		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		perf_end(mc_loop_perf);
	}

	printf("[quat att control] stopping, disarming motors.\n");

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);


	close(att_sub);
	close(state_sub);
	close(manual_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: quat_att_control [-t] {start|status|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	fprintf(stderr, "    -t enables motor test mode with 10%% thrust\n");
	exit(1);
}

int quat_att_control_main(int argc, char *argv[])
{
	int	ch;
	unsigned int optioncount = 0;

	while ((ch = getopt(argc, argv, "tm:")) != EOF) {
		switch (ch) {
		case 't':
			motor_test_mode = true;
			optioncount += 1;
			break;
		case ':':
			usage("missing parameter");
			break;
		default:
			fprintf(stderr, "option: -%c\n", ch);
			usage("unrecognized option");
			break;
		}
	}
	argc -= optioncount;
	//argv += optioncount;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "debug"))
	{
		debug = true;
	}
	if (!strcmp(argv[1+optioncount], "start") || !strcmp(argv[1], "debug")) {

		thread_should_exit = false;
		mc_task = task_spawn("quat_att_control",
				     SCHED_DEFAULT,
				     SCHED_PRIORITY_MAX - 15,
				     6000,
				     quat_att_control_thread_main,
				     NULL);
		exit(0);
	}

	if (!strcmp(argv[1+optioncount], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}
