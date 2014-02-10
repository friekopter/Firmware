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
#include <uORB/topics/vehicle_control_mode.h>
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

int
quat_att_control_thread_main(int argc, char *argv[]);

int
quat_att_control_thread_main(int argc, char *argv[])
{
	// print text
	printf("Quat Attitude Control initialized..\n\n");
	fflush(stdout);

	int printcounter = 0;
	/* declare and safely initialize all structs */
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));

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
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int params_sub = orb_subscribe(ORB_ID(parameter_update));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));

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

		/* wait for an attitude update, check for exit condition every 500 ms */
		poll(fds, 1, 500);
		//TODO FL might be a good idea to poll for gyro because that is faster
		perf_begin(mc_loop_perf);

		/* get a local copy of system state */
		bool updated;
		orb_check(control_mode_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
		}
		/* get a local copy of status */
		orb_check(status_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(vehicle_status), status_sub, &status);
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
		/* get a local copy of the current sensor values */
		orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

		att.rollspeed = raw.gyro_rad_s[0];
		att.pitchspeed = raw.gyro_rad_s[1];
		att.yawspeed = raw.gyro_rad_s[2];

		/** STEP 1: Define which input is the dominating control input */
		if (!control_mode.flag_armed) {
		    //navSetHoldHeading(att.yaw);
		    // Reset all PIDs
		    control_quadrotor_attitude_reset();
		    //Set yaw setpoint to current yaw
		    control_quadrotor_set_yaw(att.yaw);
		}
		else if ( control_mode.flag_control_manual_enabled &&
				 !control_mode.flag_control_velocity_enabled) {
			/* manual inputs, from RC control or joystick */
			if (control_mode.flag_control_attitude_enabled) {
				// Always control attitude no rates
				att_sp.roll_body = manual.roll * control.controlRollF;
				att_sp.pitch_body = manual.pitch * control.controlPitchF;
				att_sp.yaw_body = control_quadrotor_get_yaw();
				att_sp.timestamp = hrt_absolute_time();
				/* set yaw rate */
				if (manual.yaw < -control.controlDeadBand || manual.yaw > control.controlDeadBand)
				{
					rates_sp.yaw = manual.yaw * control.controlYawF;
				}
				else
				{
					rates_sp.yaw = 0.0f;
				}
				if ( !control_mode.flag_control_altitude_enabled ) {
					// enable manual altitude control
					att_sp.thrust = manual.throttle * control.controlThrottleF;
				}
				else {
					// altitude is controlled by software (hopefully!)
				}
				if (motor_test_mode) {
					att_sp.roll_body = 0.0f;
					att_sp.pitch_body = 0.0f;
					att_sp.yaw_body = 0.0f;
					att_sp.thrust = 0.1f;
					att_sp.timestamp = hrt_absolute_time();
				}
			} else {
				/* manual rate inputs (ACRO), from RC control or joystick */
				if (control_mode.flag_control_rates_enabled) {
					rates_sp.roll = manual.roll;
					rates_sp.pitch = manual.pitch;
					rates_sp.yaw = manual.yaw;
					rates_sp.timestamp = hrt_absolute_time();
				}
			}
		}
		else if ( control_mode.flag_control_manual_enabled &&
				  control_mode.flag_control_velocity_enabled ) {
			/* set yaw rate */
			if (manual.yaw < -control.controlDeadBand || manual.yaw > control.controlDeadBand)
			{
				rates_sp.yaw = manual.yaw * control.controlYawF;
			}
			else
			{
				rates_sp.yaw = 0.0f;
			}
			att_sp.yaw_body = control_quadrotor_get_yaw();
			att_sp.timestamp = hrt_absolute_time();
		} else {
			rates_sp.yaw = 0.0f;
		}

		/** STEP 3: Identify the controller setup to run and set up the inputs correctly */

		control_quadrotor_attitude(
				&att_sp,
				&att,
				&rates_sp,
				&control,
				&actuators);

		if (control_mode.flag_control_manual_enabled &&
			!control_mode.flag_control_velocity_enabled &&
			!control_mode.flag_control_altitude_enabled) {
			// only in this case we define the setpoints and are entitled to
			// publish its values
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
		}

	    if(isnan(actuators.control[0]) || isnan(actuators.control[1]) || isnan(actuators.control[2]) || isnan(actuators.control[3])) {
	    	/* init control */
	    	actuators.control[0] = 0;
	    	actuators.control[1] = 0;
	    	actuators.control[2] = 0;
	    	actuators.control[3] = 0;
	    	control_quadrotor_attitude_reset();
	    	control_initFilter();
	    	continue;
	    }
		// /* print debug information every 200th time */
		if (debug == true && printcounter % 1000 == 0)
		{
			printf("attitude_sp: %8.4f\t%8.4f\t%8.4f\t%8.4f\n", (double)att_sp.roll_body, (double)att_sp.pitch_body, (double)att_sp.yaw_body, (double)att_sp.thrust);
			printf("attitude   : %8.4f\t%8.4f\t%8.4f\n",        (double)att.roll, (double)att.pitch, (double)att.yaw);
			printf("actuators  : %8.4f\t%8.4f\t%8.4f\t%8.4f\n", (double)actuators.control[0], (double)actuators.control[1], (double)actuators.control[2], (double)actuators.control[3]);
			printf("state: %i\n", control_mode.flag_control_manual_enabled);
		}
		printcounter++;

		/* STEP 2: publish the result to the vehicle actuators */
		orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

		perf_end(mc_loop_perf);
	}

	printf("[quat att control] stopping, disarming motors.\n");

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;
	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);


	close(att_sub);
	close(status_sub);
	close(control_mode_sub);
	close(manual_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

/**
* Print the correct usage.
*/
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: quat_att_control {debug|start|stop}\n");
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

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "debug"))
	{
		debug = true;
	}
	if (!strcmp(argv[1+optioncount], "start") || !strcmp(argv[1], "debug")) {

		thread_should_exit = false;
		mc_task = task_spawn_cmd("quat_att_control",
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
