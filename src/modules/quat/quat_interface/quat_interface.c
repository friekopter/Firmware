
/**
 * @file quat_interface.c
 * Implementation of quat motor control interface.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <poll.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <systemlib/err.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <systemlib/systemlib.h>

#include "quat_motor_control.h"

__EXPORT int quat_interface_main(int argc, char *argv[]);


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int quat_interface_task;		/**< Handle of deamon task / thread */

static const char *commandline_usage = "\tusage: quat_interface start|status|stop [-t for motor test (10%% thrust)] [-s for simulation mode]\n";

/**
 * Mainloop of quat_interface.
 */
static int quat_interface_thread_main(int argc, char *argv[])
{
	thread_running = true;
	const uint16_t motors_test_pwm = 80;
	/* welcome user */
	printf("[quat_interface] Control started, taking over motors\n");

	bool motor_test_mode = false;
	int test_motor = -1;
	bool simulator_mode = false;

	/* read commandline arguments */
	for (int i = 0; i < argc && argv[i]; i++) {
		if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--simulate") == 0) {
			simulator_mode = true;
			motor_test_mode = false;
			printf("[quat_interface] Starting simulation mode\n");
			break;
		}

		if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--test") == 0) {
			motor_test_mode = true;
			printf("[quat_interface] Starting motor test mode\n");
		}

		if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--motor") == 0) {
			if (i+1 < argc) {
				int motor = atoi(argv[i+1]);
				if (motor > 0 && motor < 5) {
					test_motor = motor;
					printf("[quat_interface] Testing motor %d\n",motor);
				} else {
					thread_running = false;
					errx(1, "supply a motor # between 1 and 4. Example: -m 1\n %s", commandline_usage);
				}
			} else {
				thread_running = false;
				errx(1, "missing parameter to -m 1..4\n %s", commandline_usage);
			}
		}
	}

	if (motor_test_mode) {
		printf("[quat_interface] Motor test mode enabled, setting 10 %% thrust.\n");
	}
	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));
	struct actuator_armed_s armed;
	armed.armed = false;

	/* subscribe to attitude, motor setpoints and system state */
	int actuator_controls_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	/* rate-limit raw data updates to 143Hz */
	orb_set_interval(actuator_controls_sub, 7);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	/* rate-limit raw data updates to 5Hz */
	orb_set_interval(armed_sub, 200);
	struct pollfd fds[2] = {
		{ .fd = actuator_controls_sub,   .events = POLLIN },
		{ .fd = armed_sub,   .events = POLLIN }
	};

	printf("[quat_interface] Motors initialized - ready.\n");
	fflush(stdout);

	quat_write_motor_commands(simulator_mode, 0, 0, 0, 0);
	while (!thread_should_exit) {
		if (motor_test_mode) {
			/* set motors to idle speed */
			if (test_motor > 0 && test_motor < 5) {
				int motors[4] = {0, 0, 0, 0};
				motors[test_motor - 1] = motors_test_pwm;
				quat_write_motor_commands(simulator_mode, motors[0], motors[1], motors[2], motors[3]);
			} else {
				quat_write_motor_commands(simulator_mode, motors_test_pwm, motors_test_pwm, motors_test_pwm, motors_test_pwm);
			}

		} else {
			/* MAIN OPERATION MODE */
			/* get a local copy of the actuator controls */

			int ret = poll(fds, 2, 5000);
			if (ret < 0)
			{
				/* XXX this is seriously bad - should be an emergency */
			}
			else if (ret == 0)
			{
				/* XXX this means no sensor data - should be critical or emergency */
				printf("[quat_interface]: Could not read attitude control\n");
			}
			else
			{
				/* only update parameters if state changed */
				if (fds[1].revents & POLLIN)
				{
					orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
					if(!armed.armed || armed.lockdown) {
						/* Silently lock down motor speeds to zero */
						quat_write_motor_commands(simulator_mode, 0, 0, 0, 0);
					}
				}
				/* only update parameters if they changed */
				if (fds[0].revents & POLLIN)
				{
					orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_controls_sub, &actuator_controls);
					if (armed.armed && !armed.lockdown) {
						quat_mixing_and_output(simulator_mode, &actuator_controls);
					} else {
						/* Silently lock down motor speeds to zero */
						quat_write_motor_commands(simulator_mode, 0, 0, 0, 0);
					}
				}
			}
		}
	}
	fflush(stdout);

	thread_running = false;

	return OK;
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
	fprintf(stderr, "%s\n", commandline_usage);
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int quat_interface_main(int argc, char *argv[])
{
		if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("quat_interface already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		quat_interface_task = task_spawn_cmd("quat_interface",
						    SCHED_DEFAULT,
						    SCHED_PRIORITY_MAX - 15,
						    2048,
						    quat_interface_thread_main,
						    (argv) ? (const char **)&argv[1] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tquat_interface is running\n");
		} else {
			printf("\tquat_interface not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

