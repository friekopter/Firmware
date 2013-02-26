
/**
 * @file quat_pos_control.c
 *
 * Quat position controller
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
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <systemlib/systemlib.h>

#include "quat_pos_control_params.h"


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

__EXPORT int quat_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int quat_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int quat_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("quat pos control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn("quat pos control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 quat_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tquat pos control app is running\n");
		} else {
			printf("\tquat pos control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
quat_pos_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[quat pos control] Control started, taking over position control\n");

	/* structures */
	struct vehicle_status_s state;
	struct vehicle_attitude_s att;
	//struct vehicle_global_position_setpoint_s global_pos_sp;
	struct vehicle_local_position_setpoint_s local_pos_sp;
	struct vehicle_vicon_position_s local_pos;
	struct manual_control_setpoint_s manual;
	struct vehicle_attitude_setpoint_s att_sp;

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int local_pos_sub = orb_subscribe(ORB_ID(vehicle_vicon_position));
	//int global_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));

	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	thread_running = true;

	int loopcounter = 0;

	struct quat_position_control_params p;
	struct quat_position_control_param_handles h;
	parameters_init(&h);
	parameters_update(&h, &p);


	while (1) {
		/* get a local copy of the vehicle state */
		orb_copy(ORB_ID(vehicle_status), state_sub, &state);
		/* get a local copy of manual setpoint */
		orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
		/* get a local copy of attitude */
		orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
		/* get a local copy of local position */
		orb_copy(ORB_ID(vehicle_vicon_position), local_pos_sub, &local_pos);
		/* get a local copy of local position setpoint */
		orb_copy(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_sub, &local_pos_sp);

		if (loopcounter == 500) {
			parameters_update(&h, &p);
			loopcounter = 0;
		}

		// if (state.state_machine == SYSTEM_STATE_AUTO) {
			
			// XXX IMPLEMENT POSITION CONTROL HERE

			float dT = 1.0f / 50.0f;

			float x_setpoint = 0.0f;

			// XXX enable switching between Vicon and local position estimate
			/* local pos is the Vicon position */

			// XXX just an example, lacks rotation around world-body transformation
			att_sp.pitch_body = (local_pos.x - x_setpoint) * p.p;
			att_sp.roll_body = 0.0f;
			att_sp.yaw_body = 0.0f;
			att_sp.thrust = 0.3f;
			att_sp.timestamp = hrt_absolute_time();

			/* publish new attitude setpoint */
			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
		// } else if (state.state_machine == SYSTEM_STATE_STABILIZED) {
			/* set setpoint to current position */
			// XXX select pos reset channel on remote
			/* reset setpoint to current position  (position hold) */
			// if (1 == 2) {
			// 	local_pos_sp.x = local_pos.x;
			// 	local_pos_sp.y = local_pos.y;
			// 	local_pos_sp.z = local_pos.z;
			// 	local_pos_sp.yaw = att.yaw;
			// }
		// }

		/* run at approximately 50 Hz */
		usleep(20000);
		loopcounter++;

	}

	printf("[quat pos control] ending now...\n");

	thread_running = false;

	fflush(stdout);
	return 0;
}

