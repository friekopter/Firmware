
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
#include <poll.h>
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include "quat_pos_control_params.h"
#include "nav.h"
#include "nav_ukf.h"

#include "quat_pos_control.h"


runStruct_t runData __attribute__((section(".ccm")));
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static bool debug = false;

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

	if (!strcmp(argv[1], "debug")){
		debug = true;
	}
	if (!strcmp(argv[1], "start") || !strcmp(argv[1], "debug")) {

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
	thread_running = true;
	int loopcounter = 0;
	int printcounter = 0;

	// Output
	// Calculation result is the attitude setpoint
	struct vehicle_attitude_setpoint_s att_sp;
	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

	// For control reasons we also publish the suspected position
	struct vehicle_global_position_s global_position;
	orb_advert_t global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_position);

	// Inputs
	// manual control
	struct manual_control_setpoint_s manual;
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	// System state
	struct vehicle_status_s state;
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));

	// Parameter
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* rate-limit parameter updates to 1Hz */
	orb_set_interval(sub_params, 1000);

	// Raw data
	struct sensor_combined_s raw;
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(sub_raw, 4);

	// GPS Position
	struct vehicle_gps_position_s gps_data;
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));

	//Init parameters
	struct quat_position_control_UKF_params ukf_params;
	struct quat_position_control_UKF_param_handles ukf_handles;
	struct quat_position_control_NAV_params nav_params;
	struct quat_position_control_NAV_param_handles nav_handles;
	parameters_init(&nav_handles, &ukf_handles);
	parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);

	sleep(5);
	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "quat_pos_control");
	orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);

	navUkfInit(&ukf_params,&raw);
	navInit(&nav_params,raw.baro_alt_meter,0.0f);//TODO FL find a better yaw to init hold

	struct pollfd fds[5] = {
		{ .fd = sub_raw,   .events = POLLIN },
		{ .fd = gps_sub,   .events = POLLIN },
		{ .fd = manual_sub,   .events = POLLIN },
		{ .fd = sub_params, .events = POLLIN },
		{ .fd = state_sub, .events = POLLIN }
	};
	while (1) {
		static uint64_t timestamp_position = 0;
		static uint64_t timestamp_velocity = 0;

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
			float dt = 0;
			perf_begin(mc_loop_perf);
			/* only update parameters if state changed */
			if (fds[4].revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);
			}
			/* only update parameters if they changed */
			if (fds[3].revents & POLLIN)
			{
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);
			}
			if (fds[2].revents & POLLIN)
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			}
			if (fds[0].revents & POLLIN)
			{
				// raw parameter changed
				dt = navUkfInertialUpdate(&raw);

				// record history for acc & mag & pressure readings for smoothing purposes
				// acc
				static uint64_t acc_counter = 0;
				if(raw.accelerometer_counter > acc_counter)
				{
					acc_counter = raw.accelerometer_counter;
					runData.sumAcc[0] -= runData.accHist[0][runData.accHistIndex];
					runData.sumAcc[1] -= runData.accHist[1][runData.accHistIndex];
					runData.sumAcc[2] -= runData.accHist[2][runData.accHistIndex];

					runData.accHist[0][runData.accHistIndex] = raw.accelerometer_m_s2[0];
					runData.accHist[1][runData.accHistIndex] = raw.accelerometer_m_s2[1];
					runData.accHist[2][runData.accHistIndex] = raw.accelerometer_m_s2[2];

					runData.sumAcc[0] += runData.accHist[0][runData.accHistIndex];
					runData.sumAcc[1] += runData.accHist[1][runData.accHistIndex];
					runData.sumAcc[2] += runData.accHist[2][runData.accHistIndex];
					runData.accHistIndex = (runData.accHistIndex + 1) % RUN_SENSOR_HIST;
				}

				// mag
				static uint64_t mag_counter = 0;
				if(raw.magnetometer_counter > mag_counter)
				{
					mag_counter = raw.magnetometer_counter;
					runData.sumMag[0] -= runData.magHist[0][runData.magHistIndex];
					runData.sumMag[1] -= runData.magHist[1][runData.magHistIndex];
					runData.sumMag[2] -= runData.magHist[2][runData.magHistIndex];

					runData.magHist[0][runData.magHistIndex] = raw.magnetometer_ga[0];
					runData.magHist[1][runData.magHistIndex] = raw.magnetometer_ga[1];
					runData.magHist[2][runData.magHistIndex] = raw.magnetometer_ga[2];

					runData.sumMag[0] += runData.magHist[0][runData.magHistIndex];
					runData.sumMag[1] += runData.magHist[1][runData.magHistIndex];
					runData.sumMag[2] += runData.magHist[2][runData.magHistIndex];
					runData.magHistIndex = (runData.magHistIndex + 1) % RUN_SENSOR_HIST;
				}

				// pressure
				static uint64_t baro_counter = 0;
				if(raw.baro_counter > baro_counter)
				{
					mag_counter = raw.baro_counter;
					runData.sumPres -= runData.presHist[runData.presHistIndex];
					runData.presHist[runData.presHistIndex] = raw.baro_pres_mbar;
					runData.sumPres += runData.presHist[runData.presHistIndex];
					runData.presHistIndex = (runData.presHistIndex + 1) % RUN_SENSOR_HIST;
				}
				if (runData.accHistIndex == 3) {
				   simDoAccUpdate(	runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	&state,
						   	   	   	&ukf_params);
				}
				if (runData.presHistIndex == 6) {
				   simDoPresUpdate(runData.sumPres*(1.0 / (float)RUN_SENSOR_HIST),
				   	   	   			&state,
				   	   	   			&ukf_params);
				}
				if (runData.magHistIndex == 9) {
				   simDoMagUpdate(runData.sumMag[0]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[1]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[2]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  &state,
						   	   	  &ukf_params);
				}
			}
			if (fds[1].revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_data);
				if (timestamp_position < gps_data.timestamp_position) {
					timestamp_position = gps_data.timestamp_position;
					float hAcc = gps_data.eph_m;
				    if (runData.accMask > 1.0f &&  hAcc < 5.0f) {
					// 50 readings before mask is completely dropped
					runData.accMask -= RUN_ACC_MASK / 50.0f;
					if (runData.accMask < 1.0f)
					    runData.accMask = 1.0f;
				    }
				    navUkfGpsPosUpate(&gps_data,dt,&state,&ukf_params);
				    // refine static sea level pressure based on better GPS altitude fixes
				    if (hAcc < runData.bestHacc && hAcc < NAV_MIN_GPS_ACC) {
				    	UKFPressureAdjust((float)gps_data.alt * 1e3f);
				    	runData.bestHacc = hAcc;
				    }
				}
				if (timestamp_velocity < gps_data.timestamp_velocity) {
					timestamp_velocity = gps_data.timestamp_velocity;
				    navUkfGpsVelUpate(&gps_data,dt,&state,&ukf_params);
				}
			}
			// observe that the rates are exactly 0 if not flying or moving
			bool mightByFlying = state.flag_system_armed;
			if (!loopcounter % 100 && !mightByFlying ) {
			    static uint32_t axis = 0;
			    float stdX, stdY, stdZ;

			    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
			    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
			    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

			    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
			    	navUkfZeroRate(raw.gyro_rad_s[2], (axis++) % 3);
			    }
			}

			navUkfFinish();
			navNavigate(&gps_data,&state,&nav_params,&manual,hrt_absolute_time());

			// rotate nav's NE frame of reference to our craft's local frame of reference
			att_sp.pitch_body = navData.holdTiltN * navUkfData.yawCos - navData.holdTiltE * navUkfData.yawSin;
			att_sp.roll_body  = navData.holdTiltE * navUkfData.yawCos + navData.holdTiltN * navUkfData.yawSin;
			att_sp.thrust = pidUpdate(navData.altSpeedPID, navData.holdSpeedAlt, -UKF_VELD);
			att_sp.yaw_body = navData.targetHeading;
			att_sp.timestamp = hrt_absolute_time();

			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			if (!loopcounter % 10) {
				global_position.alt = UKF_ALTITUDE;
				global_position.lat = UKF_POSN;
				global_position.lon = UKF_POSE;
				global_position.vx = UKF_VELE * navUkfData.yawCos + UKF_VELN * navUkfData.yawSin;
				global_position.vy = UKF_VELN * navUkfData.yawCos - UKF_VELE * navUkfData.yawSin;
				global_position.vz = UKF_VELD;
				global_position.timestamp = hrt_absolute_time();
				global_position.hdg = navUkfData.yaw;
				orb_publish(ORB_ID(vehicle_global_position), global_position_pub, &global_position);
			}
			perf_end(mc_loop_perf);
			// /* print debug information every 500th time */
			if (debug == true && printcounter % 500 == 0)
			{
				printf("GPS trust: hAcc:%8.4f dt:%8.4f", gps_data.eph_m, dt);
				printf("GPS input: alt:%d lat:%d, lon:%d ", gps_data.alt, gps_data.lat, gps_data.lon);
				printf("setpoint: pitch:%8.4f roll:%8.4f thrust:%8.4f yaw:%8.4f", att_sp.pitch_body, att_sp.roll_body, att_sp.thrust, att_sp.yaw_body);
				printf("Global position alt;%8.4f lat:%8.4f lon:%8.4f vx:%8.4f vy:%8.4f vz:%8.4f yaw:%8.4f ", global_position.alt, global_position.lat, global_position.lon, global_position.vx, global_position.vy, global_position.vz, global_position.hdg);
			}
			printcounter++;
		}
		loopcounter++;
	}
	printf("[quat pos control] ending now...\n");
	thread_running = false;
	fflush(stdout);
	return 0;
}

