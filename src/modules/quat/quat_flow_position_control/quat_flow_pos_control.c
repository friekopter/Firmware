
/**
 * @file quat_flow_pos_control.c
 *
 * Quat flow position controller
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <float.h>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <geo/geo.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/ukf_state_vector.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <quat/utils/quat_pos_control_params.h>
#include "nav_flow.h"
#include "nav_flow_ukf.h"
#include <quat/utils/quat_constants.h>
#include <quat/utils/util.h>

#include "quat_flow_pos_control.h"

runStruct_t runData __attribute__((section(".ccm")));
// Struct for data output. Defined here to reduce stack frame size
static struct vehicle_attitude_setpoint_s att_sp __attribute__((section(".ccm")));
static struct vehicle_local_position_setpoint_s local_position_sp __attribute__((section(".ccm")));
static struct position_setpoint_s position_sp __attribute__((section(".ccm")));
static struct vehicle_attitude_s att __attribute__((section(".ccm")));
static struct manual_control_setpoint_s manual;
static struct vehicle_control_mode_s control_mode;
static struct vehicle_status_s vstatus;
static struct sensor_combined_s raw;
static struct vehicle_local_position_s local_position_data;
static struct vehicle_global_position_s global_position_data;
static struct filtered_bottom_flow_s filtered_bottom_flow_data;
static struct ukf_state_vector_s ukf_state;
struct vehicle_gps_position_s 	gps_data;


static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static bool debug = false;
static int32_t run_sensor_hist = 0;
static int buzzer;
static bool inAir = false;


__EXPORT int quat_flow_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int quat_flow_pos_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static int buzzer_init(void);
static void buzzer_deinit(void);

static int buzzer_init(void)
{
	buzzer = open("/dev/tone_alarm", O_WRONLY);

	if (buzzer < 0) {
		warnx("Buzzer: open fail\n");
		return ERROR;
	}

	return OK;
}

static void buzzer_deinit(void)
{
	close(buzzer);
}

static void
setRunSensorHistNumber(int32_t number)
{
	run_sensor_hist = number;
	if(run_sensor_hist > RUN_SENSOR_HIST_MAX) {
		run_sensor_hist = RUN_SENSOR_HIST_MAX;
	}
	if(run_sensor_hist < 1) {
		run_sensor_hist = 1;
	}
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: deamon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

static void quat_flow_pos_runInit(const struct sensor_combined_s* sensors);

static void quat_flow_pos_runInit(const struct sensor_combined_s* sensors) {
    float acc[3], mag[3];
    float pres;
    int i;

    memset((void *)&runData, 0, sizeof(runData));

    acc[0] = sensors->accelerometer_m_s2[0];
    acc[1] = sensors->accelerometer_m_s2[1];
    acc[2] = sensors->accelerometer_m_s2[2];

    mag[0] = sensors->magnetometer_ga[0];
    mag[1] = sensors->magnetometer_ga[1];
    mag[2] = sensors->magnetometer_ga[2];

    pres = sensors->baro_pres_mbar;

    for (i = 0; i < RUN_SENSOR_HIST_MAX; i++) {
	runData.accHist[0][i] = acc[0];
	runData.accHist[1][i] = acc[1];
	runData.accHist[2][i] = acc[2];
	runData.magHist[0][i] = mag[0];
	runData.magHist[1][i] = mag[1];
	runData.magHist[2][i] = mag[2];
	runData.presHist[i] = pres;

	runData.sumAcc[0] += acc[0];
	runData.sumAcc[1] += acc[1];
	runData.sumAcc[2] += acc[2];
	runData.sumMag[0] += mag[0];
	runData.sumMag[1] += mag[1];
	runData.sumMag[2] += mag[2];
	runData.sumPres += pres;
    }

    runData.accHistIndex = 0;
    runData.magHistIndex = 0;
    runData.presHistIndex = 0;
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_spawn().
 */
int quat_flow_pos_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "debug")){
		debug = true;
	}
	if (!strcmp(argv[1], "start") || !strcmp(argv[1], "debug")) {

		if (thread_running) {
			printf("quat flow pos control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("quat flow pos control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 60,
					 4096,
					 quat_flow_pos_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tquat flow pos control app is running\n");
		} else {
			printf("\tquat flow pos control app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

static int
quat_flow_pos_control_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[quat flow pos control] Control started, taking over position control\n");
	thread_running = true;
	int loopcounter = 0;
	int printcounter = 0;
	float acc_noise = 0.0f;
	bool sonarValid = false;
	bool gpsValid = false;

	// Output
	// Calculation result is the attitude setpoint
	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	memset(&att_sp, 0, sizeof(att_sp));

	//publish local position
	orb_advert_t local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &local_position_data);
	memset(&local_position_data, 0, sizeof(local_position_data));
	local_position_data.landed = true;

	//publish global position
	orb_advert_t global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_position_data);
	memset(&global_position_data, 0, sizeof(global_position_data));

	// Attitude
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
	memset(&att, 0, sizeof(att));

	// UKF state
	orb_advert_t pub_ukf_state = orb_advertise(ORB_ID(ukf_state_vector), &ukf_state);
	memset(&ukf_state, 0, sizeof(ukf_state));

	//Debug only
	//orb_advert_t pub_debug = orb_advertise(ORB_ID(debug_key_value), &debug_data);
	//memset(&debug_data, 0, sizeof(debug_data));

	// Inputs
	// manual control
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	memset(&manual, 0, sizeof(manual));
	/* rate-limit parameter updates to 50Hz */
	//orb_set_interval(manual_sub, 20);

	// System state
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	memset(&control_mode, 0, sizeof(control_mode));
	/* rate-limit parameter updates to 10Hz */
	//orb_set_interval(state_sub, 100);

	// Parameter
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* rate-limit parameter updates to 1Hz */
	orb_set_interval(sub_params, 1000);
	//Init parameters
	struct quat_position_control_UKF_params ukf_params;
	struct quat_position_control_UKF_param_handles ukf_handles;
	struct quat_position_control_NAV_params nav_params;
	struct quat_position_control_NAV_param_handles nav_handles;
	parameters_init(&nav_handles, &ukf_handles);
	parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);
	setRunSensorHistNumber(ukf_params.ukf_sens_hist);

	// Raw data
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	memset(&raw, 0, sizeof(raw));
	/* rate-limit raw data updates to 150Hz(200Hz) */
	orb_set_interval(sub_raw, (unsigned)ukf_params.ukf_raw_intv);

	// Flow Velocity
	int filtered_bottom_flow_sub = orb_subscribe(ORB_ID(filtered_bottom_flow));
	memset(&filtered_bottom_flow_data, 0, sizeof(filtered_bottom_flow_data));
	/* rate-limit flow updates to 10Hz */
	//orb_set_interval(filtered_bottom_flow_sub, 100);

	// GPS
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	memset(&gps_data, 0, sizeof(gps_data));
	orb_set_interval(gps_sub, 200);	/* 5Hz updates */

	// Local position setpoint
	int local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	memset(&local_position_sp, 0, sizeof(local_position_sp));
	local_position_sp.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	// Sometimes also publish
	orb_advert_t local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &local_position_sp);

	// Position setpoint
	int pos_sp_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	memset(&position_sp, 0, sizeof(position_sp));
	position_sp.type = SETPOINT_TYPE_LOITER;

	// Vehicle Status
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	memset(&vstatus, 0, sizeof(vstatus));
	/* rate-limit updates to 1Hz */
	orb_set_interval(sub_params, 1000);

	sleep(2);
	/* register the perf counter */
	perf_counter_t quat_flow_pos_loop_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_control");
	perf_counter_t quat_flow_pos_sensor_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_sensor_control");
	perf_counter_t quat_flow_pos_inertial_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_inertial_control");
	perf_counter_t quat_flow_pos_nav_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_nav_control");
	perf_counter_t quat_flow_ukf_finish_perf = perf_alloc(PC_ELAPSED, "quat_flow_ukf_finish_perf");
	perf_counter_t quat_flow_position_perf = perf_alloc(PC_ELAPSED, "quat_flow_position_perf");
	struct pollfd fds[8] = {
		{ .fd = sub_raw,   .events = POLLIN },
		{ .fd = filtered_bottom_flow_sub, .events = POLLIN },
		{ .fd = manual_sub,   .events = POLLIN },
		{ .fd = sub_params, .events = POLLIN },
		{ .fd = control_mode_sub, .events = POLLIN },
		{ .fd = gps_sub, .events = POLLIN },
		{ .fd = pos_sp_sub, .events = POLLIN },
		{ .fd = vehicle_status_sub, .events = POLLIN }
	};

	//sleep(1);
	bool firstReadCompleted = false;
	while (!firstReadCompleted) {
		usleep(1000);
		int ret = poll(fds, 1, 1000);
		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
			printf("[quat flow pos control] Poll error");
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[quat flow pos control] WARNING: Not getting sensor data for init- sensor app running?\n");
		}
		else
		{
			if (fds[0].revents & POLLIN)
			{
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				firstReadCompleted = true;
			}
		}
	}

	buzzer_init();
	ioctl(buzzer, TONE_SET_ALARM, TONE_NOTIFY_NEUTRAL_TUNE);

	printf("[quat flow pos control] Init position control\n");
	quat_flow_pos_runInit(&raw);
	printf("[quat flow pos control] Init ukf with states:%d\n",ukf_params.ukf_states);
	printf("[quat flow pos control] Inclination:%8.4f\n",raw.magnetometer_inclination);
	navFlowUkfInit(&ukf_params,
			&raw,
			raw.magnetometer_inclination);
	printf("[quat flow pos control] Init ukf done. States:%d,\t%d\n",navFlowUkfData.kf->S, navFlowUkfData.kf->V);
	//sleep(1);
	bool initCompleted = false;
	printf("[quat flow pos control] Mag-Ref X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)navFlowUkfData.v0m[0], (double)navFlowUkfData.v0m[1], (double)navFlowUkfData.v0m[2]);
	while (!initCompleted) {
		int ret = poll(fds, 1, 1000);
		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
			printf("[quat flow pos control] Poll error");
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[quat flow pos control] WARNING: Not getting sensor data for init- sensor app running?\n");
		}
		else
		{
			if (fds[0].revents & POLLIN)
			{
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				// check for static position of imu
			    float stdX, stdY, stdZ;
			    static float accX[UKF_GYO_AVG_NUM];
			    static float accY[UKF_GYO_AVG_NUM];
			    static float accZ[UKF_GYO_AVG_NUM];
			    static int i = 0;
			    static int j = 0;

			    accX[j] = raw.accelerometer_m_s2[0];
			    accY[j] = raw.accelerometer_m_s2[1];
			    accY[j] = raw.accelerometer_m_s2[2];
				j = (j + 1) % UKF_GYO_AVG_NUM;

				if (i >= UKF_GYO_AVG_NUM) {
				    arm_std_f32(accX, UKF_GYO_AVG_NUM, &stdX);
				    arm_std_f32(accY, UKF_GYO_AVG_NUM, &stdY);
				    arm_std_f32(accZ, UKF_GYO_AVG_NUM, &stdZ);
				}
				i++;
			    if (i <= UKF_GYO_AVG_NUM ||
			    		(stdX + stdY + stdZ) > IMU_STATIC_STD) {
			    	// imu not static
			    	continue;
			    }
			    // imu static
				float rotError[3];
				float acc[3], mag[3], estAcc[3], estMag[3];
				float m[3*3];
				static int k = 0;
				static int l = 0;
			    static float gyX[UKF_GYO_AVG_NUM];
			    static float gyY[UKF_GYO_AVG_NUM];
			    static float gyZ[UKF_GYO_AVG_NUM];

				gyX[k] = raw.gyro_rad_s[0];
				gyY[k] = raw.gyro_rad_s[1];
				gyZ[k] = raw.gyro_rad_s[2];
				k = (k + 1) % UKF_GYO_AVG_NUM;

				acc[0] = raw.accelerometer_m_s2[0];
				acc[1] = raw.accelerometer_m_s2[1];
				acc[2] = raw.accelerometer_m_s2[2];

				mag[0] = raw.magnetometer_ga[0];
				mag[1] = raw.magnetometer_ga[1];
				mag[2] = raw.magnetometer_ga[2];

				utilNormalizeVec3(acc, acc);
				utilNormalizeVec3(mag, mag);

				//utilQuatToMatrix(m, &UKF_FLOW_Q1, 1);

				// rotate gravity to body frame of reference
				utilRotateVectorByRevQuat(estAcc,navFlowUkfData.v0a,&UKF_FLOW_Q1);
				//utilRotateVecByRevMatrix(estAcc, navFlowUkfData.v0a, m);

				// rotate mags to body frame of reference
				utilRotateVectorByRevQuat(estMag,navFlowUkfData.v0m,&UKF_FLOW_Q1);
				//utilRotateVecByRevMatrix(estMag, navFlowUkfData.v0m, m);

				// measured error
				rotError[0] = -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 0.5f;
				rotError[1] = -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 0.5f;
				rotError[2] = -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 0.5f;
				//printf("[quat flow pos control] Estmag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)estMag[0], (double)estMag[1], (double)estMag[2]);
				//printf("[quat flow pos control] Mag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)mag[0], (double)mag[1], (double)mag[2]);
				printf("[quat flow pos control] Merr X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)rotError[0], (double)rotError[1], (double)rotError[2]);
				rotError[0] += -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 0.5f;
				rotError[1] += -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 0.5f;
				rotError[2] += -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 0.5f;


			    utilRotateQuat(&UKF_FLOW_Q1, &UKF_FLOW_Q1, rotError, 0.1f);

				if (l >= UKF_GYO_AVG_NUM) {
				    arm_std_f32(gyX, UKF_GYO_AVG_NUM, &stdX);
				    arm_std_f32(gyY, UKF_GYO_AVG_NUM, &stdY);
				    arm_std_f32(gyZ, UKF_GYO_AVG_NUM, &stdZ);
				}
				float std = stdX + stdY + stdZ;
				//printf("[quat flow pos control] Init %d:\t std: %8.4f\t errorX: %8.4f\t errorY: %8.4f\t errorZ: %8.4f\n", l, (double)std, (double)rotError[0], (double)rotError[1], (double)rotError[2]);

				l++;
			    if (l > UKF_GYO_AVG_NUM*10 && std < 0.004f) {
			    	initCompleted = true;
			    	printf("[quat flow pos control] Mag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)mag[0], (double)mag[1], (double)mag[2]);
			    	printf("[quat flow pos control] Est X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)estMag[0], (double)estMag[1], (double)estMag[2]);
					printf("[quat flow pos control] err X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)rotError[0], (double)rotError[1], (double)rotError[2]);
				    arm_mean_f32(gyX, UKF_GYO_AVG_NUM, &UKF_FLOW_GYO_BIAS_X);
				    arm_mean_f32(gyY, UKF_GYO_AVG_NUM, &UKF_FLOW_GYO_BIAS_Y);
				    arm_mean_f32(gyZ, UKF_GYO_AVG_NUM, &UKF_FLOW_GYO_BIAS_Z);
			    	printf("[quat flow pos control] Init finished. Gyo Bias: x: %8.4f\ty: %8.4f\tz:%8.4f\n", (double)UKF_FLOW_GYO_BIAS_X, (double)UKF_FLOW_GYO_BIAS_Y, (double)UKF_FLOW_GYO_BIAS_Z);
			    	printf("[quat flow pos control] Q1:%8.4f\tQ2:%8.4f\tQ3:%8.4f\tQ4:%8.4f\n", UKF_FLOW_Q1, UKF_FLOW_Q2, UKF_FLOW_Q3, UKF_FLOW_Q4);
					navFlowUkfFinish();
					// Publish attitude
					att.R_valid = false;
					att.roll = navFlowUkfData.roll;
					att.pitch = navFlowUkfData.pitch;
					att.yaw = navFlowUkfData.yaw;
					att.rollspeed = raw.gyro_rad_s[0];
					att.pitchspeed = raw.gyro_rad_s[1];
					att.yawspeed = raw.gyro_rad_s[2];
					att.q_valid = false;
					printf("roll: %8.4f\tpitch: %8.4f\tyaw:%8.4f\n", (double)att.roll, (double)att.pitch, (double)att.yaw);
					orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
					printf("1:%8.4f\t2:%8.4f\t3:%8.4f\t4:%8.4f\t5:%8.4f\t6:%8.4f\t7:%8.4f\t8:%8.4f\t9:%8.4f\t10:%8.4f\t11:%8.4f\t12:%8.4f\t13:%8.4f\t14:%8.4f\n",
							navFlowUkfData.x[0],navFlowUkfData.x[1],navFlowUkfData.x[2],navFlowUkfData.x[3],navFlowUkfData.x[4],
							navFlowUkfData.x[5],navFlowUkfData.x[6],navFlowUkfData.x[7],navFlowUkfData.x[8],navFlowUkfData.x[9],
							navFlowUkfData.x[10],navFlowUkfData.x[11],navFlowUkfData.x[12],navFlowUkfData.x[13]);

			    }
				//usleep(1000);
			}
		}
	}
	printf("[quat flow pos control] Init nav\n");
	printf("Ground level altitude: %8.4f meters\n",raw.baro_alt_meter);
	navFlowInit(&nav_params,raw.baro_alt_meter,navFlowUkfData.yaw);
	navFlowUkfSetSonarOffset(0.0f,raw.baro_alt_meter,1.0f);
	printf("Ground level offset: %8.4f meters\n",navFlowUkfData.sonarAltOffset);
	printf("[quat flow pos control] Starting loop\n");

	ioctl(buzzer, TONE_SET_ALARM, TONE_NOTIFY_POSITIVE_TUNE);
	usleep(1000000);
	buzzer_deinit();

	// Init local to global transformation
	local_position_data.ref_lat = 481292910;
	local_position_data.ref_lon = 117061650;
	double lat_home = (float)local_position_data.ref_lat * 1e-7f;
	double lon_home = (float)local_position_data.ref_lon * 1e-7f;
	map_projection_init(lat_home, lon_home);

	///////////////////////////////////////////
	// Start main loop
	///////////////////////////////////////////
	printf("[quat flow pos control] States: %u\n", (uint8_t)ukf_params.ukf_states);
	while (!thread_should_exit) {
		int ret = poll(fds, 2, 1000);
		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[quat flow pos control] WARNING: Not getting sensor data - sensor app running?\n");
		}
		else
		{
			static float dt = 0;
			perf_begin(quat_flow_pos_loop_perf);
			bool updated = false;
			orb_check(fds[4].fd, &updated);
			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
			}
			/* only update parameters if they changed */
			orb_check(fds[3].fd, &updated);
			if (updated)
			{
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);
				orb_set_interval(sub_raw, (unsigned)ukf_params.ukf_raw_intv);
				setRunSensorHistNumber(ukf_params.ukf_sens_hist);
			}
			orb_check(fds[2].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			}
			orb_check(fds[7].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);
			}
			if (fds[0].revents & POLLIN)
			{
				//float yawBefore = atan2f((2.0f * (UKF_FLOW_Q2 * UKF_FLOW_Q3 + UKF_FLOW_Q1 * UKF_FLOW_Q4)),
					//	   (UKF_FLOW_Q1*UKF_FLOW_Q1 - UKF_FLOW_Q3*UKF_FLOW_Q3 - UKF_FLOW_Q4*UKF_FLOW_Q4 + UKF_FLOW_Q2*UKF_FLOW_Q2));

				// raw parameter changed
				perf_begin(quat_flow_pos_inertial_perf);
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				if ( !((loopcounter + 2) % ukf_params.ukf_bias_update_count)  ) {
					dt = navFlowUkfInertialUpdate(&raw,true);
				} else {
					dt = navFlowUkfInertialUpdate(&raw,false);
				}
				perf_end(quat_flow_pos_inertial_perf);

				/*float yawAfter = atan2f((2.0f * (UKF_FLOW_Q2 * UKF_FLOW_Q3 + UKF_FLOW_Q1 * UKF_FLOW_Q4)),
						   (UKF_FLOW_Q1*UKF_FLOW_Q1 - UKF_FLOW_Q3*UKF_FLOW_Q3 - UKF_FLOW_Q4*UKF_FLOW_Q4 + UKF_FLOW_Q2*UKF_FLOW_Q2));
				yawDiff1 += yawAfter - yawBefore;*/
				if(dt < FLT_MIN) {
					perf_end(quat_flow_pos_loop_perf);
					continue;
				}

				perf_begin(quat_flow_pos_sensor_perf);
				// record history for acc & mag & pressure readings for smoothing purposes
				// acc
				static uint64_t acc_timestamp = 0;
				if(raw.accelerometer_timestamp > acc_timestamp)
				{
					acc_timestamp = raw.accelerometer_timestamp;
					runData.sumAcc[0] -= runData.accHist[0][runData.accHistIndex];
					runData.sumAcc[1] -= runData.accHist[1][runData.accHistIndex];
					runData.sumAcc[2] -= runData.accHist[2][runData.accHistIndex];

					runData.accHist[0][runData.accHistIndex] = raw.accelerometer_m_s2[0];
					runData.accHist[1][runData.accHistIndex] = raw.accelerometer_m_s2[1];
					runData.accHist[2][runData.accHistIndex] = raw.accelerometer_m_s2[2];

					runData.sumAcc[0] += runData.accHist[0][runData.accHistIndex];
					runData.sumAcc[1] += runData.accHist[1][runData.accHistIndex];
					runData.sumAcc[2] += runData.accHist[2][runData.accHistIndex];
					runData.accHistIndex = (runData.accHistIndex + 1) % run_sensor_hist;
				}

				// mag
				static uint64_t mag_timestamp = 0;
				if(raw.magnetometer_timestamp > mag_timestamp)
				{
					mag_timestamp = raw.magnetometer_timestamp;
					runData.sumMag[0] -= runData.magHist[0][runData.magHistIndex];
					runData.sumMag[1] -= runData.magHist[1][runData.magHistIndex];
					runData.sumMag[2] -= runData.magHist[2][runData.magHistIndex];

					runData.magHist[0][runData.magHistIndex] = raw.magnetometer_ga[0];
					runData.magHist[1][runData.magHistIndex] = raw.magnetometer_ga[1];
					runData.magHist[2][runData.magHistIndex] = raw.magnetometer_ga[2];

					runData.sumMag[0] += runData.magHist[0][runData.magHistIndex];
					runData.sumMag[1] += runData.magHist[1][runData.magHistIndex];
					runData.sumMag[2] += runData.magHist[2][runData.magHistIndex];
					runData.magHistIndex = (runData.magHistIndex + 1) % run_sensor_hist;
				}

				// pressure
				static uint64_t baro_timestamp = 0;
				if(raw.baro_timestamp > baro_timestamp)
				{
					baro_timestamp = raw.baro_timestamp;
					runData.sumPres -= runData.presHist[runData.presHistIndex];
					runData.presHist[runData.presHistIndex] = raw.baro_pres_mbar;
					runData.sumPres += runData.presHist[runData.presHistIndex];
					runData.presHistIndex = (runData.presHistIndex + 1) % run_sensor_hist;
				}
				if (!(loopcounter % 20)) {
					acc_noise = navFlowDoAccUpdate(	runData.sumAcc[0]*(1.0f / (float)run_sensor_hist),
									runData.sumAcc[1]*(1.0f / (float)run_sensor_hist),
									runData.sumAcc[2]*(1.0f / (float)run_sensor_hist),
									&control_mode,
									&ukf_params);
					float stdDeviationAcc = 0;
					static int accDeviationCounter = 0;
					arm_std_f32(runData.accHist[2],run_sensor_hist,&stdDeviationAcc);
					bool inAirVibrations = stdDeviationAcc > nav_params.nav_in_air_acc_deviation;
					if(inAirVibrations != inAir) {
						accDeviationCounter++;
					}
					//Delay switch
					if(accDeviationCounter > 10) {
						inAir = inAirVibrations;
					}
					if(inAirVibrations == inAir) {
						accDeviationCounter = 0;
					}
				}
				else if (!((loopcounter+7) % 20)) {
					navFlowDoPresUpdate(runData.sumPres*(1.0f / (float)run_sensor_hist),
				   	   	   			&control_mode,
				   	   	   			&ukf_params);
				}
				else if (!((loopcounter+14) % 20)) {
					navFlowDoMagUpdate(runData.sumMag[0]*(1.0f / (float)run_sensor_hist),
									  runData.sumMag[1]*(1.0f / (float)run_sensor_hist),
									  runData.sumMag[2]*(1.0f / (float)run_sensor_hist),
									  &control_mode,
									  &ukf_params);
				}
				perf_begin(quat_flow_ukf_finish_perf);
				navFlowUkfFinish();

/*				if(!(yawDiffCount3 % 100)) {
					printf("yawspeed, bias:%8.4f  %8.4f  %8.4f\n",raw.gyro_rad_s[2], UKF_FLOW_GYO_BIAS_Z, ybias);
					printf("1 yaw diff:%8.4f\n",yawDiff1);
					printf("2 yaw diff:%8.4f\n",yawDiff2);
					printf("3 yaw diff:%8.4f\n",yawDiff3);
					printf("yaw:%8.4f  %8.4f\n",yawAfter, navFlowUkfData.yaw);
					yawDiff1 = 0.0f;
					yawDiff2 = 0.0f;
					yawDiff3 = 0.0f;
					ybias = 0.0f;
				}
				yawDiffCount3++;*/

				// Publish attitude
				att.R_valid = true;
				utilQuatToMatrix2(att.R, &UKF_FLOW_Q1, 1);
				att.roll = navFlowUkfData.roll;
				att.pitch = navFlowUkfData.pitch;
				att.yaw = navFlowUkfData.yaw;
				att.rollspeed = raw.gyro_rad_s[0];// - UKF_FLOW_GYO_BIAS_X; //TODO: Remove bias here?
				att.pitchspeed = raw.gyro_rad_s[1];// - UKF_FLOW_GYO_BIAS_Y;
				att.yawspeed = raw.gyro_rad_s[2];// - UKF_FLOW_GYO_BIAS_Z;
				att.q_valid = false;
				orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
				perf_end(quat_flow_ukf_finish_perf);
				perf_end(quat_flow_pos_sensor_perf);
			}

			// Attitude and position calculation from inertial sensors finished
			// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			// Calculate velocity and position from Flow and GPS

			if (fds[1].revents & POLLIN)
			{
				static uint8_t count = 0;
				perf_begin(quat_flow_position_perf);
				orb_copy(ORB_ID(filtered_bottom_flow), filtered_bottom_flow_sub, &filtered_bottom_flow_data);
				if(!control_mode.flag_armed) {
					if(!(count++ % 2)){
						navFlowUkfFlowUpdate(&filtered_bottom_flow_data,dt,&control_mode,gpsValid,&ukf_params);
					} else {
						sonarValid = navFlowUkfSonarUpdate(&filtered_bottom_flow_data,dt,raw.baro_alt_meter,&control_mode,&ukf_params);
					}
				}
				else if(filtered_bottom_flow_data.ned_xy_valid > 0) {
					navFlowUkfFlowUpdate(&filtered_bottom_flow_data,dt,&control_mode,gpsValid,&ukf_params);
				} else {
					sonarValid = navFlowUkfSonarUpdate(&filtered_bottom_flow_data,dt,raw.baro_alt_meter,&control_mode,&ukf_params);
				}
				////navFlowUkfFlowPosUpate(&filtered_bottom_flow_data,&control_mode,&ukf_params);
				////navFlowUkfFlowVelUpate(&filtered_bottom_flow_data,&control_mode,&ukf_params);
				perf_end(quat_flow_position_perf);
			}
			else if (fds[5].revents & POLLIN)
			{
				static uint64_t timestamp_velocity = 0;
				static uint64_t timestamp_position = 0;
				orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_data);
				if(gps_data.timestamp_position > timestamp_position) {
					timestamp_position = gps_data.timestamp_position;
					//gpsValid = navFlowUkfGpsPosUpate(&gps_data,&local_position_data,dt,&control_mode,&ukf_params);
				} else if(gps_data.timestamp_velocity > timestamp_velocity) {
					timestamp_velocity = gps_data.timestamp_velocity;
					//gpsValid = navFlowUkfGpsVelUpate(&gps_data,dt,&control_mode,&ukf_params);
				}
			}

			if (fds[6].revents & POLLIN)
			{
				// only in case of auto we listen for the setpoint, otherwise we self calculate it
				orb_copy(ORB_ID(position_setpoint_triplet), pos_sp_sub, &position_sp);
				printf("[quat flow pos control] position setpoint %d, %8.4f, %8.4f, %8.4f, %8.4f",position_sp.type,
						position_sp.alt, position_sp.lat, position_sp.lon, position_sp.yaw);
			}

			if (!(loopcounter % 100) && !control_mode.flag_armed ) {
			    static int axis = 0;
			    float stdX, stdY, stdZ;

			    arm_std_f32(runData.accHist[0], run_sensor_hist, &stdX);
			    arm_std_f32(runData.accHist[1], run_sensor_hist, &stdY);
			    arm_std_f32(runData.accHist[2], run_sensor_hist, &stdZ);

			    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
			    	int current_axis = (axis++) % 3;
			    	navFlowUkfZeroRate(raw.gyro_rad_s[current_axis], current_axis);
			    }
			}

			// Finished State calculation
			// ++++++++++++++++++++++++++++++++++++++++

			// Set local position calculation results
			if(UKF_FLOW_CALCULATES_POSITION) {
				local_position_data.x = UKF_FLOW_POSX;
				local_position_data.y = UKF_FLOW_POSY;
			} else {
				local_position_data.x = filtered_bottom_flow_data.ned_x;
				local_position_data.y = filtered_bottom_flow_data.ned_y;
			}
			// local altitude is the negative distance to ground
			if(UKF_FLOW_CALCULATES_ALTITUDE) {
				local_position_data.z = UKF_FLOW_POSD + navFlowUkfData.sonarAltOffset;
			} else {
				local_position_data.z = -UKF_FLOW_PRES_ALT + navFlowUkfData.sonarAltOffset;
			}
			local_position_data.xy_valid = filtered_bottom_flow_data.ned_xy_valid;
			local_position_data.v_xy_valid = true;//filtered_bottom_flow_data.ned_v_xy_valid;
			//if (control_mode.flag_control_auto_enabled) {
				// auto mode can be started on ground, so set this always true
				local_position_data.z_valid = true;
				local_position_data.v_z_valid = true;
			/*}
			else {
				local_position_data.z_valid = filtered_bottom_flow_data.ned_z_valid;
				local_position_data.v_z_valid = filtered_bottom_flow_data.ned_v_z_valid;
			}*/
			local_position_data.vx = UKF_FLOW_VELX;
			local_position_data.vy = UKF_FLOW_VELY;
			local_position_data.vz = UKF_FLOW_VELD;
			local_position_data.timestamp = raw.timestamp;
			local_position_data.yaw = att.yaw;
			local_position_data.landed = !inAir;
			// Local position calculation results set

			// +++++++++++++++++++++++++++++++++++++++++++
			// Do navigation, calculate setpoints
			perf_begin(quat_flow_pos_nav_perf);
			navFlowNavigate(&control_mode,
							&vstatus,
							&nav_params,
							&manual,
							&local_position_data,
							&position_sp,
							&att,
							sonarValid,
							raw.timestamp);
			perf_end(quat_flow_pos_nav_perf);
			// rotate nav's NE frame of reference to our craft's local frame of reference
			// Tilt north means for yaw=0 nose up. If yaw=90 degrees it means left wing up that is positive roll
			float tilt[3] = { navFlowData.holdTiltX, navFlowData.holdTiltY, 0.0f };
			float tilt_body[3] = { 0.0f, 0.0f, 0.0f };
			utilRotateVecByRevMatrix2(tilt_body,tilt,att.R);
			att_sp.roll_body   = +tilt_body[1] * DEG_TO_RAD;
			att_sp.pitch_body  = -tilt_body[0] * DEG_TO_RAD;
			att_sp.thrust = navFlowData.autoThrust;
			if(control_mode.flag_control_manual_enabled) {
				// yaw control comes from autopilot
				att_sp.yaw_body = navFlowData.holdHeading;
			} else {
				// TODO: read att setpoint from attitude control
			}

			att_sp.timestamp = hrt_absolute_time();
			if(control_mode.flag_control_velocity_enabled ||
				control_mode.flag_control_altitude_enabled) {
				orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			}
			perf_end(quat_flow_pos_loop_perf);

			// +++++++++++++++++++++++++++++++++++++++++++
			// Publish Results
			//struct debug_key_value_s debug_message = {.key="alspeed", .timestamp_ms =filtered_bottom_flow_data.timestamp / 1000, .value = filtered_bottom_flow_data.ned_vz};
			//orb_publish(ORB_ID(debug_key_value), pub_debug, &debug_message);
			// Publish state
			if(!((printcounter+3) % 100)) {
				ukf_state.acc_bias_x = UKF_FLOW_ACC_BIAS_X;
				ukf_state.acc_bias_y = UKF_FLOW_ACC_BIAS_Y;
				ukf_state.acc_bias_z = UKF_FLOW_ACC_BIAS_Z;
				ukf_state.gyo_bias_x = UKF_FLOW_GYO_BIAS_X;
				ukf_state.gyo_bias_y = UKF_FLOW_GYO_BIAS_Y;
				ukf_state.gyo_bias_z = UKF_FLOW_GYO_BIAS_Z;
				ukf_state.vel_x = UKF_FLOW_VELX;
				ukf_state.vel_y = UKF_FLOW_VELY;
				ukf_state.vel_d = UKF_FLOW_VELD;
				ukf_state.q1 = UKF_FLOW_Q1;
				ukf_state.q2 = UKF_FLOW_Q2;
				ukf_state.q3 = UKF_FLOW_Q3;
				ukf_state.q4 = UKF_FLOW_Q4;
				ukf_state.pres_alt = UKF_FLOW_PRES_ALT;
				if(UKF_FLOW_CALCULATES_POSITION) {
					ukf_state.pos_x = UKF_FLOW_POSX;
					ukf_state.pos_y = UKF_FLOW_POSY;
				}
				if(UKF_FLOW_CALCULATES_ALTITUDE) {
					ukf_state.pos_d = UKF_FLOW_POSD;
				}
				ukf_state.acc_noise = acc_noise;
				orb_publish(ORB_ID(ukf_state_vector), pub_ukf_state, &ukf_state);
			}

			// Publish local position setpoint and local and global position
			if(!((printcounter + 10) % 20)) {
				orb_publish(ORB_ID(vehicle_local_position), local_pos_pub, &local_position_data);
			} else if(!((printcounter + 15) % 20)) {
				double lat = 0.0f;
				double lon = 0.0f;
				map_projection_reproject(local_position_data.x,local_position_data.y,
						&lat,&lon);
				global_position_data.lat = (int32_t) (lat * (double)1e7f);
				global_position_data.lon = (int32_t) (lon * (double)1e7f);
				//global_position_data.relative_alt = local_position_data.z;
				global_position_data.alt = UKF_FLOW_POSD;
				global_position_data.timestamp = hrt_absolute_time();
				global_position_data.global_valid = true;
				orb_publish(ORB_ID(vehicle_global_position), global_pos_pub, &global_position_data);
			} else if(!control_mode.flag_control_auto_enabled && !((printcounter + 18) % 20)) {
				local_position_sp.x = navFlowData.holdPositionX;
				local_position_sp.y = navFlowData.holdPositionY;
				local_position_sp.z = navFlowData.holdAlt;
				local_position_sp.yaw = navFlowData.holdHeading;
				orb_publish(ORB_ID(vehicle_local_position_setpoint), local_pos_sp_pub, &local_position_sp);
			}

			// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
			// Do some logging
			/*if(!(printcounter%20)){
				printf("[qfpc] vxy valid %d,filtered_bottom_flow_data.ned_xy_valid);
			}*/
			if (debug == true && !((printcounter+10) % 101)) {
				printf("local z: %8.4fm hold alt: %8.4f\n",local_position_data.z,navFlowData.holdAlt);
				printf("local vz: %8.4fm hold speed: %8.4f\n",navFlowData.holdSpeedAlt, local_position_data.vz);
				printf("Pressure alt offset: %8.4fm Pressure alt: %8.4fm\tBaro alt: %8.4fm\n",navFlowUkfData.pressAltOffset,UKF_FLOW_PRES_ALT,raw.baro_alt_meter);
				printf("Sonar offset: %8.4f meters\tSonar measured: %8.4f meters\tposd alt: %8.4fm\n",navFlowUkfData.sonarAltOffset,filtered_bottom_flow_data.ned_z, UKF_FLOW_POSD);
			}
			// print debug information every 1000th time
			if (debug == true && !(printcounter % 101))
			{
				printf("sonar alt:%8.4f\n",local_position_data.z);
				printf("measured alt:%8.4f\thold alt:%8.4f\ttarget hold speed:%8.4f\n",
						-UKF_FLOW_PRES_ALT,navFlowData.holdAlt,navFlowData.targetHoldSpeedAlt);
				printf("alt speed:%8.4f\thold speed:%8.4f\ttthrust:%8.4f\n",
						-UKF_FLOW_VELD,navFlowData.holdSpeedAlt,att_sp.thrust);

			}
			else if (debug == true && !(printcounter % 1000))
			{
				//float frequence = 0;
				//static uint32_t last_measure = 0;
				//uint32_t current = hrt_absolute_time();
				//frequence = 1000.0f*1000000.0f/(float)(current - last_measure);
				//last_measure = current;
				printf("------\n");
				navFlowLogVariance();
				/*
				printf("velx:%8.4f\tvely:%8.4f\tveld:%8.4f\n"
						"holdSpeedX:%8.4f\tholdSpeedY:%8.4f\tholdSpeedAlt:%8.4f\tTargetHoldSpeedAlt:%8.4f\n"
						"holdTiltX:%8.4f\tholdTiltY:%8.4f\tholdThrust:%8.4f\n"
						"accbx:%8.4f\taccby:%8.4f\taccbz:%8.4f\n"
						"gbybx:%8.4f\tgbyby:%8.4f\tgbybz:%8.4f\n"
						"q1:%8.4f\tq2:%8.4f\tq3:%8.4f\tq4:%8.4f\t"
						"presalt:%8.4f\tholdsalt:%8.4f\n",
					navFlowUkfData.x[0],navFlowUkfData.x[1],navFlowUkfData.x[2],
					navFlowData.holdSpeedX, navFlowData.holdSpeedY, navFlowData.holdSpeedAlt, navFlowData.targetHoldSpeedAlt,
					navFlowData.holdTiltX, navFlowData.holdTiltY, att_sp.thrust,
					navFlowUkfData.x[3],navFlowUkfData.x[4],navFlowUkfData.x[5],
					navFlowUkfData.x[6],navFlowUkfData.x[7],navFlowUkfData.x[8],
					navFlowUkfData.x[9],navFlowUkfData.x[10],navFlowUkfData.x[11],navFlowUkfData.x[12],
					navFlowUkfData.x[13],navFlowData.holdAlt);
				printf("roll:    %8.4f\tpitch:   %8.4f\tyaw:   %8.4f\n", (double)att.roll, (double)att.pitch, (double)att.yaw);
				printf("sp_roll: %8.4f\tsp_pitch:%8.4f\tsp_yaw:%8.4f\tthrust:%8.4f\n",
										att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body, att_sp.thrust);
				printf("Frequence: %8.4f\n", (double)frequence);
				printf("Acc x:%8.4f\ty:%8.4f\tz:%8.4f\n",
						raw.accelerometer_m_s2[0], raw.accelerometer_m_s2[1], raw.accelerometer_m_s2[2]);
				printf("Gyro x:%8.4f\ty:%8.4f\tz:%8.4f\n",
						raw.gyro_rad_s[0],raw.gyro_rad_s[1],raw.gyro_rad_s[2]);
				printf("Mag x:%8.4f\ty:%8.4f\tz:%8.4f\n",
						raw.magnetometer_ga[0],raw.magnetometer_ga[1],raw.magnetometer_ga[2]);
				printf("Pressure:%8.4f\t%8.4f\n",
						raw.baro_pres_mbar, runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST));
				printf("Throttle:%8.4f\tcapable:%d\n",
						manual.throttle,navFlowData.navCapable);*/
			}
			printcounter++;
		}
		loopcounter++;
	}
	printf("[quat flow pos control] ending now...\n");
	thread_running = false;
	fflush(stdout);
	return 0;
}



