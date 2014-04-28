
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
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <quat/utils/quat_pos_control_params.h>
#include "nav.h"
#include "nav_ukf.h"
#include <quat/utils/quat_constants.h>

#include "quat_pos_control.h"


runStruct_t runData __attribute__((section(".ccm")));
// Struct for data output. Defined here to reduce stack frame size
static struct vehicle_attitude_setpoint_s att_sp __attribute__((section(".ccm")));
static struct vehicle_global_position_s global_position __attribute__((section(".ccm")));
static struct vehicle_attitude_s att __attribute__((section(".ccm")));
static struct manual_control_setpoint_s manual;
static struct vehicle_status_s state;
static struct sensor_combined_s raw;
static struct vehicle_gps_position_s gps_data;
static struct filtered_bottom_flow_s flow_data;

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

static void quat_pos_runInit(const struct sensor_combined_s* sensors);

static void quat_pos_runInit(const struct sensor_combined_s* sensors) {
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

    for (i = 0; i < RUN_SENSOR_HIST; i++) {
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

    runData.bestHacc = 99.9f;
    runData.accMask = RUN_ACC_MASK;
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
		deamon_task = task_spawn_cmd("quat pos control",
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
	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	memset(&att_sp, 0, sizeof(att_sp));

	// For control reasons we also publish the suspected position
	orb_advert_t global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_position);
	memset(&global_position, 0, sizeof(global_position));

	// Attitude
	orb_advert_t pub_att = orb_advertise(ORB_ID(vehicle_attitude), &att);
	memset(&att, 0, sizeof(att));


	// Inputs
	// manual control
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	memset(&manual, 0, sizeof(manual));
	/* rate-limit parameter updates to 50Hz */
	//orb_set_interval(manual_sub, 20);

	// System state
	int state_sub = orb_subscribe(ORB_ID(vehicle_status));
	memset(&state, 0, sizeof(state));
	/* rate-limit parameter updates to 10Hz */
	//orb_set_interval(state_sub, 100);

	// Parameter
	int sub_params = orb_subscribe(ORB_ID(parameter_update));
	/* rate-limit parameter updates to 1Hz */
	orb_set_interval(sub_params, 1000);

	// Raw data
	int sub_raw = orb_subscribe(ORB_ID(sensor_combined));
	memset(&raw, 0, sizeof(raw));
	/* rate-limit raw data updates to 200Hz */
	orb_set_interval(sub_raw, 5);

	// GPS Position
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	memset(&gps_data, 0, sizeof(gps_data));
	/* rate-limit parameter updates to 5Hz */
	//orb_set_interval(gps_sub, 200);

	// Flow Velocity
	int flow_sub = orb_subscribe(ORB_ID(filtered_bottom_flow));
	memset(&flow_data, 0, sizeof(flow_data));
	/* rate-limit parameter updates to 10Hz */
	orb_set_interval(flow_sub, 100);

	//Init parameters
	struct quat_position_control_UKF_params ukf_params;
	struct quat_position_control_UKF_param_handles ukf_handles;
	struct quat_position_control_NAV_params nav_params;
	struct quat_position_control_NAV_param_handles nav_handles;
	parameters_init(&nav_handles, &ukf_handles);
	parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);

	sleep(2);
	/* register the perf counter */
	perf_counter_t quat_pos_loop_perf = perf_alloc(PC_ELAPSED, "quat_pos_control");
	perf_counter_t quat_pos_sensor_perf = perf_alloc(PC_ELAPSED, "quat_pos_sensor_control");
	perf_counter_t quat_pos_inertial_perf = perf_alloc(PC_ELAPSED, "quat_pos_inertial_control");
	perf_counter_t quat_pos_gps_perf = perf_alloc(PC_ELAPSED, "quat_pos_gps_control");
	perf_counter_t quat_pos_nav_perf = perf_alloc(PC_ELAPSED, "quat_pos_nav_control");

	struct pollfd fds[6] = {
		{ .fd = sub_raw,   .events = POLLIN },
		{ .fd = gps_sub,   .events = POLLIN },
		{ .fd = flow_sub, .events = POLLIN },
		{ .fd = manual_sub,   .events = POLLIN },
		{ .fd = sub_params, .events = POLLIN },
		{ .fd = state_sub, .events = POLLIN }
	};

	//sleep(1);
	bool firstReadCompleted = false;
	while (!firstReadCompleted) {
		int ret = poll(fds, 1, 1000);
		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
			printf("[quat pos control] Poll error");
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[quat pos control] WARNING: Not getting sensor data for init- sensor app running?\n");
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
	printf("[quat pos control] Init position control\n");
	quat_pos_runInit(&raw);
	printf("[quat pos control] Init ukf\n");
	navUkfInit(&ukf_params,&raw);
	//sleep(1);
	bool initCompleted = false;
	printf("[quat pos control] Mag-Ref X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)navUkfData.v0m[0], (double)navUkfData.v0m[1], (double)navUkfData.v0m[2]);
	while (!initCompleted) {
		int ret = poll(fds, 1, 1000);
		if (ret < 0)
		{
			/* XXX this is seriously bad - should be an emergency */
			printf("[quat pos control] Poll error");
		}
		else if (ret == 0)
		{
			/* XXX this means no sensor data - should be critical or emergency */
			printf("[quat pos control] WARNING: Not getting sensor data for init- sensor app running?\n");
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
			    if (i <= UKF_GYO_AVG_NUM || (stdX + stdY + stdZ) > IMU_STATIC_STD) {
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

				utilQuatToMatrix(m, &UKF_Q1, 1);

				// rotate gravity to body frame of reference
				utilRotateVecByRevMatrix(estAcc, navUkfData.v0a, m);

				// rotate mags to body frame of reference
				utilRotateVecByRevMatrix(estMag, navUkfData.v0m, m);

				// measured error
				rotError[0] = -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 0.50f;
				rotError[1] = -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 0.50f;
				rotError[2] = -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 0.50f;
				//printf("[quat pos control] Estmag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)estMag[0], (double)estMag[1], (double)estMag[2]);
				//printf("[quat pos control] Mag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)mag[0], (double)mag[1], (double)mag[2]);
				//printf("[quat pos control] Mag error errorX: %8.4f\t errorY: %8.4f\t errorZ: %8.4f\n", (double)rotError[0], (double)rotError[1], (double)rotError[2]);


				rotError[0] += -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
				rotError[1] += -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
				rotError[2] += -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;


			    utilRotateQuat(&UKF_Q1, &UKF_Q1, rotError, 0.1f);

				if (l >= UKF_GYO_AVG_NUM) {
				    arm_std_f32(gyX, UKF_GYO_AVG_NUM, &stdX);
				    arm_std_f32(gyY, UKF_GYO_AVG_NUM, &stdY);
				    arm_std_f32(gyZ, UKF_GYO_AVG_NUM, &stdZ);
				}
				float std = stdX + stdY + stdZ;
				printf("[quat pos control] Init %d:\t std: %8.4f\t errorX: %8.4f\t errorY: %8.4f\t errorZ: %8.4f\n", l, (double)std, (double)rotError[0], (double)rotError[1], (double)rotError[2]);

				l++;
			    if (l > UKF_GYO_AVG_NUM*5 && std < 0.004f) {
			    	initCompleted = true;
				    arm_mean_f32(gyX, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_X);
				    arm_mean_f32(gyY, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_Y);
				    arm_mean_f32(gyZ, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_Z);
			    	printf("[quat pos control] Init finished. Gyo Bias: x: %8.4f\ty: %8.4f\tz:%8.4f\n", (double)UKF_GYO_BIAS_X, (double)UKF_GYO_BIAS_Y, (double)UKF_GYO_BIAS_Z);
			    	printf("[quat pos control] Q1:%8.4f\tQ2:%8.4f\tQ3:%8.4f\tQ4:%8.4f\n", UKF_Q1, UKF_Q2, UKF_Q3, UKF_Q4);
					navUkfFinish();
					// Publish attitude
					att.R_valid = false;
					att.roll = navUkfData.roll;
					att.pitch = navUkfData.pitch;
					att.yaw = navUkfData.yaw;
					att.rollspeed = raw.gyro_rad_s[0];
					att.pitchspeed = raw.gyro_rad_s[1];
					att.yawspeed = raw.gyro_rad_s[2];
					att.q_valid = false;
					printf("roll: %8.4f\tpitch: %8.4f\tyaw:%8.4f\n", (double)att.roll, (double)att.pitch, (double)att.yaw);
					orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
					printf("1:%8.4f\t2:%8.4f\t3:%8.4f\t4:%8.4f\t5:%8.4f\t6:%8.4f\t7:%8.4f\t8:%8.4f\t9:%8.4f\t10:%8.4f\t11:%8.4f\t12:%8.4f\t13:%8.4f\t14:%8.4f\t15:%8.4f\t16:%8.4f\t17:%8.4f\n",
							navUkfData.x[0],navUkfData.x[1],navUkfData.x[2],navUkfData.x[3],navUkfData.x[4],
							navUkfData.x[5],navUkfData.x[6],navUkfData.x[7],navUkfData.x[8],navUkfData.x[9],
							navUkfData.x[10],navUkfData.x[11],navUkfData.x[12],navUkfData.x[13],navUkfData.x[14],
							navUkfData.x[15],navUkfData.x[16]);

			    }

			}
		}
	}
	printf("[quat pos control] Init nav\n");
	navInit(&nav_params,raw.baro_alt_meter,navUkfData.yaw);
	printf("[quat pos control] Starting loop\n");

	while (!thread_should_exit) {
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
			printf("[quat pos control] WARNING: Not getting sensor data - sensor app running?\n");
		}
		else
		{
			static float dt = 0;
			perf_begin(quat_pos_loop_perf);
			/* only update parameters if state changed */
			bool updated = false;
			orb_check(fds[5].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);
			}
			/* only update parameters if they changed */
			orb_check(fds[4].fd, &updated);
			if (updated)
			{
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), sub_params, &update);
				/* update parameters */
				parameters_update(&nav_handles, &nav_params, &ukf_handles, &ukf_params);
			}
			orb_check(fds[3].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			}
			if (fds[0].revents & POLLIN)
			{
				perf_begin(quat_pos_inertial_perf);
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				// raw parameter changed
				dt = navUkfInertialUpdate(&raw);
				perf_end(quat_pos_inertial_perf);
				if(dt < FLT_MIN) {
					perf_end(quat_pos_loop_perf);
					continue;
				}

				perf_begin(quat_pos_sensor_perf);
				// record history for acc & mag & pressure readings for smoothing purposes
				// acc
				static uint32_t acc_counter = 0;
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
				static uint32_t mag_counter = 0;
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
				static uint32_t baro_counter = 0;
				if(raw.baro_counter > baro_counter)
				{
					baro_counter = raw.baro_counter;
					runData.sumPres -= runData.presHist[runData.presHistIndex];
					runData.presHist[runData.presHistIndex] = raw.baro_pres_mbar;
					runData.sumPres += runData.presHist[runData.presHistIndex];
					runData.presHistIndex = (runData.presHistIndex + 1) % RUN_SENSOR_HIST;
				}

				if (!(loopcounter % 20)) {
				   simDoAccUpdate(	runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	&state,
						   	   	   	&ukf_params);
				}
				else if (!((loopcounter+7) % 20)) {
					simDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST),
				   	   	   			&state,
				   	   	   			&ukf_params);
				}
				else if (!((loopcounter+14) % 20)) {
					simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[1]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[2]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	  &state,
						   	   	  &ukf_params);
				}
				navUkfFinish();
				// Publish attitude
				att.R_valid = false;
				att.roll = navUkfData.roll;
				att.pitch = navUkfData.pitch;
				att.yaw = navUkfData.yaw;
				att.rollspeed = raw.gyro_rad_s[0];
				att.pitchspeed = raw.gyro_rad_s[1];
				att.yawspeed = raw.gyro_rad_s[2];
				att.q_valid = false;
				orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
				perf_end(quat_pos_sensor_perf);
			}
			if (fds[1].revents & POLLIN)
			{
				orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_data);
				if(dt < FLT_MIN) {
					perf_end(quat_pos_loop_perf);
					continue;
				}
				perf_begin(quat_pos_gps_perf);
				if (timestamp_position < gps_data.timestamp_position) {
					timestamp_position = gps_data.timestamp_position;
					float hAcc = gps_data.eph_m;
				    if (runData.accMask > 1.0f &&  hAcc < 5.0f) {
					// 50 readings before mask is completely dropped
					runData.accMask -= RUN_ACC_MASK / 50.0f;
					if (runData.accMask < 1.0f)
					    runData.accMask = 1.0f;
				    }
				    //navUkfGpsPosUpate(&gps_data,dt,&state,&ukf_params);
				    // refine static sea level pressure based on better GPS altitude fixes
				    if (hAcc < runData.bestHacc && hAcc < NAV_MIN_GPS_ACC) {
				    	//UKFPressureAdjust((float)gps_data.alt * 1e3f);
				    	runData.bestHacc = hAcc;
				    }
				}
				if (timestamp_velocity < gps_data.timestamp_velocity) {
					timestamp_velocity = gps_data.timestamp_velocity;
				    //navUkfGpsVelUpate(&gps_data,dt,&state,&ukf_params);
				}
				perf_end(quat_pos_gps_perf);
			}
			// observe that the rates are exactly 0 if not flying or moving
			bool mightByFlying = !state.condition_landed;
			if (fds[2].revents & POLLIN)
			{
				orb_copy(ORB_ID(filtered_bottom_flow), flow_sub, &flow_data);
				if(!mightByFlying) {
						memset(&flow_data,0,sizeof(flow_data));
						navUkfFlowVelUpate(&flow_data,dt,&state,&ukf_params);
				}
				else {
					//printf("flowx:%8.4f\tflowy:%8.4f\n",flow_data.vx, flow_data.vy);
					navUkfFlowVelUpate(&flow_data,dt,&state,&ukf_params);
				}
			}
			if(dt < FLT_MIN) {
				perf_end(quat_pos_loop_perf);
				continue;
			}
			if (!(loopcounter % 100) && !mightByFlying ) {
			    static uint32_t axis = 0;
			    float stdX, stdY, stdZ;

			    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
			    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
			    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

			    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
			    	navUkfZeroRate(raw.gyro_rad_s[2], (axis++) % 3);
			    }
			}

			perf_begin(quat_pos_nav_perf);
			if(mightByFlying) {
				navNavigate(&gps_data,&state,&nav_params,&manual,&flow_data,raw.timestamp);
			}

			perf_end(quat_pos_nav_perf);

			// rotate nav's NE frame of reference to our craft's local frame of reference
			// Tilt north means for yaw=0 nose up. If yaw=90 degrees it means left wing up that is positive roll
			att_sp.pitch_body = (navData.holdTiltN * navUkfData.yawCos - navData.holdTiltE * navUkfData.yawSin) * DEG_TO_RAD;
			att_sp.roll_body  = (navData.holdTiltE * navUkfData.yawCos + navData.holdTiltN * navUkfData.yawSin) * DEG_TO_RAD;
			att_sp.thrust = pidUpdate(navData.altSpeedPID, navData.holdSpeedAlt, -UKF_VELD);
			att_sp.yaw_body = navData.holdHeading;
			att_sp.timestamp = hrt_absolute_time();

			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

			if (!(loopcounter % 10)) {
				global_position.alt = UKF_ALTITUDE;
				global_position.lat = UKF_POSN;
				global_position.lon = UKF_POSE;
				global_position.vel_n = UKF_VELN;
				global_position.vel_e = UKF_VELE;
				global_position.vel_d = UKF_VELD;
				global_position.timestamp = hrt_absolute_time();
				global_position.yaw = navUkfData.yaw;
				orb_publish(ORB_ID(vehicle_global_position), global_position_pub, &global_position);
			}
			perf_end(quat_pos_loop_perf);
			// print debug information every 1000th time
/*			printf("Pressure:%8.4f\t%8.4f\t%8.4f\n",
					raw.baro_pres_mbar, runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST),
					navUkfPresToAlt(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST)));*/

			if (debug == true && !(printcounter % 1000))
			{
				float frequence = 0;
				static uint32_t last_measure = 0;
				uint32_t current = hrt_absolute_time();
				frequence = 1000.0f*1000000.0f/(float)(current - last_measure);
				last_measure = current;
				printf("------\n");
				printf("veln:%8.4f\tvele:%8.4f\tveld:%8.4f\n"
						"posn:%8.4f\tpose:%8.4f\tposd:%8.4f\n"
						"holdSpeedN:%8.4f\tholdSpeedE:%8.4f\nholdTiltN:%8.4f\tholdTiltE:%8.4f\n"
						"accbx:%8.4f\taccby:%8.4f\taccbz:%8.4f\n"
						"gbybx:%8.4f\tgbyby:%8.4f\tgbybz:%8.4f\n"
						"q1:%8.4f\tq2:%8.4f\tq3:%8.4f\tq4:%8.4f\tpresalt:%8.4f\n",
					navUkfData.x[0],navUkfData.x[1],navUkfData.x[2],
					navUkfData.x[3],navUkfData.x[4],navUkfData.x[5],
					navData.holdSpeedN, navData.holdSpeedE, navData.holdTiltN, navData.holdTiltE,
					navUkfData.x[6],navUkfData.x[7],navUkfData.x[8],navUkfData.x[9],
					navUkfData.x[10],navUkfData.x[11],navUkfData.x[12],navUkfData.x[13],navUkfData.x[14],
					navUkfData.x[15],navUkfData.x[16]);
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
				//printf("GPS trust: hAcc:%8.4f dt:%8.4f\n", gps_data.eph_m, dt);
				//printf("GPS input: alt:%d lat:%d, lon:%d velE:%8.4f velN:%8.4f velD:%8.4f\n", gps_data.alt, gps_data.lat, gps_data.lon, gps_data.vel_e_m_s, gps_data.vel_n_m_s, gps_data.vel_d_m_s);

				//printf("Global position: alt:%8.4f lat:%8.4f lon:%8.4f vx:%8.4f vy:%8.4f vz:%8.4f yaw:%8.4f\n", global_position.alt, global_position.lat, global_position.lon, global_position.vx, global_position.vy, global_position.vz, global_position.hdg);
				/*float rotError[3];
				float estMag[3];
				float m[3*3];
				navUkfQuatToMatrix(m, &UKF_Q1, 1);
				// rotate mags to body frame of reference
				navUkfRotateVecByRevMatrix(estMag, navUkfData.v0m, m);
				// add in mag vector
				rotError[0] = -(raw.magnetometer_ga[2] * estMag[1] - estMag[2] * raw.magnetometer_ga[1]) * 0.50f;
				rotError[1] = -(raw.magnetometer_ga[0] * estMag[2] - estMag[0] * raw.magnetometer_ga[2]) * 0.50f;
				rotError[2] = -(raw.magnetometer_ga[1] * estMag[0] - estMag[1] * raw.magnetometer_ga[0]) * 0.50f;
				printf("Rot Error x: %8.4f\ty: %8.4f\tz:%8.4f\n", (double)rotError[0], (double)rotError[1], (double)rotError[2]);*/
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



