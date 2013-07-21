
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
#include <quat/quat_position_control/quat_pos_control_params.h>
#include "nav_flow.h"
#include "nav_flow_ukf.h"
#include <quat/utils/quat_constants.h>

#include "quat_flow_pos_control.h"

runStruct_t runData __attribute__((section(".ccm")));
// Struct for data output. Defined here to reduce stack frame size
static struct vehicle_attitude_setpoint_s att_sp __attribute__((section(".ccm")));
static struct vehicle_attitude_s att __attribute__((section(".ccm")));
static struct manual_control_setpoint_s manual;
static struct vehicle_status_s state;
static struct sensor_combined_s raw;
static struct filtered_bottom_flow_s flow_data;

static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */
static bool debug = false;

__EXPORT int quat_flow_pos_control_main(int argc, char *argv[]);

/**
 * Mainloop of position controller.
 */
static int quat_flow_pos_control_thread_main(int argc, char *argv[]);

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
		deamon_task = task_spawn("quat flow pos control",
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

	// Output
	// Calculation result is the attitude setpoint
	/* publish attitude setpoint */
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	memset(&att_sp, 0, sizeof(att_sp));

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
	perf_counter_t quat_flow_pos_loop_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_control");
	perf_counter_t quat_flow_pos_sensor_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_sensor_control");
	perf_counter_t quat_flow_pos_inertial_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_inertial_control");
	perf_counter_t quat_flow_pos_nav_perf = perf_alloc(PC_ELAPSED, "quat_flow_pos_nav_control");

	struct pollfd fds[5] = {
		{ .fd = sub_raw,   .events = POLLIN },
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
	printf("[quat flow pos control] Init position control\n");
	quat_flow_pos_runInit(&raw);
	printf("[quat flow pos control] Init ukf\n");
	navFlowUkfInit(&ukf_params,&raw);
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

				navFlowUkfNormalizeVec3(acc, acc);
				navFlowUkfNormalizeVec3(mag, mag);

				navFlowUkfQuatToMatrix(m, &UKF_FLOW_Q1, 1);

				// rotate gravity to body frame of reference
				navFlowUkfRotateVecByRevMatrix(estAcc, navFlowUkfData.v0a, m);

				// rotate mags to body frame of reference
				navFlowUkfRotateVecByRevMatrix(estMag, navFlowUkfData.v0m, m);

				// measured error
				rotError[0] = -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 0.50f;
				rotError[1] = -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 0.50f;
				rotError[2] = -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 0.50f;
				//printf("[quat flow pos control] Estmag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)estMag[0], (double)estMag[1], (double)estMag[2]);
				//printf("[quat flow pos control] Mag X: %8.4f\t Y: %8.4f\t Z: %8.4f\n", (double)mag[0], (double)mag[1], (double)mag[2]);
				//printf("[quat flow pos control] Mag error errorX: %8.4f\t errorY: %8.4f\t errorZ: %8.4f\n", (double)rotError[0], (double)rotError[1], (double)rotError[2]);


				rotError[0] += -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
				rotError[1] += -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
				rotError[2] += -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;


			    navFlowUkfRotateQuat(&UKF_FLOW_Q1, &UKF_FLOW_Q1, rotError, 0.1f);

				if (l >= UKF_GYO_AVG_NUM) {
				    arm_std_f32(gyX, UKF_GYO_AVG_NUM, &stdX);
				    arm_std_f32(gyY, UKF_GYO_AVG_NUM, &stdY);
				    arm_std_f32(gyZ, UKF_GYO_AVG_NUM, &stdZ);
				}
				float std = stdX + stdY + stdZ;
				printf("[quat flow pos control] Init %d:\t std: %8.4f\t errorX: %8.4f\t errorY: %8.4f\t errorZ: %8.4f\n", l, (double)std, (double)rotError[0], (double)rotError[1], (double)rotError[2]);

				l++;
			    if (l > UKF_GYO_AVG_NUM*5 && std < 0.004f) {
			    	initCompleted = true;
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

			}
		}
	}
	printf("[quat flow pos control] Init nav\n");
	navFlowInit(&nav_params,raw.baro_alt_meter,navFlowUkfData.yaw);
	printf("[quat flow pos control] Starting loop\n");

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
			/* only update parameters if state changed */
			bool updated = false;
			orb_check(fds[4].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(vehicle_status), state_sub, &state);
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
			}
			orb_check(fds[2].fd, &updated);
			if (updated)
			{
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
			}
			if (fds[0].revents & POLLIN)
			{
				perf_begin(quat_flow_pos_inertial_perf);
				orb_copy(ORB_ID(sensor_combined), sub_raw, &raw);
				// raw parameter changed
				dt = navFlowUkfInertialUpdate(&raw);
				perf_end(quat_flow_pos_inertial_perf);
				if(dt < FLT_MIN) {
					perf_end(quat_flow_pos_loop_perf);
					continue;
				}

				perf_begin(quat_flow_pos_sensor_perf);
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
				   navFlowDoAccUpdate(	runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST),
						   	   	   	&state,
						   	   	   	&ukf_params);
				}
				else if (!((loopcounter+7) % 20)) {
					navFlowDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST),
				   	   	   			&state,
				   	   	   			&ukf_params);
				}
				//else if (!((loopcounter+14) % 20)) {
				navFlowDoMagUpdate(runData.sumMag[0]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[1]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  runData.sumMag[2]*(1.0 / (float)RUN_SENSOR_HIST),
						   	   	  &state,
						   	   	  &ukf_params);
				//}
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
				orb_publish(ORB_ID(vehicle_attitude), pub_att, &att);
				perf_end(quat_flow_pos_sensor_perf);
			}
			// observe that the rates are exactly 0 if not flying or moving
			bool mightByFlying = state.flag_system_armed;
			if (fds[1].revents & POLLIN)
			{
				orb_copy(ORB_ID(filtered_bottom_flow), flow_sub, &flow_data);
				if(!mightByFlying) {
						memset(&flow_data,0,sizeof(flow_data));
						navFlowUkfFlowVelUpate(&flow_data,dt,&state,&ukf_params);
				}
				else {
					//printf("flowx:%8.4f\tflowy:%8.4f\n",flow_data.vx, flow_data.vy);
					navFlowUkfFlowVelUpate(&flow_data,dt,&state,&ukf_params);
				}
			}
			if(dt < FLT_MIN) {
				perf_end(quat_flow_pos_loop_perf);
				continue;
			}
			if (!(loopcounter % 100) && !mightByFlying ) {
			    static uint32_t axis = 0;
			    float stdX, stdY, stdZ;

			    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
			    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
			    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

			    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
			    	navFlowUkfZeroRate(raw.gyro_rad_s[2], (axis++) % 3);
			    }
			}

			perf_begin(quat_flow_pos_nav_perf);
			if(mightByFlying) {
				navFlowNavigate(&state,&nav_params,&manual,&flow_data,raw.timestamp);
			}

			perf_end(quat_flow_pos_nav_perf);

			// rotate nav's NE frame of reference to our craft's local frame of reference
			// Tilt north means for yaw=0 nose up. If yaw=90 degrees it means left wing up that is positive roll
			att_sp.pitch_body = navFlowData.holdTiltX * DEG_TO_RAD;
			att_sp.roll_body  = navFlowData.holdTiltY * DEG_TO_RAD;
			att_sp.thrust = pidUpdate(navFlowData.altSpeedPID, navFlowData.holdSpeedAlt, -UKF_FLOW_VELD);
			att_sp.yaw_body = navFlowData.holdHeading;
			att_sp.timestamp = hrt_absolute_time();

			orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
			perf_end(quat_flow_pos_loop_perf);
			// print debug information every 1000th time

			if (debug == true && !(printcounter % 100))
			{
				float frequence = 0;
				static uint32_t last_measure = 0;
				uint32_t current = hrt_absolute_time();
				frequence = 1000.0f*1000000.0f/(float)(current - last_measure);
				last_measure = current;
				printf("------\n");
				printf("velx:%8.4f\tvely:%8.4f\tveld:%8.4f\n"
						"holdSpeedX:%8.4f\tholdSpeedY:%8.4f\tholdSpeedAlt:%8.4f\tTargetHoldSpeedAlt:%8.4f\n"
						"holdTiltX:%8.4f\tholdTiltY:%8.4f\tholdThrust:%8.4f\n"
						"accbx:%8.4f\taccby:%8.4f\taccbz:%8.4f\n"
						"gbybx:%8.4f\tgbyby:%8.4f\tgbybz:%8.4f\n"
						"q1:%8.4f\tq2:%8.4f\tq3:%8.4f\tq4:%8.4f\tpresalt:%8.4f\tholdsalt:%8.4f\n",
					navFlowUkfData.x[0],navFlowUkfData.x[1],navFlowUkfData.x[2],
					navFlowData.holdSpeedX, navFlowData.holdSpeedY, navFlowData.holdSpeedAlt, navFlowData.targetHoldSpeedAlt,
					navFlowData.holdTiltX, navFlowData.holdTiltY, att_sp.thrust,
					navFlowUkfData.x[3],navFlowUkfData.x[4],navFlowUkfData.x[5],navFlowUkfData.x[6],
					navFlowUkfData.x[7],navFlowUkfData.x[8],navFlowUkfData.x[9],navFlowUkfData.x[10],navFlowUkfData.x[11],
					navFlowUkfData.x[12],navFlowUkfData.x[13],navFlowData.holdAlt);
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
						manual.throttle,navFlowData.navCapable);
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



