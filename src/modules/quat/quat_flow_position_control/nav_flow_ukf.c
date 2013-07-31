

#include "nav_flow_ukf.h"
#include <systemlib/conversions.h>
#include <quat/utils/quat_constants.h>
#include "nav_flow.h"
#include <float.h>
#include <math.h>
#include <quat/utils/aq_math.h>
#include <quat/utils/util.h>
#include <quat/utils/compass_utils.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/filtered_bottom_flow.h>

navFlowUkfStruct_t navFlowUkfData __attribute__((section(".ccm")));

void navFlowUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt);
void navFlowUkfRateUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfInitState(const struct sensor_combined_s* sensors);
void navFlowUkfAccUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfMagUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfPresUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfFlowUpdate(float *u, float *x, float *noise, float *y);

bool isFlying(const struct vehicle_status_s *current_status){
	return (current_status->state_machine > SYSTEM_STATE_GROUND_READY);
}

void navFlowUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];

    // acc bias
    out[3] = in[3] + noise[0] * dt;
    out[4] = in[4] + noise[1] * dt;
    out[5] = in[5] + noise[2] * dt;

    // gbias
    out[6]  = in[6]  + noise[3] * dt;
    out[7] = in[7] + noise[4] * dt;
    out[8] = in[8] + noise[5] * dt;

    // rate = rate + bias + noise
    rate[0] = (u[3] + out[6]  + noise[6]) * dt;
    rate[1] = (u[4] + out[7] + noise[7]) * dt;
    rate[2] = (u[5] + out[8] + noise[8]) * dt;

    // rotate
    utilRotateQuat(&out[9], &in[9], rate, dt);
    utilQuatToMatrix(mat3x3, &out[9], 1);

    // acc
    tmp[0] = u[0] + out[3];
    tmp[1] = u[1] + out[4];
    tmp[2] = u[2] + out[5];

    // rotate acc to world frame
    utilRotateVecByMatrix(acc, tmp, mat3x3);
    acc[2] += CONSTANTS_ONE_G;

    // vel
    out[0] = in[0] + acc[0] * dt + noise[10];
    out[1] = in[1] + acc[1] * dt + noise[11];
    out[2] = in[2] + acc[2] * dt + noise[12];

    // pres alt
    out[13] = in[13] - (in[2] + out[2]) * 0.5f * dt + noise[9];
}

void navFlowUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[6+(int)u[0]] + noise[0];
}

void navFlowUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    utilRotateVectorByRevQuat(y, navFlowUkfData.v0a, &x[9]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navFlowUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    utilRotateVectorByRevQuat(y, navFlowUkfData.v0m, &x[9]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navFlowUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[13] + noise[0]; // return altitude
}


void navFlowUkfFlowUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[0] + noise[0]; // return velocity
    y[1] = x[1] + noise[1];
}

void navFlowUkfFinish(void) {
	float yaw, pitch, roll;
    utilNormalizeQuat(&UKF_FLOW_Q1, &UKF_FLOW_Q1);
    utilQuatExtractEuler(&UKF_FLOW_Q1, &yaw, &pitch, &roll);
    navFlowUkfData.roll = roll;
    navFlowUkfData.pitch = pitch;
    yaw = compassNormalizeRad(yaw);
    navFlowUkfData.yaw = yaw;

    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f
    //navFlowUkfData.yawCos = cosf(navFlowUkfData.yaw);
    //navFlowUkfData.yawSin = sinf(navFlowUkfData.yaw);
}

float navFlowUkfInertialUpdate(const struct sensor_combined_s* raw) {

	/* Calculate data time difference in seconds */
	static uint64_t last_inertialUpdate = 0;
	float dt = 0;
	if(last_inertialUpdate != 0) {
		dt = ((float)(raw->timestamp - last_inertialUpdate)) / 1e6f;
	}
	last_inertialUpdate = raw->timestamp;
	if(dt < FLT_MIN) return 0.0f;

    float u[6];

    u[0] = raw->accelerometer_m_s2[0];
    u[1] = raw->accelerometer_m_s2[1];
    u[2] = raw->accelerometer_m_s2[2];

    u[3] = raw->gyro_rad_s[0];
    u[4] = raw->gyro_rad_s[1];
    u[5] = raw->gyro_rad_s[2];

    srcdkfTimeUpdate(navFlowUkfData.kf, u, dt);
    return dt;
}

void navFlowUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navFlowUkfData.kf, u, y, 1, 1, noise, navFlowUkfRateUpdate);
}

void navFlowDoPresUpdate(float pres,
					 const struct vehicle_status_s *current_status,
					 const struct quat_position_control_UKF_params* params) {
    float noise[2];        // measurement variance
    float y[2];            // measurment(s)

    noise[0] = params->ukf_alt_n;
    noise[0] = params->ukf_alt_n;

    y[0] = utilPresToAlt(pres);
    y[1] = y[0];

   	srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 1, 1, noise, navFlowUkfPresUpdate);
}

void navFlowDoAccUpdate(float accX, float accY, float accZ,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_FLOW_ACC_BIAS_X;
    accY += UKF_FLOW_ACC_BIAS_Y;
    accZ += UKF_FLOW_ACC_BIAS_Z;

    // normalize vector
    norm =  aq_sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = params->ukf_acc_n + fabsf(CONSTANTS_ONE_G - norm) * params->ukf_dist_n;
    //printf("Noise:%8.4f\n",noise[0]);
    if (!isFlying(current_status)) {
    	noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfAccUpdate);
}

void navFlowDoMagUpdate(float magX, float magY, float magZ,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = params->ukf_mag_n;

    if (!isFlying(current_status))
	noise[0] = 0.001f;

    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / aq_sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfMagUpdate);
}


void navFlowUkfFlowVelUpate(
		const struct filtered_bottom_flow_s* measured_flow,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params) {
	// Don't do anything for invalid dt
	if(dt < FLT_MIN) return;
    float y[2];
    float noise[2];

	// rotate to earth frame
    y[0] = measured_flow->vx;
    y[1] = measured_flow->vy;


	noise[0] = params->ukf_flow_vel_n;
	noise[1] = noise[0];

	srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfFlowUpdate);
}

void navFlowUkfInitState(const struct sensor_combined_s* sensors) {

    // vel
    UKF_FLOW_VELX = 0.0f;
    UKF_FLOW_VELY = 0.0f;
    UKF_FLOW_VELD = 0.0f;

    // acc bias
    UKF_FLOW_ACC_BIAS_X = 0.0f;
    UKF_FLOW_ACC_BIAS_Y = 0.0f;
    UKF_FLOW_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_FLOW_GYO_BIAS_X = 0.0f;
    UKF_FLOW_GYO_BIAS_Y = 0.0f;
    UKF_FLOW_GYO_BIAS_Z = 0.0f;

    // quat
    UKF_FLOW_Q1 =  1.0f;
    UKF_FLOW_Q2 =  0.0f;
    UKF_FLOW_Q3 =  0.0f;
    UKF_FLOW_Q4 =  0.0f;

    UKF_FLOW_PRES_ALT = 0.0f;
}

void navFlowUkfInit(const struct quat_position_control_UKF_params* params,
				const struct sensor_combined_s* sensors) {
    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];

    memset((void *)&navFlowUkfData, 0, sizeof(UKF_FLOW_Q1));

    navFlowUkfData.v0a[0] = 0.0f;
    navFlowUkfData.v0a[1] = 0.0f;
    navFlowUkfData.v0a[2] = -1.0f;

    // calculate mag vector based on inclination
    mag[0] = cosf(COMPASS_INCLINATION * DEG_TO_RAD);
    mag[1] = 0.0f;
    mag[2] = -sinf(COMPASS_INCLINATION * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    navFlowUkfData.v0m[0] = mag[0] * cosf(COMPASS_DECLINATION * DEG_TO_RAD) - mag[1] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navFlowUkfData.v0m[1] = mag[1] * cosf(COMPASS_DECLINATION  * DEG_TO_RAD) + mag[0] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navFlowUkfData.v0m[2] = mag[2];

    navFlowUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navFlowUkfTimeUpdate);

    navFlowUkfData.x = srcdkfGetState(navFlowUkfData.kf);

    // State variance
    Q[0] = params->ukf_vel_q;
    Q[1] = params->ukf_vel_q;
    Q[2] = params->ukf_vel_alt_q;
    Q[3] = params->ukf_acc_bias_q;
    Q[4] = params->ukf_acc_bias_q;
    Q[5] = params->ukf_acc_bias_q;
    Q[6] = params->ukf_gyo_bias_q;
    Q[7] = params->ukf_gyo_bias_q;
    Q[8] = params->ukf_gyo_bias_q;
    Q[9] = params->ukf_quat_q;
    Q[10] = params->ukf_quat_q;
    Q[11] = params->ukf_quat_q;
    Q[12] = params->ukf_quat_q;
    Q[13] = params->ukf_pres_alt_q;

    // Process noise
    V[0] = params->ukf_acc_bias_v;
    V[1] = params->ukf_acc_bias_v;
    V[2] = params->ukf_acc_bias_v;
    V[3] = params->ukf_gyo_bias_v;
    V[4] = params->ukf_gyo_bias_v;
    V[5] = params->ukf_gyo_bias_v;
    V[6] = params->ukf_rate_v;
    V[7] = params->ukf_gyo_bias_v;
    V[8] = params->ukf_gyo_bias_v;
    V[9] = params->ukf_pres_alt_v;
    V[10] = params->ukf_vel_v;
    V[11] = params->ukf_vel_v;
    V[12] = params->ukf_alt_vel_v;

    srcdkfSetVariance(navFlowUkfData.kf, Q, V, 0, 0);

    navFlowUkfInitState(sensors);

}