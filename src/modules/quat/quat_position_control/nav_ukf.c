/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#include "nav_ukf.h"
#include <systemlib/conversions.h>
#include <quat/utils/quat_constants.h>
#include "nav.h"
#include <quat/utils/aq_math.h>
#include <float.h>
#include <math.h>
#include <quat/utils/aq_math.h>
#include <quat/utils/util.h>
#include <quat/utils/compass_utils.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/filtered_bottom_flow.h>

navUkfStruct_t navUkfData __attribute__((section(".ccm")));

#ifdef UKF_LOG_BUF
char ukfLog[UKF_LOG_BUF];
#endif
void navUkfCalcEarthRadius(double lat);
float navUkfPresToAlt(float pressure);
void navUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast);
void navUkfResetPosition(float deltaN, float deltaE, float deltaD);
void navUkfNormalizeQuat(float *qr, float *q);
void crossVector3(float *vr, float *va, float *vb);
float dotVector3(float *va, float *vb);
void navUkfRotateVectorByQuat(float *vr, float *v, float *q);
void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);
void navUkfRotateVecByMatrix(float *vr, float *v, float *m);
void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll);
void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt);
void navUkfRateUpdate(float *u, float *x, float *noise, float *y);
void navUkfInitState(const struct sensor_combined_s* sensors);
void navUkfAccUpdate(float *u, float *x, float *noise, float *y);
void navUkfMagUpdate(float *u, float *x, float *noise, float *y);
void navUkfPresUpdate(float *u, float *x, float *noise, float *y);
void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y);
void navUkfPosUpdate(float *u, float *x, float *noise, float *y);
void navUkfVelUpdate(float *u, float *x, float *noise, float *y);
void navUkfFlowUpdate(float *u, float *x, float *noise, float *y);

bool isFlying(const struct vehicle_status_s *current_status){
	return (current_status->state_machine > SYSTEM_STATE_GROUND_READY);
}

float navUkfPresToAlt(float pressure) {
    //return (1.0f -  powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0f + 273.15f;	/* temperature at base height in Kelvin */
	const double a  = -6.5f / 1000.0f;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665f;	/* gravity constant in m/s/s */
	const double R  = 287.05f;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = 101325.0f / 1000.0f;

	/* measured pressure in kPa */
	double p = pressure / 10.0f;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	return (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
	//pow((p/101.325),0.000001902)
}

// reset current sea level static pressure based on better GPS estimate
void UKFPressureAdjust(float altitude) {
    navUkfData.presAltOffset = altitude - UKF_PRES_ALT;
}

void navUkfCalcEarthRadius(double lat) {
    float sinLat2;

    sinLat2 = sinf(lat * DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = NAV_EQUATORIAL_RADIUS * DEG_TO_RAD * (1.0f - NAV_E_2) / powf(1.0f - (NAV_E_2 * sinLat2), (3.0f / 2.0f));
    navUkfData.r2 = NAV_EQUATORIAL_RADIUS * DEG_TO_RAD / aq_sqrtf(1.0f - (NAV_E_2 * sinLat2));
}

void navUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * cosf(lat * DEG_TO_RAD) * navUkfData.r2;
}

void navUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
	navUkfData.posN[i] += deltaN;
	navUkfData.posE[i] += deltaE;
	navUkfData.posD[i] += deltaD;
    }

    UKF_POSN += deltaN;
    UKF_POSE += deltaE;
    UKF_POSD += deltaD;

//    UKF_PRES_BIAS += deltaD;

    navResetHoldAlt(deltaD);
}

void navUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcDistance(lat, lon, &oldPosN, &oldPosE);

    navUkfData.holdLat = lat;
    navUkfData.holdLon = lon;

    navUkfCalcDistance(lat, lon, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

void navUkfNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = aq_sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}

void navUkfNormalizeQuat(float *qr, float *q) {
    float norm;

    norm = aq_sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] = q[0] / norm;
    qr[1] = q[1] / norm;
    qr[2] = q[2] / norm;
    qr[3] = q[3] / norm;
}

void crossVector3(float *vr, float *va, float *vb) {
    vr[0] = va[1] * vb[2] - vb[1] * va[2];
    vr[1] = va[2] * vb[0] - vb[2] * va[0];
    vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb) {
    return va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2];
}

void navUkfRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0*y*w*v[2] - 2.0*z*w*v[1] + x*x*v[0] + 2.0*y*x*v[1] + 2.0*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0*x*y*v[0] + y*y*v[1] + 2.0*z*y*v[2] + 2.0*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0*x*z*v[0] + 2.0*y*z*v[1] + z*z*v[2] - 2.0*w*y*v[0] - y*y*v[2] + 2.0*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    navUkfRotateVectorByQuat(vr, v, qc);
}

void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length
    if (normalize)
	    invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
	    invs = 1.0f;

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0 * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0 * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0 * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0 * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0 * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0 * (tmp1 - tmp2) * invs;
}

void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll) {
    if (m[1*3+0] > 0.998f) { // singularity at north pole
	*pitch = atan2f(m[0*3+2], m[2*3+2]);
	*yaw = M_PI/2.0f;
	*roll = 0.0f;
    } else if (m[1*3+0] < -0.998f) { // singularity at south pole
	*pitch = atan2f(m[0*3+2] ,m[2*3+2]);
	*yaw = -M_PI/2.0f;
	*roll = 0.0f;
    }
    else {
	*pitch = atan2f(-m[2*3+0] ,m[0*3+0]);
	*yaw = asinf(m[1*3+0]);
	*roll = atan2f(-m[1*3+2], m[1*3+1]);
    }
}

void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;

    q0 = q[1];
    q1 = q[2];
    q2 = q[3];
    q3 = q[0];

    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
    float pitchProduct = - 2.0f * (q0 * q2 - q1 * q3);
    //The following is needed because the valid parameter range of asinf is [-1,1]
    float base = 0;
    if(pitchProduct > 1)
    {
    	pitchProduct -= 1;
    	base = M_PI;
    }
    else if(pitchProduct < -1)
    {
    	pitchProduct += 1;
    	base = - M_PI;
    }
    *pitch = asinf(pitchProduct) + base;
    *roll  = atan2f((2.0f * (q1 * q2 + q0 * q3)),-(1.0f-2.0f*(q3*q3 + q2*q2)));
/*
    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
    *pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
    *roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));*/
}

// result and source can be the same
void navUkfRotateQuat(float *qr, float *q, float *rate, float dt) {
    float q1[4];
    float s, t, lg;
    float qMag;

    s = aq_sqrtf(rate[0]*rate[0] + rate[1]*rate[1] + rate[2]*rate[2]) * 0.5f;
    if(s < FLT_MIN) {
        qr[0] = q[0];
        qr[1] = q[1];
        qr[2] = q[2];
        qr[3] = q[3];
        return;
    }
    t = -(0.5f * sinf(s) / s);
    rate[0] *= t;
    rate[1] *= t;
    rate[2] *= t;

    // create Lagrange factor to control quat's numerical integration errors
    qMag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    lg = cosf(s) + (1.0f - qMag*qMag) * dt * dt;

    // rotate
    q1[0] = q[0];
    q1[1] = q[1];
    q1[2] = q[2];
    q1[3] = q[3];

    qr[0] =  lg*q1[0]      + rate[0]*q1[1] + rate[1]*q1[2] + rate[2]*q1[3];
    qr[1] = -rate[0]*q1[0] + lg*q1[1]      - rate[2]*q1[2] + rate[1]*q1[3];
    qr[2] = -rate[1]*q1[0] + rate[2]*q1[1] + lg*q1[2]      - rate[0]*q1[3];
    qr[3] = -rate[2]*q1[0] - rate[1]*q1[1] + rate[0]*q1[2] + lg*q1[3];
}

void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];

    // acc bias
    out[6] = in[6] + noise[0] * dt;
    out[7] = in[7] + noise[1] * dt;
    out[8] = in[8] + noise[2] * dt;

    // gbias
    out[9]  = in[9]  + noise[3] * dt;
    out[10] = in[10] + noise[4] * dt;
    out[11] = in[11] + noise[5] * dt;

    // rate = rate + bias + noise
    rate[0] = (u[3] + out[9]  + noise[6]) * dt;
    rate[1] = (u[4] + out[10] + noise[7]) * dt;
    rate[2] = (u[5] + out[11] + noise[8]) * dt;

    // rotate
    navUkfRotateQuat(&out[12], &in[12], rate, dt);
    navUkfQuatToMatrix(mat3x3, &out[12], 1);

    // acc
    tmp[0] = u[0] + out[6];
    tmp[1] = u[1] + out[7];
    tmp[2] = u[2] + out[8];

    // rotate acc to world frame
    navUkfRotateVecByMatrix(acc, tmp, mat3x3);
    acc[2] += CONSTANTS_ONE_G;

    // vel
    out[0] = in[0] + acc[0] * dt + noise[10];
    out[1] = in[1] + acc[1] * dt + noise[11];
    out[2] = in[2] + acc[2] * dt + noise[12];

/*
    if(fabs(acc[0])> 0.2 || fabs(acc[1])> 0.2 || fabs(acc[2])> 0.2)
    {
    	printf("in1:%8.4f\t2:%8.4f\t3:%8.4f\n",
    	    			u[0],u[1],u[2]);
    	printf("acc1:%8.4f\t2:%8.4f\t3:%8.4f\n",
    	    	    			acc[0],acc[1],acc[2]);
    }
*/
    // pos
    out[3] = in[3] + (in[0] + out[0]) * 0.5f * dt + noise[13];
    out[4] = in[4] + (in[1] + out[1]) * 0.5f * dt + noise[14];
    out[5] = in[5] - (in[2] + out[2]) * 0.5f * dt + noise[15];

    // pres alt
    out[16] = in[16] - (in[2] + out[2]) * 0.5f * dt + noise[9];
}

void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[9+(int)u[0]] + noise[0];
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[12]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[12]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[16] + noise[0]; // return altitude
}

void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[16] + noise[0];// return pres altitude
    y[1] = x[5] + noise[1]; // return GPS altitude
}

void navUkfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[3] + noise[0]; // return position
    y[1] = x[4] + noise[1];
    y[2] = x[5] + noise[2];
}

void navUkfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[0] + noise[0]; // return velocity
    y[1] = x[1] + noise[1];
    y[2] = x[2] + noise[2];
}

void navUkfFlowUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[0] + noise[0]; // return velocity
    y[1] = x[1] + noise[1];
}

void navUkfFinish(void) {
	float yaw, pitch, roll;
    navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);
    navUkfQuatExtractEuler(&UKF_Q1, &yaw, &pitch, &roll);
    navUkfData.roll = roll;
    navUkfData.pitch = pitch;
    yaw = compassNormalizeRad(yaw);
    navUkfData.yaw = yaw;
    //navUkfData.pitch *= RAD_TO_DEG;
    //navUkfData.roll *= RAD_TO_DEG;

    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f
    navUkfData.yawCos = cosf(navUkfData.yaw);
    navUkfData.yawSin = sinf(navUkfData.yaw);
}

float navUkfInertialUpdate(const struct sensor_combined_s* raw) {

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

    srcdkfTimeUpdate(navUkfData.kf, u, dt);
    /*static int c = 0;
    printf("acc x:%8.4f\ty:%8.4f\tz:%8.4f\n",
    		u[0],u[1],u[2]);
    if(!(c++ % 20)) {
        matrixDump("Sigma Points", &navUkfData.kf->Xa);
    }*/
    // store history
    navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
    navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
    navUkfData.posD[navUkfData.navHistIndex] = UKF_POSD;

    navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
    navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
    navUkfData.velD[navUkfData.navHistIndex] = UKF_VELD;

    navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;
    return dt;
}

void navUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}

void simDoPresUpdate(float pres,
					 const struct vehicle_status_s *current_status,
					 const struct quat_position_control_UKF_params* params) {
    float noise[2];        // measurement variance
    float y[2];            // measurment(s)

    noise[0] = params->ukf_alt_n;
    noise[0] = params->ukf_alt_n;

    y[0] = navUkfPresToAlt(pres);
    y[1] = y[0];

    // if GPS altitude data has been available, only update pressure altitude
    if (fabsf(navUkfData.presAltOffset) > FLT_MIN) {
    	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 1, 1, noise, navUkfPresUpdate);
    }
      // otherwise update pressure and GPS altitude from the single pressure reading
    else {
    	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 2, 2, noise, navUkfPresGPSAltUpdate);
    }
}

void simDoAccUpdate(float accX, float accY, float accZ,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_ACC_BIAS_X;
    accY += UKF_ACC_BIAS_Y;
    accZ += UKF_ACC_BIAS_Z;

    // normalize vector
    norm =  aq_sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = params->ukf_acc_n + fabsf(CONSTANTS_ONE_G - norm) * params->ukf_dist_n;
    //printf("Noise:%8.4f\n",noise[0]);
    if (!isFlying(current_status)) {
    	accX -= UKF_ACC_BIAS_X; // TODO FL: There is no point about this code
    	accY -= UKF_ACC_BIAS_Y; // Result is not used
    	noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}

void simDoMagUpdate(float magX, float magY, float magZ,
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

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
}

void navUkfGpsPosUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params) {
    float y[3];
    float noise[3];
    float posDelta[3];
    int histIndex;
    if (dt < FLT_MIN) return;
    if (gps_position->eph_m < 4.0f && fabsf(gps_position->tDop) > FLT_MIN) {
		if (fabsf(navUkfData.holdLat) < FLT_MIN) {
			navUkfData.holdLat = gps_position->lat;
			navUkfData.holdLon = gps_position->lon;
			navUkfCalcEarthRadius(gps_position->lat);
			navUkfSetGlobalPositionTarget(gps_position->lat, gps_position->lon);
			navUkfResetPosition(-UKF_POSN, -UKF_POSE, gps_position->alt - UKF_POSD);
		}
		else {
			navUkfCalcDistance(gps_position->lat, gps_position->lon, &y[0], &y[1]);
			y[2] = gps_position->alt;

			// determine how far back this GPS position update came from
			histIndex = (hrt_absolute_time() - (gps_position->timestamp_position + params->ukf_pos_delay)) / (int)(1e6f * dt);
			histIndex = navUkfData.navHistIndex - histIndex;
			if (histIndex < 0)
			histIndex += UKF_HIST;
			if (histIndex < 0 || histIndex >= UKF_HIST)
			histIndex = 0;

			// calculate delta from current position
			posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
			posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
			posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];

			// set current position state to historic data
			UKF_POSN = navUkfData.posN[histIndex];
			UKF_POSE = navUkfData.posE[histIndex];
			UKF_POSD = navUkfData.posD[histIndex];

			noise[0] = params->ukf_gps_pos_n + gps_position->eph_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->nDop*gps_position->nDop) * params->ukf_gps_pos_m_n;
			noise[1] = params->ukf_gps_pos_n + gps_position->eph_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->eDop*gps_position->eDop) * params->ukf_gps_pos_m_n;
			noise[2] = params->ukf_gps_alt_n + gps_position->epv_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->vDop*gps_position->vDop) * params->ukf_gps_alt_m_n;

			srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);

			// add the historic position delta back to the current state
			UKF_POSN += posDelta[0];
			UKF_POSE += posDelta[1];
			UKF_POSD += posDelta[2];

	#ifdef UKF_LOG_BUF
		{
			float *log = (float *)&ukfLog[navUkfData.logPointer];

			*(uint32_t *)&log[0] = 0xffffffff;
			log[1] = y[0];
			log[2] = y[1];
			log[3] = y[2];
			log[4] = noise[0];
			log[5] = noise[1];
			log[6] = noise[2];

			navUkfData.logPointer = (navUkfData.logPointer + 7*sizeof(float)) % UKF_LOG_BUF;
			filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
		}
	#endif
		}
    }
    else {
		y[0] = 0.0f;
		y[1] = 0.0f;
		y[2] = UKF_PRES_ALT;

		if (isFlying(current_status)) {
			noise[0] = 1e1f;
			noise[1] = 1e1f;
			noise[2] = 1e2f;
		}
		else {
			noise[0] = 1e-7f;
			noise[1] = 1e-7f;
			noise[2] = 1e2f;
	}

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
    }
}

void navUkfGpsVelUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params) {
	// Don't do anything for invalid dt
	if(dt < FLT_MIN) return;
    float y[3];
    float noise[3];
    float velDelta[3];
    int histIndex;

    if (gps_position->sAcc < 2.0f && fabsf(gps_position->tDop) > FLT_MIN) {
	y[0] = gps_position->vel_n_m_s;
	y[1] = gps_position->vel_e_m_s;
	y[2] = gps_position->vel_d_m_s;

	// determine how far back this GPS velocity update came from
	histIndex = (hrt_absolute_time() - (gps_position->timestamp_velocity + params->ukf_vel_delay)) / (int)(1e6f * dt);
	histIndex = navUkfData.navHistIndex - histIndex;
	if (histIndex < 0)
	    histIndex += UKF_HIST;
	if (histIndex < 0 || histIndex >= UKF_HIST)
	    histIndex = 0;

	// calculate delta from current position
	velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
	velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
	velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];

	// set current position state to historic data
	UKF_VELN = navUkfData.velN[histIndex];
	UKF_VELE = navUkfData.velE[histIndex];
	UKF_VELD = navUkfData.velD[histIndex];

	noise[0] = params->ukf_gps_vel_n + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->nDop*gps_position->nDop) * params->ukf_gps_vel_m_n;
	noise[1] = params->ukf_gps_vel_n + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->eDop*gps_position->eDop) * params->ukf_gps_vel_m_n;
	noise[2] = params->ukf_gps_vd_n  + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->vDop*gps_position->vDop) * params->ukf_gps_vd_m_n;

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);

	// add the historic position delta back to the current state
	UKF_VELN += velDelta[0];
	UKF_VELE += velDelta[1];
	UKF_VELD += velDelta[2];

#ifdef UKF_LOG_BUF
	{
	    float *log = (float *)&ukfLog[navUkfData.logPointer];

	    *(uint32_t *)&log[0] = 0xffffffff;
	    log[1] = y[0];
	    log[2] = y[1];
	    log[3] = y[2];
	    log[4] = noise[0];
	    log[5] = noise[1];
	    log[6] = noise[2];

	    navUkfData.logPointer = (navUkfData.logPointer + 7*sizeof(float)) % UKF_LOG_BUF;
	    filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
	}
#endif
    }
    else {
	y[0] = 0.0f;
	y[1] = 0.0f;
	y[2] = UKF_PRES_ALT;

	if (isFlying(current_status)) {
	    noise[0] = 5.0f;
	    noise[1] = 5.0f;
	    noise[2] = 2.0f;
	}
	else {
	    noise[0] = 1e-7f;
	    noise[1] = 1e-7f;
	    noise[2] = 1e2f;
	}

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
    }
}

void navUkfFlowVelUpate(
		const struct filtered_bottom_flow_s* measured_flow,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params) {
	// Don't do anything for invalid dt
	if(dt < FLT_MIN) return;
    float y[2];
    float noise[2];
    float velDelta[3];

	// rotate to earth frame
    y[0] = measured_flow->vx * navUkfData.yawCos - measured_flow->vy * navUkfData.yawSin; //North
    y[1] = measured_flow->vy * navUkfData.yawCos + measured_flow->vx * navUkfData.yawSin; //East


	noise[0] = params->ukf_flow_vel_n;
	noise[1] = noise[0];

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 2, 2, noise, navUkfFlowUpdate);
}

void navUkfInitState(const struct sensor_combined_s* sensors) {

    // vel
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;

    // pos
    UKF_POSN = 0.0f;
    UKF_POSE = 0.0f;
    UKF_POSD = navUkfPresToAlt(sensors->baro_pres_mbar);

    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;

    // quat
    UKF_Q1 =  1.0f;
    UKF_Q2 =  0.0f;
    UKF_Q3 =  0.0f;
    UKF_Q4 =  0.0f;

    UKF_PRES_ALT = UKF_POSD;
}

void navUkfInit(const struct quat_position_control_UKF_params* params,
				const struct sensor_combined_s* sensors) {
    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];

    memset((void *)&navUkfData, 0, sizeof(navUkfData));

    navUkfData.v0a[0] = 0.0f;
    navUkfData.v0a[1] = 0.0f;
    navUkfData.v0a[2] = -1.0f;

    // calculate mag vector based on inclination
    mag[0] = cosf(COMPASS_INCLINATION * DEG_TO_RAD);
    mag[1] = 0.0f;
    mag[2] = -sinf(COMPASS_INCLINATION * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    navUkfData.v0m[0] = mag[0] * cosf(COMPASS_DECLINATION * DEG_TO_RAD) - mag[1] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navUkfData.v0m[1] = mag[1] * cosf(COMPASS_DECLINATION  * DEG_TO_RAD) + mag[0] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navUkfData.v0m[2] = mag[2];

    navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);

    navUkfData.x = srcdkfGetState(navUkfData.kf);

    // State variance
    Q[0] = params->ukf_vel_q;
    Q[1] = params->ukf_vel_q;
    Q[2] = params->ukf_vel_alt_q;
    Q[3] = params->ukf_pos_q;
    Q[4] = params->ukf_pos_q;
    Q[5] = params->ukf_pos_alt_q;
    Q[6] = params->ukf_acc_bias_q;
    Q[7] = params->ukf_acc_bias_q;
    Q[8] = params->ukf_acc_bias_q;
    Q[9] = params->ukf_gyo_bias_q;
    Q[10] = params->ukf_gyo_bias_q;
    Q[11] = params->ukf_gyo_bias_q;
    Q[12] = params->ukf_quat_q;
    Q[13] = params->ukf_quat_q;
    Q[14] = params->ukf_quat_q;
    Q[15] = params->ukf_quat_q;
    Q[16] = params->ukf_pres_alt_q;

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
    V[13] = params->ukf_pos_v;
    V[14] = params->ukf_pos_v;
    V[15] = params->ukf_alt_pos_v;

    srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

    navUkfInitState(sensors);

#ifdef UKF_LOG_BUF
    navUkfData.logHandle = filerGetHandle(UKF_FNAME);
    filerStream(navUkfData.logHandle, ukfLog, UKF_LOG_BUF);
#endif
}
