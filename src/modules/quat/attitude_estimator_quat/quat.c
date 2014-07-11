
#include "quat.h"
#include <quat/utils/compass_utils.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <geo/geo.h>
#include <systemlib/conversions.h>

#include "quat/utils/quat_constants.h"
#include "attitude_estimator_quat_params.h"
#include "aq.h"
#define COMPASS_INCLINATION		-64.132470167481f

quatStruct_t quatData;

const quat_real_t v0a[3] = {0.0f, 0.0f, -1.0f};

void normalizeVector3(quat_real_t *v) {
    quat_real_t mag;

    mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (mag == 0)return;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
}

void normalizeVector4(quat_real_t *v) {
    quat_real_t mag;

    mag = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
    if (mag == 0)return;
    v[0] /= mag;
    v[1] /= mag;
    v[2] /= mag;
    v[3] /= mag;
}

void rotateVecByMatrix(quat_real_t *vr, const quat_real_t * const v, quat_real_t m[3][3]) {
    vr[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    vr[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    vr[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}

void rotateVecByRevMatrix(quat_real_t *vr, const quat_real_t * const v, quat_real_t m[3][3]) {
    vr[0] = m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2];
    vr[1] = m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2];
    vr[2] = m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2];
}

// no need to normalize as our quat stays very close to norm
void quatToMatrix(quat_real_t m[3][3], const quat_real_t * const q) {
    quat_real_t tmp1, tmp2;
    quat_real_t sqw = q[0]*q[0];
    quat_real_t sqx = q[1]*q[1];
    quat_real_t sqy = q[2]*q[2];
    quat_real_t sqz = q[3]*q[3];

    // get the invert square length
    //	quat_real_t invs = 1.0f / (sqx + sqy + sqz + sqw);

    // rotation matrix is scaled by inverse square length
    m[0][0] = ( sqx - sqy - sqz + sqw); // * invs;
    m[1][1] = (-sqx + sqy - sqz + sqw); // * invs;
    m[2][2] = (-sqx - sqy + sqz + sqw); // * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1][0] = 2.0f * (tmp1 + tmp2); // * invs;
    m[0][1] = 2.0f * (tmp1 - tmp2); // * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2][0] = 2.0f * (tmp1 - tmp2); // * invs;
    m[0][2] = 2.0f * (tmp1 + tmp2); // * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2][1] = 2.0f * (tmp1 + tmp2); // * invs;
    m[1][2] = 2.0f * (tmp1 - tmp2); // * invs;
}

void quatExtractEuler(const quat_real_t *q, quat_real_t *yaw, quat_real_t *pitch, quat_real_t *roll) {
    quat_real_t q0, q1, q2, q3;

    q0 = q[1];
    q1 = q[2];
    q2 = q[3];
    q3 = q[0];

    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (1.0f-2.0f*(q2*q2 + q1*q1)));
    quat_real_t pitchProduct = 2.0f * (q0 * q2 - q1 * q3);
    //The following is needed because the valid parameter range of asinf is [-1,1]
    quat_real_t base = 0;
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
    *roll = atan2f((2.0f * (q1 * q2 + q0 * q3)),-(1.0f-2.0f*(q3*q3 + q2*q2)));
}

// result and source can be the same
void rotateQuat(quat_real_t *qr,
				const quat_real_t const *q,
				const quat_real_t const *rate,
				const quat_real_t dt) {
    quat_real_t q1[4];
    quat_real_t s, t, lg;
    quat_real_t qMag;
    quat_real_t lrate[3];

    lrate[0] = rate[0];
    lrate[1] = rate[1];
    lrate[2] = rate[2];

    s = sqrtf(rate[0]*rate[0] + rate[1]*rate[1] + rate[2]*rate[2]) * 0.5f;
    if(s==0.0f)
    {
    	t = 1;
    }
    else
    {
    	t = -(0.5f * sinf(s) / s);
    }
    lrate[0] *= t;
    lrate[1] *= t;
    lrate[2] *= t;

    // create Lagrange factor to control quat's numerical integration errors
    qMag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    lg = cosf(s) + (1.0f - qMag*qMag) * dt * dt;

    // rotate
    q1[0] = q[0];
    q1[1] = q[1];
    q1[2] = q[2];
    q1[3] = q[3];

    qr[0] =  lg*q1[0]      + lrate[0]*q1[1] + lrate[1]*q1[2] + lrate[2]*q1[3];
    qr[1] = -lrate[0]*q1[0] + lg*q1[1]      - lrate[2]*q1[2] + lrate[1]*q1[3];
    qr[2] = -lrate[1]*q1[0] + lrate[2]*q1[1] + lg*q1[2]      - lrate[0]*q1[3];
    qr[3] = -lrate[2]*q1[0] - lrate[1]*q1[1] + lrate[0]*q1[2] + lg*q1[3];

}

void quatMatrixMultiply(quat_real_t mr[3][3], const quat_real_t ma[3][3], const quat_real_t mb[3][3]) {
    mr[0][0] = ma[0][0]*mb[0][0] + ma[0][1]*mb[1][0] + ma[0][2]*mb[2][0];
    mr[0][1] = ma[0][0]*mb[0][1] + ma[0][1]*mb[1][1] + ma[0][2]*mb[2][1];
    mr[0][2] = ma[0][0]*mb[0][2] + ma[0][1]*mb[1][2] + ma[0][2]*mb[2][2];

    mr[1][0] = ma[1][0]*mb[0][0] + ma[1][1]*mb[1][0] + ma[1][2]*mb[2][0];
    mr[1][1] = ma[1][0]*mb[0][1] + ma[1][1]*mb[1][1] + ma[1][2]*mb[2][1];
    mr[1][2] = ma[1][0]*mb[0][2] + ma[1][1]*mb[1][2] + ma[1][2]*mb[2][2];

    mr[2][0] = ma[2][0]*mb[0][0] + ma[2][1]*mb[1][0] + ma[2][2]*mb[2][0];
    mr[2][1] = ma[2][0]*mb[0][1] + ma[2][1]*mb[1][1] + ma[2][2]*mb[2][1];
    mr[2][2] = ma[2][0]*mb[0][2] + ma[2][1]*mb[1][2] + ma[2][2]*mb[2][2];
}

void quatMatrixTranspose(quat_real_t mt[3][3], const quat_real_t m[3][3]) {
    mt[0][0] = m[0][0];
    mt[0][1] = m[1][0];
    mt[0][2] = m[2][0];

    mt[1][0] = m[0][1];
    mt[1][1] = m[1][1];
    mt[1][2] = m[2][1];

    mt[2][0] = m[0][2];
    mt[2][1] = m[1][2];
    mt[2][2] = m[2][2];
}

/*
void quatCode(void *p) {
    AQ_NOTICE("Quat task started...\n");

    while (1) {
	ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &publicEvents, AQ_RATE_RDY, CTL_TIMEOUT_NONE, 0);

	quatUpdate();

	// notify world of new data
	ctl_events_pulse(&publicEvents, AQ_SOL_RDY|AQ_TELEM_RDY);
    }
}
*/

void quatInit(void) {
    //unsigned long lastUpdate; //TODO FL uncommented
    float x, y, z;
    unsigned int i;

	//sleep 1000 ms
	usleep(1000 * 1000);

    // calculate mag vector based on inclination
    x = cosf(COMPASS_INCLINATION * DEG_TO_RAD);
    y = 0.0f;
    z = sinf(COMPASS_INCLINATION * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    quatData.v0m[0] = x * cosf(COMPASS_DECLINATION * DEG_TO_RAD) - y * sinf(COMPASS_DECLINATION * DEG_TO_RAD);
    quatData.v0m[1] = y * cosf(COMPASS_DECLINATION * DEG_TO_RAD * DEG_TO_RAD) + x * sinf(COMPASS_DECLINATION * DEG_TO_RAD);
    quatData.v0m[2] = z;
    normalizeVector3(quatData.v0m);

    // lazy way to initiate quat to our current orientation
    quatData.q[0] = 1.0f;
    quatData.q[1] = 0.0f;
    quatData.q[2] = 0.0f;
    quatData.q[3] = 0.0f;
    quatToMatrix(quatData.m, quatData.q);

    //first calibrate quat in null position
    //for that reset sensor data
//	  AQ_ACCX = v0a[0];
//    AQ_ACCY = v0a[1];
//    AQ_ACCZ = v0a[2];
//    AQ_MAGX = quatData.v0m[0];
//    AQ_MAGY = quatData.v0m[1];
//    AQ_MAGZ = quatData.v0m[2];
    //reset bias
    quatData.rateBias[0] = 0;
    quatData.rateBias[1] = 0;
    quatData.rateBias[2] = 0;
    //global_data.param[PARAM_QUAT_KP] *= 20.0f;
    for (i = 0; i < 2000; i++)
    {
		//quatUpdate();
    }
    //global_data.param[PARAM_QUAT_KP] /= 20.0f;

    normalizeVector4(quatData.q);

/*
	const int reads = 100;
	const int results = 26;
    for (i = 0; i < results; i++)
    {
    	for (j=0; j < reads; j++)
    	{
        	acc_read_raw(reads);
    	}
    	gyro_read();
    	acc_read();
    	rateSum[0] += AQ_RATEX;
    	rateSum[1] += AQ_RATEY;
    	rateSum[2] += AQ_RATEZ;
    }
    quatData.rateBias[0] = rateSum[0] / (float)results;
    quatData.rateBias[1] = rateSum[1] / (float)results;
    quatData.rateBias[2] = rateSum[2] / (float)results;
*/
	//gyro_read_raw(1);

	//acc_read_raw(1);
	//quatData.lastUpdate = sys_time_clock_get_time_usec();

}

// ~ 485 us
void quatUpdate(const uint8_t updateVect[3],
				quat_real_t dt,
				const bool isFlying,
				const quat_real_t z[9],
				const struct attitude_estimator_quat_params* params,
				struct vehicle_attitude_s* attitudeResult)
{
    quat_real_t gyro[3], acc[3], mag[3];		// measured gyro, acc & mag
    quat_real_t estAcc[3], estMag[3];	// estimated
    quat_real_t dotRate[3];		// angular velocity
    quat_real_t rotError[3];		// measured error
    quat_real_t gDev;			// deviation from 1g
    quat_real_t yaw, pitch, roll;

    if(updateVect[0] == 1){
        gyro[0] = z[0];
        gyro[1] = z[1];
        gyro[2] = z[2];
        attitudeResult->rollspeed = gyro[0];
        attitudeResult->pitchspeed = gyro[1];
        attitudeResult->yawspeed = gyro[2];
    }
    else{
    	gyro[0] = 0;
    	gyro[1] = 0;
    	gyro[2] = 0;
    }

    if(updateVect[1] == 1){
    	acc[0] = z[3];
    	acc[1] = z[4];
    	acc[2] = z[5];
    }
    else{
    	acc[0] = 0;
    	acc[1] = 0;
    	acc[2] = 0;
    }

    if(updateVect[2] == 1){
    	mag[0] = z[6];
    	mag[1] = z[7];
    	mag[2] = z[8];
    }
    else{
    	mag[0] = 0;
    	mag[1] = 0;
    	mag[2] = 0;
    }

    // estimate bias shift due to motor speed (electromagnetic induced field)
    gDev = 1.0f;
    if (isFlying)
    {
    	// calculate deviance from 1g
    	gDev = 1.0f + fabsf(sqrtf(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]) - CONSTANTS_ONE_G) * params->accdist;
    	gDev = 1.0f / (gDev*gDev);
    }

    normalizeVector3(acc);
    normalizeVector3(mag);

    // rotate gravity to body frame of reference
    rotateVecByRevMatrix(estAcc, v0a, quatData.m);

    // rotate mags to body frame of reference
    rotateVecByRevMatrix(estMag, quatData.v0m, quatData.m);


    // measured error, starting with accel vector
    rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * params->ka * gDev;
    rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * params->ka * gDev;
    rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * params->ka * gDev;

	rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * params->km1;
	rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * params->km1;
	rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * params->km2;

    // Integrate error into existing bias estimates
    quatData.rateBias[0] -= rotError[0] * params->ki;
    quatData.rateBias[1] -= rotError[1] * params->ki;
    quatData.rateBias[2] -= rotError[2] * params->ki;

    // apply bias and P&I portions of error
    dotRate[0] = (gyro[0] - quatData.rateBias[0]) * dt + params->kp * rotError[0];
    dotRate[1] = (gyro[1] - quatData.rateBias[1]) * dt + params->kp * rotError[1];
    dotRate[2] = (gyro[2] - quatData.rateBias[2]) * dt + params->kp * rotError[2];


    rotateQuat(quatData.q, quatData.q, dotRate, dt);
    quatToMatrix(quatData.m, quatData.q);

    // extract and save euler angles
    quatExtractEuler(quatData.q, &yaw, &pitch, &roll);
    yaw = compassNormalizeRad(yaw);

    attitudeResult->roll = roll;
    attitudeResult->pitch = -pitch;
    attitudeResult->yaw = yaw;

    // Copy quaternion
    memcpy(attitudeResult->q, quatData.q, sizeof(quatData.q));
    attitudeResult->q_valid = true;
    // Copy rotation matrix
    memcpy(attitudeResult->R, quatData.m, sizeof(quatData.m));
    attitudeResult->R_valid = true;

	// Copy offsets
	memcpy(attitudeResult->rate_offsets, quatData.rateBias, sizeof(quatData.rateBias));
}
