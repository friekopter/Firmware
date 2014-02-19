#include "util.h"
#include "float.h"
#include <quat/utils/aq_math.h>

#define IMU_STATIC_STD		0.05f						// Standard deviation

void crossVector3(float *vr, float *va, float *vb);
float dotVector3(float *va, float *vb);
void utilRotateVectorByQuat(float *vr, float *v, float *q);
void utilMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll);

uint32_t heapUsed, dataSramUsed;
uint32_t *ccmHeap[UTIL_CCM_HEAP_SIZE] __attribute__((section(".ccm")));

void *aqCalloc(size_t count, size_t size) {
    heapUsed += count * size;
    printf("heap used: %d\n",heapUsed);
    return calloc(count, size);
}
// allocates memory from 64KB CCM
void *aqDataCalloc(uint16_t count, uint16_t size) {
    uint32_t words;

    // round up to word size
    words = (count*size + sizeof(int)-1) / sizeof(int);

    if ((dataSramUsed + words) > UTIL_CCM_HEAP_SIZE) {
    	fprintf(stderr, "Out of data SRAM!\n");
    }
    else {
    	dataSramUsed += words;
    }

    return (void *)(ccmHeap + dataSramUsed - words);
}


int constrainInt(int i, int lo, int hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;
    return i;
}

float constrainFloat(float i, float lo, float hi) {
    if (i < lo)
       return	lo;
    if (i > hi)
       return	hi;
    return i;
}


void utilFilterReset(utilFilter_t *f, float setpoint) {
    f->z1 = setpoint;
}

void utilFilterReset3(utilFilter_t *f, float setpoint) {
    utilFilterReset(&f[0], setpoint);
    utilFilterReset(&f[1], setpoint);
    utilFilterReset(&f[2], setpoint);
}

void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint) {
    f->tc = dt / tau;
    utilFilterReset(f, setpoint);
}

void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint) {
    utilFilterInit(&f[0], dt, tau, setpoint);
    utilFilterInit(&f[1], dt, tau, setpoint);
    utilFilterInit(&f[2], dt, tau, setpoint);
}

inline float utilFilter3(utilFilter_t *f, float signal) {
    return utilFilter(&f[0], utilFilter(&f[1], utilFilter(&f[2], signal)));
}

inline float utilFilter(utilFilter_t *f, float signal) {
    register float z1;

    z1 = f->z1 + (signal - f->z1) * f->tc;

    f->z1 = z1;

    return z1;
}

// wait for lack of movement
void utilQuasiStatic(int n, float acc_x, float acc_y, float acc_z) {
    float stdX, stdY, stdZ;
    float vX[n];
    float vY[n];
    float vZ[n];
    int i, j;

    i = 0;
    j = 0;
    do {

	vX[j] = acc_x;
	vY[j] = acc_y;
	vZ[j] = acc_z;
	j = (j + 1) % n;

	if (i >= n) {
	    arm_std_f32(vX, n, &stdX);
	    arm_std_f32(vY, n, &stdY);
	    arm_std_f32(vZ, n, &stdZ);
	}

	i++;
    } while (i <= n || (stdX + stdY + stdZ) > IMU_STATIC_STD);
}

void utilMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll) {
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

void utilQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;

    q0 = q[1];
    q1 = q[2];
    q2 = q[3];
    q3 = q[0];
    float q0sq = q0*q0;
    float q1sq = q1*q1;
    float q2sq = q2*q2;
    float q3sq = q3*q3;

    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3sq - q2sq - q1sq + q0sq));
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
    *roll  = atan2f((2.0f * (q1 * q2 + q0 * q3)),-(1.0f-2.0f*(q3sq + q2sq)));
/*
    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
    *pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
    *roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));*/
}

// result and source can be the same
void utilRotateQuat(float *qr, float *q, float *rate, float dt) {
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
    t = -(0.5f * __aq_sinf(s) / s);
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


void utilRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    utilRotateVectorByQuat(vr, v, qc);
}

void utilRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

void utilRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

void utilRotateVecByMatrix2(float *vr, float *v, float m[3][3]) {
    vr[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
    vr[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
    vr[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
}

void utilRotateVecByRevMatrix2(float *vr, float *v, float m[3][3]) {
    vr[0] = m[0][0]*v[0] + m[1][0]*v[1] + m[2][0]*v[2];
    vr[1] = m[0][1]*v[0] + m[1][1]*v[1] + m[2][1]*v[2];
    vr[2] = m[0][2]*v[0] + m[1][2]*v[1] + m[2][2]*v[2];
}

void utilQuatToMatrix(float *m, float *q, int normalize) {
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

void utilQuatToMatrix2(float m[3][3], float *q, int normalize) {
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
    m[0][0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1][1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2][2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1][0] = 2.0 * (tmp1 + tmp2) * invs;
    m[0][1] = 2.0 * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2][0] = 2.0 * (tmp1 - tmp2) * invs;
    m[0][2] = 2.0 * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2][1] = 2.0 * (tmp1 + tmp2) * invs;
    m[1][2] = 2.0 * (tmp1 - tmp2) * invs;
}

void utilNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = aq_sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}

void utilNormalizeQuat(float *qr, float *q) {
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

void utilRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0*y*w*v[2] - 2.0*z*w*v[1] + x*x*v[0] + 2.0*y*x*v[1] + 2.0*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0*x*y*v[0] + y*y*v[1] + 2.0*z*y*v[2] + 2.0*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0*x*z*v[0] + 2.0*y*z*v[1] + z*z*v[2] - 2.0*w*y*v[0] - y*y*v[2] + 2.0*w*x*v[1] - x*x*v[2] + w*w*v[2];
}


float utilPresToAlt(float pressure) {
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

