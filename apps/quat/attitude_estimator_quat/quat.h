
#ifndef _quat_h
#define _quat_h
#include <attitude_estimator_quat_params.h>
#include <uORB/topics/vehicle_attitude.h>


#define QUAT_N                  1.0f		// factor that determines the convergence speed of the quat's numerical error: (n * dt) < 1
#define QUAT_BIAS_FACTOR	0.001f		//(AQ_TIMESTEP * 1.0f)

typedef float quat_real_t;

typedef struct {
    quat_real_t v0m[3];
    quat_real_t rateBias[3];
    quat_real_t q[4];
    quat_real_t m[3][3];
} quatStruct_t;


extern quatStruct_t quatData;

extern void quatInit(void);
extern void quatUpdate(const uint8_t updateVect[3],
						quat_real_t dt,
						const bool isFlying,
						const quat_real_t z[9],
						const struct attitude_estimator_quat_params* params,
						struct vehicle_attitude_s* attitudeResult );
extern void rotateVecByMatrix(quat_real_t *vr, const quat_real_t * const v, quat_real_t m[3][3]);
extern void rotateVecByRevMatrix(quat_real_t *vr, const quat_real_t * const v, quat_real_t m[3][3]);

void normalizeVector3(quat_real_t *v);
void normalizeVector4(quat_real_t *v);
void quatToMatrix(quat_real_t m[3][3], const quat_real_t * const q);
void quatExtractEuler( const quat_real_t *q, quat_real_t *yaw, quat_real_t *pitch, quat_real_t *roll);
void rotateQuat(quat_real_t *qr, const quat_real_t * const q, const quat_real_t * const rate, const quat_real_t dt);
void quatMatrixMultiply(quat_real_t mr[3][3], const quat_real_t ma[3][3], const quat_real_t mb[3][3]);
void quatMatrixTranspose(quat_real_t mt[3][3], const quat_real_t m[3][3]);

#endif

