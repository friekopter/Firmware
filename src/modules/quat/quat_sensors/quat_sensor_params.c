#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(IMU_GYO_SCAL_X, 1.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_SCAL_Y, 1.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_SCAL_Z, 1.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS1_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS1_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS1_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS2_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS2_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS2_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS3_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS3_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_BIAS3_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_XY, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_XZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_YX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_YZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_ZX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_GYO_ALGN_ZY, 0.0f);

/* accel offsets */
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL_X, 1.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL_Y, 1.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL_Z, 1.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL1_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL1_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL1_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL2_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL2_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL2_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL3_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL3_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_SCAL3_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS1_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS1_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS1_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS2_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS2_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS2_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS3_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS3_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_BIAS3_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_XY, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_XZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_YX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_YZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_ZX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_ACC_ALGN_ZY, 0.0f);

/* accel offsets */
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL_X, 1.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL_Y, 1.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL_Z, 1.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL1_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL1_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL1_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL2_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL2_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL2_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL3_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL3_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_SCAL3_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS1_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS1_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS1_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS2_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS2_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS2_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS3_X, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS3_Y, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_BIAS3_Z, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_XY, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_XZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_YX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_YZ, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_ZX, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_ALGN_ZY, 0.0f);
PARAM_DEFINE_FLOAT(IMU_MAG_INCL,-64.0f);

PARAM_DEFINE_INT32(IMU_GYO_RA_POLL, 1000);
PARAM_DEFINE_INT32(IMU_GYO_RA_SAMP, 1000);
PARAM_DEFINE_INT32(IMU_ACC_RA_POLL, 800);
PARAM_DEFINE_INT32(IMU_ACC_RA_SAMP, 800);
PARAM_DEFINE_INT32(IMU_GYO_DLPF, 42);
