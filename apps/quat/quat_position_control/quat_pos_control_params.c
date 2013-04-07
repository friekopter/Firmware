
/*
 * @file quat_position_control_params.c
 * 
 * Parameters for EKF filter
 */

#include "quat_pos_control_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
    PARAM_DEFINE_FLOAT(Q_U_ACC_BIAS_Q, +9.3722e-04f);
    PARAM_DEFINE_FLOAT(Q_U_ACC_BIAS_V, +2.7535e-07f);
    PARAM_DEFINE_FLOAT(Q_U_ACC_N, +9.5468e-05f);
    PARAM_DEFINE_FLOAT(Q_U_ALT_N, +1.7077e-01f);
    PARAM_DEFINE_FLOAT(Q_U_ALT_POS_V, +5.3821e-09f);
    PARAM_DEFINE_FLOAT(Q_U_ALT_VEL_V, +2.8103e-07f);
    PARAM_DEFINE_FLOAT(Q_U_DIST_N, +1.8705e-02f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_ALT_M_N, +3.8535e-05f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_ALT_N, +7.6558e-05f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_POS_M_N, +4.7413e-05f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_POS_N, +1.7620e-05f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_VD_M_N, +1.5841e-02f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_VD_N, +3.7820e+00f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_VEL_M_N, +1.2336e-02f);
    PARAM_DEFINE_FLOAT(Q_U_GPS_VEL_N, +4.6256e-02f);
    PARAM_DEFINE_FLOAT(Q_U_GYO_BIAS_Q, +4.6872e-02f);
    PARAM_DEFINE_FLOAT(Q_U_GYO_BIAS_V, +8.2738e-07f);
    PARAM_DEFINE_FLOAT(Q_U_MAG_N, +3.8226e-01f);
    PARAM_DEFINE_FLOAT(Q_U_POS_ALT_Q, +4.5576e+03f);
    PARAM_DEFINE_FLOAT(Q_U_POS_DELAY, +2.0574e+03f);
    PARAM_DEFINE_FLOAT(Q_U_POS_Q, +6.0490e+03f);
    PARAM_DEFINE_FLOAT(Q_U_POS_V, +6.4505e-08f);
    PARAM_DEFINE_FLOAT(Q_U_PRES_ALT_Q, +6.5172e+01f);
    PARAM_DEFINE_FLOAT(Q_U_PRES_ALT_V, +1.0204e-04f);
    PARAM_DEFINE_FLOAT(Q_U_QUAT_Q, +7.3021e-04f);
    PARAM_DEFINE_FLOAT(Q_U_RATE_V, +6.0568e-05f);
    PARAM_DEFINE_FLOAT(Q_U_VEL_ALT_Q, +1.4149e-01f);
    PARAM_DEFINE_FLOAT(Q_U_VEL_DELAY, -1.0373e+05f);
    PARAM_DEFINE_FLOAT(Q_U_VEL_Q, +7.6020e-02f);
    PARAM_DEFINE_FLOAT(Q_U_VEL_V, +1.0980e-07f);


int parameters_init(struct quat_position_control_UKF_param_handles *h)
{
	h->ukf_acc_bias_q = param_find("Q_U_ACC_BIAS_Q");
	h->ukf_acc_bias_v = param_find("Q_U_ACC_BIAS_V");
	h->ukf_acc_n = param_find("Q_U_ACC_N");
	h->ukf_alt_n = param_find("Q_U_ALT_N");
	h->ukf_alt_pos_v = param_find("Q_U_ALT_POS_V");
	h->ukf_alt_vel_v = param_find("Q_U_ALT_VEL_V");
	h->ukf_dist_n = param_find("Q_U_DIST_N");
	h->ukf_gps_alt_n = param_find("Q_U_GPS_ALT_N");
	h->ukf_gps_alt_m_n = param_find("Q_U_GPS_ALT_M_N");
	h->ukf_gps_pos_n = param_find("Q_U_GPS_POS_N");
	h->ukf_gps_pos_m_n = param_find("Q_U_GPS_POS_M_N");
	h->ukf_gps_vd_n = param_find("Q_U_GPS_VD_N");
	h->ukf_gps_vd_m_n = param_find("Q_U_GPS_VD_M_N");
	h->ukf_gps_vel_n = param_find("Q_U_GPS_VEL_N");
	h->ukf_gps_vel_m_n = param_find("Q_U_GPS_VEL_M_N");
	h->ukf_gyo_bias_q = param_find("Q_U_GYO_BIAS_Q");
	h->ukf_gyo_bias_v = param_find("Q_U_GYO_BIAS_V");
	h->ukf_mag_n = param_find("Q_U_MAG_N");
	h->ukf_pos_alt_q = param_find("Q_U_POS_ALT_Q");
	h->ukf_pos_delay = param_find("Q_U_POS_DELAY");
	h->ukf_pos_q = param_find("Q_U_POS_Q");
	h->ukf_pos_v = param_find("Q_U_POS_V");
	h->ukf_pres_alt_q = param_find("Q_U_PRES_ALT_Q");
	h->ukf_pres_alt_v = param_find("Q_U_PRES_ALT_V");
    h->ukf_quat_q = param_find("Q_U_QUAT_Q");
    h->ukf_rate_v = param_find("Q_U_RATE_V");
    h->ukf_vel_alt_q = param_find("Q_U_VEL_ALT_Q");
	h->ukf_pos_delay = param_find("Q_U_VEL_DELAY");
	h->ukf_vel_q = param_find("Q_U_VEL_Q");
	h->ukf_vel_v = param_find("Q_U_VEL_V");
	return OK;
}

int parameters_update(const struct quat_position_control_UKF_param_handles *param, struct quat_position_control_UKF_params *paramDest)
{
	param_get(param->ukf_acc_bias_q, &(paramDest->ukf_acc_bias_q));
	param_get(param->ukf_acc_bias_v, &(paramDest->ukf_acc_bias_v));
	param_get(param->ukf_acc_n, &(paramDest->ukf_acc_n));
	param_get(param->ukf_alt_n, &(paramDest->ukf_alt_n));
	param_get(param->ukf_alt_pos_v, &(paramDest->ukf_alt_pos_v));
	param_get(param->ukf_alt_vel_v, &(paramDest->ukf_alt_vel_v));
	param_get(param->ukf_dist_n, &(paramDest->ukf_dist_n));
	param_get(param->ukf_gps_alt_m_n, &(paramDest->ukf_gps_alt_m_n));
	param_get(param->ukf_gps_alt_n, &(paramDest->ukf_gps_alt_n));
	param_get(param->ukf_gps_pos_m_n, &(paramDest->ukf_gps_pos_m_n));
	param_get(param->ukf_gps_pos_n, &(paramDest->ukf_gps_pos_n));
	param_get(param->ukf_gps_vd_m_n, &(paramDest->ukf_gps_vd_m_n));
	param_get(param->ukf_gps_vd_n, &(paramDest->ukf_gps_vd_n));
	param_get(param->ukf_gps_vel_m_n, &(paramDest->ukf_gps_vd_m_n));
	param_get(param->ukf_gps_vel_n, &(paramDest->ukf_gps_vel_n));
	param_get(param->ukf_gyo_bias_q, &(paramDest->ukf_gyo_bias_q));
	param_get(param->ukf_gyo_bias_v, &(paramDest->ukf_gyo_bias_v));
	param_get(param->ukf_mag_n, &(paramDest->ukf_mag_n));
	param_get(param->ukf_pos_alt_q, &(paramDest->ukf_pos_alt_q));
	param_get(param->ukf_pos_delay, &(paramDest->ukf_pos_delay));
	param_get(param->ukf_pos_q, &(paramDest->ukf_pos_q));
	param_get(param->ukf_pos_v, &(paramDest->ukf_pos_v));
	param_get(param->ukf_pres_alt_q, &(paramDest->ukf_pres_alt_q));
	param_get(param->ukf_pres_alt_v, &(paramDest->ukf_pres_alt_v));
	param_get(param->ukf_quat_q, &(paramDest->ukf_quat_q));
	param_get(param->ukf_rate_v, &(paramDest->ukf_rate_v));
	param_get(param->ukf_vel_alt_q, &(paramDest->ukf_vel_alt_q));
	param_get(param->ukf_vel_delay, &(paramDest->ukf_vel_delay));
	param_get(param->ukf_vel_q, &(paramDest->ukf_vel_q));
	param_get(param->ukf_vel_v, &(paramDest->ukf_vel_v));

	return OK;
}
