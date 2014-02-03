
/*
 * @file quat_position_control_params.c
 * 
 * Parameters for EKF filter
 */

#include "quat_pos_control_params.h"

/* Extended Kalman Filter covariances */

/* controller parameters */
//Noise
PARAM_DEFINE_FLOAT(Q_U_ACC_N, +9.5468e-05f); 	//acc sensor
PARAM_DEFINE_FLOAT(Q_U_DIST_N, +1.8705e-02f);  	//acc deviation from one G
PARAM_DEFINE_FLOAT(Q_U_ALT_N, +1.7077e-01f); 	//baro sensor
PARAM_DEFINE_FLOAT(Q_U_FLOW_VEL_N, +1e-03f); 	//flow sensor
PARAM_DEFINE_FLOAT(Q_U_FLW_VEL_M_N, +100.0f); 	//flow sensor max noise if no valid signal available
PARAM_DEFINE_FLOAT(Q_U_FLOW_VELA_N, +1e-03f); 	//sonar sensor
PARAM_DEFINE_FLOAT(Q_U_FLOW_A_N, +1e-03f); 	//sonar sensor
PARAM_DEFINE_FLOAT(Q_U_MAG_N, +3.8226e-01f); 	//mag sensor

// state variance
PARAM_DEFINE_FLOAT(Q_U_ACC_BIAS_Q, +9.3722e-04f); 	//variables 3,4,5
PARAM_DEFINE_FLOAT(Q_U_GYO_BIAS_Q, +4.6872e-02f); 	//variables 6,7,8
PARAM_DEFINE_FLOAT(Q_U_POS_ALT_Q, +4.5576e+03f);
PARAM_DEFINE_FLOAT(Q_U_POS_Q, +6.0490e+03f);
PARAM_DEFINE_FLOAT(Q_U_PRES_ALT_Q, +6.5172e+01f); 	//variable 13
PARAM_DEFINE_FLOAT(Q_U_PRES_ALT_K, 1.0e-3f);
PARAM_DEFINE_FLOAT(Q_U_QUAT_Q, +7.3021e-04f); 		//variables 9,10,11,12
PARAM_DEFINE_FLOAT(Q_U_VEL_ALT_Q, +1.4149e-01f); 	//variable 2
PARAM_DEFINE_FLOAT(Q_U_VEL_Q, +7.6020e-02f); 		//variables 0,1

// Process covariance
PARAM_DEFINE_FLOAT(Q_U_ACC_BIAS_V, +2.7535e-07f);//changing
PARAM_DEFINE_FLOAT(Q_U_GYO_BIAS_V, +8.2738e-07f);
PARAM_DEFINE_FLOAT(Q_U_ALT_POS_V, +5.3821e-09f);
PARAM_DEFINE_FLOAT(Q_U_POS_V, +6.4505e-08f);
PARAM_DEFINE_FLOAT(Q_U_PRES_ALT_V, +1.0204e-04f);
PARAM_DEFINE_FLOAT(Q_U_RATE_V, +6.0568e-05f);
PARAM_DEFINE_FLOAT(Q_U_VEL_V, +1.0980e-07f);
PARAM_DEFINE_FLOAT(Q_U_ALT_VEL_V, +2.8103e-07f);

PARAM_DEFINE_FLOAT(Q_U_VEL_DELAY, -1.0373e+05f);
PARAM_DEFINE_FLOAT(Q_U_POS_DELAY, +2.0574e+03f);

PARAM_DEFINE_FLOAT(Q_U_GPS_ALT_M_N, +3.8535e-05f);
PARAM_DEFINE_FLOAT(Q_U_GPS_ALT_N, +7.6558e-05f);
PARAM_DEFINE_FLOAT(Q_U_GPS_POS_M_N, +4.7413e-05f);
PARAM_DEFINE_FLOAT(Q_U_GPS_POS_N, +1.7620e-05f);
PARAM_DEFINE_FLOAT(Q_U_GPS_VD_M_N, +1.5841e-02f);
PARAM_DEFINE_FLOAT(Q_U_GPS_VD_N, +3.7820e+00f);
PARAM_DEFINE_FLOAT(Q_U_GPS_VEL_M_N, +1.2336e-02f);
PARAM_DEFINE_FLOAT(Q_U_GPS_VEL_N, +4.6256e-02f);

PARAM_DEFINE_INT32(Q_U_SENS_HIST, 10);
PARAM_DEFINE_INT32(Q_U_SENS_INTV, 5);
PARAM_DEFINE_INT32(Q_U_STATES, 14);
PARAM_DEFINE_INT32(Q_U_BI_UP_CO, 1);

PARAM_DEFINE_FLOAT(Q_N_MAX_SPEED,	    5.0f);	// m/s
PARAM_DEFINE_FLOAT(Q_N_MAX_DECENT,	    1.5f);	// m/s

// speed => tilt PID
PARAM_DEFINE_FLOAT(Q_N_SPEED_P,	    7.0f);
PARAM_DEFINE_FLOAT(Q_N_SPEED_I,    0.005f);
PARAM_DEFINE_FLOAT(Q_N_SPEED_PM,	    20.0f);
PARAM_DEFINE_FLOAT(Q_N_SPEED_IM,	    20.0f);
PARAM_DEFINE_FLOAT(Q_N_SPEED_OM,	    30.0f);

// distance => speed PID
PARAM_DEFINE_FLOAT(Q_N_DIST_P,	    0.5f);
PARAM_DEFINE_FLOAT(Q_N_DIST_I,	    0.0f);
PARAM_DEFINE_FLOAT(Q_N_DIST_PM,	    999.0f);
PARAM_DEFINE_FLOAT(Q_N_DIST_IM,	    0.0f);
PARAM_DEFINE_FLOAT(Q_N_DIST_OM,	    999.0f);


// Altitude hold Position PID
PARAM_DEFINE_FLOAT(Q_N_ALT_POS_P,	    0.20f);
PARAM_DEFINE_FLOAT(Q_N_ALT_POS_I,	    0.0f);
PARAM_DEFINE_FLOAT(Q_N_ALT_POS_PM,	    2.5f);
PARAM_DEFINE_FLOAT(Q_N_ALT_POS_IM,	    0.0f);
PARAM_DEFINE_FLOAT(Q_N_ALT_POS_OM,	    2.5f);

// Altitude hold Speed PID
PARAM_DEFINE_FLOAT(Q_N_ALT_SPED_P,0.333f);//	    200.0f);
PARAM_DEFINE_FLOAT(Q_N_ALT_SPED_I,0.00475f);//	    2.85f);
PARAM_DEFINE_FLOAT(Q_N_ALT_SPED_PM,0.25f);//	    150.0f);
PARAM_DEFINE_FLOAT(Q_N_ALT_SPED_IM,0.8f);//	    600.0f);
PARAM_DEFINE_FLOAT(Q_N_ALT_SPED_OM,0.8f);//	    600.0f);
PARAM_DEFINE_FLOAT(Q_N_ACC_INAIR_D,0.4f);

int parameters_init(struct quat_position_control_NAV_param_handles *nav,
		struct quat_position_control_UKF_param_handles *ukf)
{
	nav->nav_max_speed = param_find("Q_N_MAX_SPEED");
	nav->nav_max_decent = param_find("Q_N_MAX_DECENT");
	nav->nav_speed_p = param_find("Q_N_SPEED_P");
	nav->nav_speed_i = param_find("Q_N_SPEED_I");
	nav->nav_speed_pm = param_find("Q_N_SPEED_PM");
	nav->nav_speed_im = param_find("Q_N_SPEED_IM");
	nav->nav_speed_om = param_find("Q_N_SPEED_OM");
	nav->nav_dist_p = param_find("Q_N_DIST_P");
	nav->nav_dist_i = param_find("Q_N_DIST_I");
	nav->nav_dist_pm = param_find("Q_N_DIST_PM");
	nav->nav_dist_im = param_find("Q_N_DIST_IM");
	nav->nav_dist_om = param_find("Q_N_DIST_OM");
	nav->nav_alt_speed_p = param_find("Q_N_ALT_SPED_P");
	nav->nav_alt_speed_i = param_find("Q_N_ALT_SPED_I");
	nav->nav_alt_speed_pm = param_find("Q_N_ALT_SPED_PM");
	nav->nav_alt_speed_im = param_find("Q_N_ALT_SPED_IM");
	nav->nav_alt_speed_om = param_find("Q_N_ALT_SPED_OM");
	nav->nav_alt_pos_p = param_find("Q_N_ALT_POS_P");
	nav->nav_alt_pos_i = param_find("Q_N_ALT_POS_I");
	nav->nav_alt_pos_pm = param_find("Q_N_ALT_POS_PM");
	nav->nav_alt_pos_im = param_find("Q_N_ALT_POS_IM");
	nav->nav_alt_pos_om = param_find("Q_N_ALT_POS_OM");
	nav->nav_in_air_acc_deviation = param_find("Q_N_ACC_INAIR_D");


	ukf->ukf_acc_bias_q = param_find("Q_U_ACC_BIAS_Q");
	ukf->ukf_acc_bias_v = param_find("Q_U_ACC_BIAS_V");
	ukf->ukf_acc_n = param_find("Q_U_ACC_N");
	ukf->ukf_alt_n = param_find("Q_U_ALT_N");
	ukf->ukf_alt_pos_v = param_find("Q_U_ALT_POS_V");
	ukf->ukf_alt_vel_v = param_find("Q_U_ALT_VEL_V");
	ukf->ukf_dist_n = param_find("Q_U_DIST_N");


	ukf->ukf_gps_alt_n = param_find("Q_U_GPS_ALT_N");
	ukf->ukf_gps_alt_m_n = param_find("Q_U_GPS_ALT_M_N");
	ukf->ukf_gps_pos_n = param_find("Q_U_GPS_POS_N");
	ukf->ukf_gps_pos_m_n = param_find("Q_U_GPS_POS_M_N");
	ukf->ukf_gps_vd_n = param_find("Q_U_GPS_VD_N");
	ukf->ukf_gps_vd_m_n = param_find("Q_U_GPS_VD_M_N");
	ukf->ukf_gps_vel_n = param_find("Q_U_GPS_VEL_N");
	ukf->ukf_gps_vel_m_n = param_find("Q_U_GPS_VEL_M_N");
	ukf->ukf_gyo_bias_q = param_find("Q_U_GYO_BIAS_Q");
	ukf->ukf_gyo_bias_v = param_find("Q_U_GYO_BIAS_V");
	ukf->ukf_mag_n = param_find("Q_U_MAG_N");
	ukf->ukf_flow_vel_n = param_find("Q_U_FLOW_VEL_N");
	ukf->ukf_flow_vel_max_n = param_find("Q_U_FLW_VEL_M_N");
	ukf->ukf_flow_vel_alt_n = param_find("Q_U_FLOW_VELA_N");
	ukf->ukf_flow_alt_n = param_find("Q_U_FLOW_A_N");
	ukf->ukf_pos_alt_q = param_find("Q_U_POS_ALT_Q");
	ukf->ukf_pos_delay = param_find("Q_U_POS_DELAY");
	ukf->ukf_pos_q = param_find("Q_U_POS_Q");
	ukf->ukf_pos_v = param_find("Q_U_POS_V");
	ukf->ukf_pres_alt_q = param_find("Q_U_PRES_ALT_Q");
	ukf->ukf_pres_alt_k = param_find("Q_U_PRES_ALT_K");
	ukf->ukf_pres_alt_v = param_find("Q_U_PRES_ALT_V");
    ukf->ukf_quat_q = param_find("Q_U_QUAT_Q");
    ukf->ukf_rate_v = param_find("Q_U_RATE_V");
    ukf->ukf_vel_alt_q = param_find("Q_U_VEL_ALT_Q");
	ukf->ukf_pos_delay = param_find("Q_U_VEL_DELAY");
	ukf->ukf_vel_q = param_find("Q_U_VEL_Q");
	ukf->ukf_vel_v = param_find("Q_U_VEL_V");
	ukf->ukf_sens_hist = param_find("Q_U_SENS_HIST");
	ukf->ukf_raw_intv = param_find("Q_U_SENS_INTV");
	ukf->ukf_states = param_find("Q_U_STATES");
	ukf->ukf_bias_update_count = param_find("Q_U_BI_UP_CO");
	return OK;
}

int parameters_update(const struct quat_position_control_NAV_param_handles *nav_handles, struct quat_position_control_NAV_params *nav_params,
		const struct quat_position_control_UKF_param_handles *ukf_handles, struct quat_position_control_UKF_params *ukf_params)
{
	param_get(nav_handles->nav_max_speed, &(nav_params->nav_max_speed));
	param_get(nav_handles->nav_max_decent, &(nav_params->nav_max_decent));
	param_get(nav_handles->nav_speed_p, &(nav_params->nav_speed_p));
	param_get(nav_handles->nav_speed_i, &(nav_params->nav_speed_i));
	param_get(nav_handles->nav_speed_pm, &(nav_params->nav_speed_pm));
	param_get(nav_handles->nav_speed_im, &(nav_params->nav_speed_im));
	param_get(nav_handles->nav_speed_om, &(nav_params->nav_speed_om));
	param_get(nav_handles->nav_dist_p, &(nav_params->nav_dist_p));
	param_get(nav_handles->nav_dist_i, &(nav_params->nav_dist_i));
	param_get(nav_handles->nav_dist_pm, &(nav_params->nav_dist_pm));
	param_get(nav_handles->nav_dist_im, &(nav_params->nav_dist_im));
	param_get(nav_handles->nav_dist_om, &(nav_params->nav_dist_om));
	param_get(nav_handles->nav_alt_speed_p, &(nav_params->nav_alt_speed_p));
	param_get(nav_handles->nav_alt_speed_i, &(nav_params->nav_alt_speed_i));
	param_get(nav_handles->nav_alt_speed_pm, &(nav_params->nav_alt_speed_pm));
	param_get(nav_handles->nav_alt_speed_im, &(nav_params->nav_alt_speed_im));
	param_get(nav_handles->nav_alt_speed_om, &(nav_params->nav_alt_speed_om));
	param_get(nav_handles->nav_alt_pos_p, &(nav_params->nav_alt_pos_p));
	param_get(nav_handles->nav_alt_pos_i, &(nav_params->nav_alt_pos_i));
	param_get(nav_handles->nav_alt_pos_pm, &(nav_params->nav_alt_pos_pm));
	param_get(nav_handles->nav_alt_pos_im, &(nav_params->nav_alt_pos_im));
	param_get(nav_handles->nav_alt_pos_om, &(nav_params->nav_alt_pos_om));
	param_get(nav_handles->nav_in_air_acc_deviation, &(nav_params->nav_in_air_acc_deviation));

	param_get(ukf_handles->ukf_acc_bias_q, &(ukf_params->ukf_acc_bias_q));
	param_get(ukf_handles->ukf_acc_bias_v, &(ukf_params->ukf_acc_bias_v));
	param_get(ukf_handles->ukf_acc_n, &(ukf_params->ukf_acc_n));
	param_get(ukf_handles->ukf_alt_n, &(ukf_params->ukf_alt_n));
	param_get(ukf_handles->ukf_alt_pos_v, &(ukf_params->ukf_alt_pos_v));
	param_get(ukf_handles->ukf_alt_vel_v, &(ukf_params->ukf_alt_vel_v));
	param_get(ukf_handles->ukf_dist_n, &(ukf_params->ukf_dist_n));
	param_get(ukf_handles->ukf_gps_alt_m_n, &(ukf_params->ukf_gps_alt_m_n));
	param_get(ukf_handles->ukf_gps_alt_n, &(ukf_params->ukf_gps_alt_n));
	param_get(ukf_handles->ukf_gps_pos_m_n, &(ukf_params->ukf_gps_pos_m_n));
	param_get(ukf_handles->ukf_gps_pos_n, &(ukf_params->ukf_gps_pos_n));
	param_get(ukf_handles->ukf_gps_vd_m_n, &(ukf_params->ukf_gps_vd_m_n));
	param_get(ukf_handles->ukf_gps_vd_n, &(ukf_params->ukf_gps_vd_n));
	param_get(ukf_handles->ukf_gps_vel_m_n, &(ukf_params->ukf_gps_vd_m_n));
	param_get(ukf_handles->ukf_gps_vel_n, &(ukf_params->ukf_gps_vel_n));
	param_get(ukf_handles->ukf_gyo_bias_q, &(ukf_params->ukf_gyo_bias_q));
	param_get(ukf_handles->ukf_gyo_bias_v, &(ukf_params->ukf_gyo_bias_v));
	param_get(ukf_handles->ukf_flow_vel_n, &(ukf_params->ukf_flow_vel_n));
	param_get(ukf_handles->ukf_flow_vel_max_n, &(ukf_params->ukf_flow_vel_max_n));
	param_get(ukf_handles->ukf_flow_alt_n, &(ukf_params->ukf_flow_alt_n));
	param_get(ukf_handles->ukf_flow_vel_alt_n, &(ukf_params->ukf_flow_vel_alt_n));
	param_get(ukf_handles->ukf_mag_n, &(ukf_params->ukf_mag_n));
	param_get(ukf_handles->ukf_pos_alt_q, &(ukf_params->ukf_pos_alt_q));
	param_get(ukf_handles->ukf_pos_delay, &(ukf_params->ukf_pos_delay));
	param_get(ukf_handles->ukf_pos_q, &(ukf_params->ukf_pos_q));
	param_get(ukf_handles->ukf_pos_v, &(ukf_params->ukf_pos_v));
	param_get(ukf_handles->ukf_pres_alt_q, &(ukf_params->ukf_pres_alt_q));
	param_get(ukf_handles->ukf_pres_alt_k, &(ukf_params->ukf_pres_alt_k));
	param_get(ukf_handles->ukf_pres_alt_v, &(ukf_params->ukf_pres_alt_v));
	param_get(ukf_handles->ukf_quat_q, &(ukf_params->ukf_quat_q));
	param_get(ukf_handles->ukf_rate_v, &(ukf_params->ukf_rate_v));
	param_get(ukf_handles->ukf_vel_alt_q, &(ukf_params->ukf_vel_alt_q));
	param_get(ukf_handles->ukf_vel_delay, &(ukf_params->ukf_vel_delay));
	param_get(ukf_handles->ukf_vel_q, &(ukf_params->ukf_vel_q));
	param_get(ukf_handles->ukf_vel_v, &(ukf_params->ukf_vel_v));

	param_get(ukf_handles->ukf_sens_hist, &(ukf_params->ukf_sens_hist));
	param_get(ukf_handles->ukf_raw_intv, &(ukf_params->ukf_raw_intv));
	param_get(ukf_handles->ukf_states, &(ukf_params->ukf_states));
	param_get(ukf_handles->ukf_bias_update_count, &(ukf_params->ukf_bias_update_count));


	return OK;
}
