#ifndef quat_pos_control_params_h_
#define quat_pos_control_params_h_

#include <systemlib/param/param.h>

    struct quat_position_control_UKF_params {
    	float ukf_vel_q,
    	ukf_vel_alt_q,
    	ukf_pos_q,
    	ukf_pos_alt_q,
    	ukf_acc_bias_q,
    	ukf_gyo_bias_q,
    	ukf_quat_q,
    	ukf_pres_alt_q,
    	ukf_acc_bias_v,
    	ukf_gyo_bias_v,
    	ukf_rate_v,
    	ukf_pres_alt_v,
    	ukf_pos_v,
    	ukf_vel_v,
    	ukf_alt_pos_v,
    	ukf_alt_vel_v,
    	ukf_gps_pos_n,
    	ukf_gps_pos_m_n,
    	ukf_gps_alt_n,
    	ukf_gps_alt_m_n,
    	ukf_gps_vel_n,
    	ukf_gps_vel_m_n,
    	ukf_gps_vd_n,
    	ukf_gps_vd_m_n,
    	ukf_alt_n,
    	ukf_acc_n,
    	ukf_dist_n,
    	ukf_mag_n,
    	ukf_pos_delay,
    	ukf_vel_delay;
    };

    struct quat_position_control_UKF_param_handles {
    	float ukf_vel_q,
		ukf_vel_alt_q,
		ukf_pos_q,
		ukf_pos_alt_q,
		ukf_acc_bias_q,
		ukf_gyo_bias_q,
		ukf_quat_q,
		ukf_pres_alt_q,
		ukf_acc_bias_v,
		ukf_gyo_bias_v,
		ukf_rate_v,
    	ukf_pres_alt_v,
		ukf_pos_v,
		ukf_vel_v,
		ukf_alt_pos_v,
		ukf_alt_vel_v,
		ukf_gps_pos_n,
		ukf_gps_pos_m_n,
		ukf_gps_alt_n,
		ukf_gps_alt_m_n,
		ukf_gps_vel_n,
		ukf_gps_vel_m_n,
		ukf_gps_vd_n,
		ukf_gps_vd_m_n,
		ukf_alt_n,
		ukf_acc_n,
		ukf_dist_n,
		ukf_mag_n,
		ukf_pos_delay,
		ukf_vel_delay;
    };

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct quat_position_control_UKF_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct quat_position_control_UKF_param_handles *h, struct quat_position_control_UKF_params *p);


#endif //quat_pos_control_params_h_
