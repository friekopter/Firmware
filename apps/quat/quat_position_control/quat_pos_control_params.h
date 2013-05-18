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

    struct quat_position_control_NAV_params {
        	float nav_max_speed,
        	nav_max_decent,
        	nav_speed_p,
        	nav_speed_i,
        	nav_speed_pm,
        	nav_speed_im,
        	nav_speed_om,
        	nav_dist_p,
        	nav_dist_i,
        	nav_dist_pm,
        	nav_dist_im,
        	nav_dist_om,
        	nav_alt_speed_p,
        	nav_alt_speed_i,
        	nav_alt_speed_pm,
        	nav_alt_speed_im,
        	nav_alt_speed_om,
        	nav_alt_pos_p,
        	nav_alt_pos_i,
        	nav_alt_pos_pm,
        	nav_alt_pos_im,
        	nav_alt_pos_om;
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

    struct quat_position_control_NAV_param_handles {
        	float nav_max_speed,
        	nav_max_decent,
        	nav_speed_p,
        	nav_speed_i,
        	nav_speed_pm,
        	nav_speed_im,
        	nav_speed_om,
        	nav_dist_p,
        	nav_dist_i,
        	nav_dist_pm,
        	nav_dist_im,
        	nav_dist_om,
        	nav_alt_speed_p,
        	nav_alt_speed_i,
        	nav_alt_speed_pm,
        	nav_alt_speed_im,
        	nav_alt_speed_om,
        	nav_alt_pos_p,
        	nav_alt_pos_i,
        	nav_alt_pos_pm,
        	nav_alt_pos_im,
        	nav_alt_pos_om;
    };
/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct quat_position_control_NAV_param_handles *nav, struct quat_position_control_UKF_param_handles *ukf);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct quat_position_control_NAV_param_handles *nav_handles, struct quat_position_control_NAV_params *nav_params,
		const struct quat_position_control_UKF_param_handles *ukf_handles, struct quat_position_control_UKF_params *ukf_params);


#endif //quat_pos_control_params_h_
