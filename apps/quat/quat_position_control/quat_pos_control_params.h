
#include <systemlib/param/param.h>

struct quat_position_control_params {
	float p;
	float i;
	float d;
};

struct quat_position_control_param_handles {
	param_t p;
	param_t i;
	param_t d;
};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct quat_position_control_param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct quat_position_control_param_handles *h, struct quat_position_control_params *p);
