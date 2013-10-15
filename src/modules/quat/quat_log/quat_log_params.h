/*
 * quat_log_params.h
 *
 *  Created on: Oct 9, 2013
 *      Author: fludwig
 */

#ifndef QUAT_LOG_PARAMS_H_
#define QUAT_LOG_PARAMS_H_


#include <systemlib/param/param.h>
#include <systemlib/visibility.h>

    struct quat_log_params {
    	int32_t q_log_interval;
    };


    struct quat_log_param_handles {
    	param_t q_log_interval;
    };

    __BEGIN_DECLS
    /**
     * Initialize all parameter handles and values
     *
     */
    __EXPORT int quat_log_parameters_init(struct quat_log_param_handles *paramHandles);

    /**
     * Update all parameters
     *
     */
    __EXPORT int quat_log_parameters_update(const struct quat_log_param_handles *handles, struct quat_log_params *params);
    __END_DECLS

#endif /* QUAT_LOG_PARAMS_H_ */
