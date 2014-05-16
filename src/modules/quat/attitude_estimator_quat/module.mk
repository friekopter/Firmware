

MODULE_COMMAND		= attitude_estimator_quat
MODULE_PRIORITY	 	= SCHED_PRIORITY_DEFAULT
#MODULE_STACKSIZE	= 4096

SRCS		 	= attitude_estimator_quat_main.c \
					attitude_estimator_quat_params.c \
					quat.c

INCLUDE_DIRS	 += $(MAVLINK_SRC)/include/mavlink