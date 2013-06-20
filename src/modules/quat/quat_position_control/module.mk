#
# Makefile to build quat position control
#

MODULE_COMMAND		 = quat_pos_control
MODULE_PRIORITY	 = SCHED_PRIORITY_MAX-25
MODULE_STACKSIZE	 = 2048

SRCS				= algebra.c \
						nav_ukf.c \
						nav.c \
						position_control.c \
						quat_pos_control_params.c \
						quat_pos_control.c \
						srcdkf.c

INCLUDE_DIRS	 += ../utils