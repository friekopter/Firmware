#
# Makefile to build quat position control
#

MODULE_COMMAND		 = quat_pos_control
MODULE_PRIORITY	 = SCHED_PRIORITY_MAX-25
#MODULE_STACKSIZE	 = 2048

SRCS				= nav_ukf.c \
						nav.c \
						quat_pos_control.c 

INCLUDE_DIRS	 += ../utils