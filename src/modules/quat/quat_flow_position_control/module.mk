#
# Makefile to build quat position control
#

MODULE_COMMAND		 = quat_flow_pos_control
MODULE_PRIORITY	     = SCHED_PRIORITY_MAX-25
MODULE_STACKSIZE	 = 2048

SRCS				= nav_flow_ukf.c \
						nav_flow.c \
						quat_flow_pos_control.c 

INCLUDE_DIRS	 += ../utils
INCLUDE_DIRS	 += ../quat_position_control


EXTRACFLAGS = -Wno-shadow