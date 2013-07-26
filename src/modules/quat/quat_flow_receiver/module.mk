#
# Makefile to build quat position control
#

MODULE_COMMAND		 = quat_flow_receiver
MODULE_PRIORITY	 = SCHED_PRIORITY_DEFAULT
MODULE_STACKSIZE	 = 1024

SRCS				= quat_flow_receiver.c 

INCLUDE_DIRS	 += ../utils
INCLUDE_DIRS	 += $(MAVLINK_SRC)/include/mavlink