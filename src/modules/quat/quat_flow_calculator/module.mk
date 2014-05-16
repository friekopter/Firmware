#
# Makefile to build quat flow calculator
#

MODULE_COMMAND		 = quat_flow_calculator
MODULE_PRIORITY	     = SCHED_PRIORITY_MAX-25
#MODULE_STACKSIZE	 = 2048

SRCS				= quat_flow_calculator_main.c \
		  quat_flow_calculator_params.c
		  
INCLUDE_DIRS	 += ../utils