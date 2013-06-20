#
# Makefile to build quat log
#

MODULE_COMMAND		 	= quat_log
MODULE_PRIORITY	 		= SCHED_PRIORITY_MAX-25
MODULE_STACKSIZE		= 2048
SRCS					= quat_log_main.c \
						quat_log.c

