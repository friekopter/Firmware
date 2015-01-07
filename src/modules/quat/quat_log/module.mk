#
# Makefile to build quat log
#

MODULE_COMMAND		 	= quat_log
MODULE_PRIORITY	 		= SCHED_PRIORITY_MAX-30
MODULE_STACKSIZE		= 2048
SRCS					= quat_log_main.c \
						quat_log.c \
						quat_log_params.c

EXTRACFLAGS = -Wframe-larger-than=1080 -Wno-shadow -Wno-pointer-arith