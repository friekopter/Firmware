

#
# Makefile to build quat interface
#

MODULE_COMMAND		 = quat_interface
MODULE_PRIORITY	 = SCHED_PRIORITY_MAX-25
MODULE_STACKSIZE	 = 2048

SRCS				= mkMotorDriver.cpp \
						quat_interface.c \
						quat_motor_control.c
