
MODULE_COMMAND		= quat_att_control
MODULE_PRIORITY	    = "SCHED_PRIORITY_MAX-15"
#MODULE_STACKSIZE	= 2048
SRCS				= quat_att_control_main.c \
					quat_att_control_params.c \
					quat_att_control.c
INCLUDE_DIRS	 += $(TOPDIR)/arch/arm/src/stm32 $(TOPDIR)/arch/arm/src/common

