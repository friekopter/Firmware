
/**
 * @file quat_flow_receiver.c
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <v1.0/common/mavlink.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <commander/state_machine_helper.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/optical_flow.h>
#include <mavlink/mavlink_log.h>
#include <quat/utils/util.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#define QUAT_FLOW_SUM 10.0f
#define QUAT_FLOW_QUALITY_LIMIT 50

__EXPORT int quat_flow_receiver_main(int argc, char *argv[]);
int quat_flow_receiver_thread_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int mavlink_task;

/* terminate MAVLink on user request - disabled by default */
static bool mavlink_link_termination_allowed = false;

mavlink_system_t mavlink_system = {100, 50, MAV_TYPE_QUADROTOR, 0, 0, 0}; // System ID, 1-255, Component/Subsystem ID, 1-255
static uint8_t chan = MAVLINK_COMM_0;
static mavlink_status_t status;


// ORB registrations for data that will be sent
struct filtered_bottom_flow_s flow_result;
static orb_advert_t flow_pub;
struct optical_flow_s raw_flow_result;
static orb_advert_t raw_flow_pub;

static struct vehicle_attitude_s att;
int att_sub;

static int baudrate = 57600;

static int printcounter = 0;
static bool debug = false;

void handleMessage(mavlink_message_t *msg);

int quat_flow_receiver_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);

/**
 * Print the usage
 */
static void usage(const char *reason);

void handleMessage(mavlink_message_t *msg)
{
	static float absoluteDistanceEarthFrame[3] = {0.0f,0.0f,0.0f};
	static float absoluteDistanceBodyFrame[3] = {0.0f,0.0f,0.0f};
	if (msg->msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
	    static int8_t index = 0;
	    static float sumVX = 0.0f;
	    static float sumVY = 0.0f;
		hrt_abstime currentTime = hrt_absolute_time();

		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		raw_flow_result.flow_comp_x_m = flow.flow_comp_m_x;
		raw_flow_result.flow_comp_y_m = flow.flow_comp_m_y;
		raw_flow_result.flow_raw_x = flow.flow_x;
		raw_flow_result.flow_raw_y = flow.flow_y;
		raw_flow_result.ground_distance_m = flow.ground_distance;
		raw_flow_result.quality = flow.quality;
		raw_flow_result.sensor_id = flow.sensor_id;
		raw_flow_result.timestamp = currentTime;

		/* check if topic is advertised */
		if (raw_flow_pub <= 0) {
			raw_flow_pub = orb_advertise(ORB_ID(optical_flow), &raw_flow_result);
		} else {
			/* publish */
			orb_publish(ORB_ID(optical_flow), raw_flow_pub, &raw_flow_result);
		}

		if(flow.quality < QUAT_FLOW_QUALITY_LIMIT) {
			sumVX = 0.0f;
			sumVY = 0.0f;
			index = 0;
			absoluteDistanceEarthFrame[0] = 0.0f;
			absoluteDistanceEarthFrame[1] = 0.0f;
			absoluteDistanceEarthFrame[2] = 0.0f;
			absoluteDistanceBodyFrame[0] = 0.0f;
			absoluteDistanceBodyFrame[1] = 0.0f;
			absoluteDistanceBodyFrame[2] = 0.0f;
		}
		else {
			sumVX += flow.flow_comp_m_x;
			sumVY += flow.flow_comp_m_y;
			index++;
		}

		if(index >= QUAT_FLOW_SUM) {
			index = 0;
			float speedBody[3];
			float speedEarth[3];
			static hrt_abstime lastTime = 0;
			if (lastTime == 0) {
				lastTime = currentTime;
			}
			speedBody[0] = sumVX / QUAT_FLOW_SUM;
			speedBody[1] = sumVY / QUAT_FLOW_SUM;
			speedBody[2] = 0.0f;
			flow_result.vx = speedBody[0];
			flow_result.vy = speedBody[1];
			sumVX = 0.0f;
			sumVY = 0.0f;
			flow_result.timestamp = currentTime;
			bool updated;
			orb_check(att_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
			}
			if(!att.R_valid) {
				flow_result.sumx = 0.0f;
				flow_result.sumy = 0.0f;
			}
			else {
				// Rotate speed to earth frame
				utilRotateVecByMatrix2(speedEarth, speedBody, att.R);
				hrt_abstime dt = currentTime - lastTime;
				absoluteDistanceEarthFrame[0] += speedEarth[0] * (dt / 10e6f);
				absoluteDistanceEarthFrame[1] += speedEarth[1] * (dt / 10e6f);
				lastTime = currentTime;
				utilRotateVecByRevMatrix2(absoluteDistanceBodyFrame, absoluteDistanceEarthFrame, att.R);
				flow_result.sumx = absoluteDistanceBodyFrame[0];
				flow_result.sumy = absoluteDistanceBodyFrame[1];
			}
			/* check if topic is advertised */
			if (flow_pub <= 0) {
				flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &flow_result);
			} else {
				/* publish */
				orb_publish(ORB_ID(filtered_bottom_flow), flow_pub, &flow_result);
			}
		}
		if (debug == true && !(printcounter % 100))
		{
			printf("Flow Speed x:%8.4f\ty:%8.4f\n",(double)flow_result.vx,(double)flow_result.vy);
			printf("Flow Distance x:%8.4f\ty:%8.4f\n",(double)flow_result.sumx,(double)flow_result.sumy);
			printf("Flow Distance Earth n:%8.4f\te:%8.4f\n",(double)absoluteDistanceEarthFrame[0],(double)absoluteDistanceEarthFrame[1]);
			printf("Flow quality:%d\n",raw_flow_result.quality);
		}
		printcounter++;
	}
}


int quat_flow_receiver_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		fprintf(stderr, "[quat_flow_receiver] ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n", baud);
		return -EINVAL;
	}

	/* open uart */
	printf("[quat_flow_receiver] UART is %s, baudrate is %d\n", uart_name, baud);
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	if (strcmp(uart_name, "/dev/ttyACM0") != OK) {
		/* Back up the original uart configuration to restore it after exit */
		if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
			fprintf(stderr, "[quat_flow_receiver] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
			close(uart);
			return -1;
		}

		/* Fill the struct for the new configuration */
		tcgetattr(uart, &uart_config);

		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			fprintf(stderr, "[quat_flow_receiver] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
			close(uart);
			return -1;
		}


		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
			fprintf(stderr, "[quat_flow_receiver] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
			close(uart);
			return -1;
		}

	} else {
		*is_usb = true;
	}

	return uart;
}

/**
 * quat_flow_receiver Protocol main function.
 */
int quat_flow_receiver_thread_main(int argc, char *argv[])
{

	/* print welcome text */
	printf("[quat_flow_receiver] MAVLink v1.0 serial interface starting..\n");

	/* default values for arguments */
	char *uart_name = "/dev/ttyS2";
	baudrate = 115200;

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "quat_flow_receiver" */

		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			usage("");
			return 0;
		} else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];
				i++;
			} else {
				usage("missing argument for device (-d)");
				return 1;
			}
		} else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);
				i++;
			} else {
				usage("missing argument for baud rate (-b)");
				return 1;
			}
		} else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--exit-allowed") == 0) {
			mavlink_link_termination_allowed = true;
		}
		else {
			usage("out of order or invalid argument");
			return 1;
		}
	}

	struct termios uart_config_original;

	bool usb_uart;

	uart = quat_flow_receiver_open_uart(baudrate, uart_name, &uart_config_original, &usb_uart);

	if (uart < 0) {
		printf("[quat_flow_receiver] FAILED to open %s, terminating.\n", uart_name);
		goto exit_cleanup;
	}

	mavlink_msg_param_set_send(MAVLINK_COMM_0, 81, 50, "IMAGE_L_LIGHT", 1.0f, MAV_PARAM_TYPE_REAL32);

	/* advertise to ORB */
	flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &flow_result);
	thread_running = true;
	const int timeout = 1000;
	uint8_t ch;
	mavlink_message_t msg;

	memset(&att, 0, sizeof(att));
	att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);

	while (!thread_should_exit) {
		struct pollfd fds[] = { { .fd = uart, .events = POLLIN } };

		if (poll(fds, 1, timeout) > 0) {
			/* non-blocking read until buffer is empty */
			int nread = 0;

			do {
				nread = read(uart, &ch, 1);

				if (mavlink_parse_char(chan, ch, &msg, &status)) { //parse the char
					/* handle generic messages and commands */
					handleMessage(&msg);
				}
			} while (nread > 0);
		}
		else {
			printf("[quat_flow_receiver] Timeout poll");
		}
	}

	/* Reset the UART flags to original state */
	if (!usb_uart) {
		int termios_state;

		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config_original)) < 0) {
			fprintf(stderr, "[quat_flow_receiver] ERROR setting baudrate / termios config for %s (tcsetattr)\r\n", uart_name);
		}

		printf("[quat_flow_receiver] Restored original UART config\n");
	}

exit_cleanup:

	/* close uart */
	close(uart);

	fflush(stdout);
	fflush(stderr);

	thread_running = false;
	printf("[quat_flow_receiver] Exiting..\n");
	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: quat_flow_receiver {start|stop|status} [-d <devicename>] [-b <baudrate>] [-e/--exit-allowed]\n\n");
	exit(1);
}

int quat_flow_receiver_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");
	if (!strcmp(argv[1], "debug")){
		debug = true;
	}
	if (!strcmp(argv[1], "start") || !strcmp(argv[1], "debug")) {

		if (thread_running) {
			printf("quat_flow_receiver already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		mavlink_task = task_spawn("quat_flow_receiver",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  6000,
					  quat_flow_receiver_thread_main,
					  (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tquat_flow_receiver app is running\n");
		} else {
			printf("\tquat_flow_receiver app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

