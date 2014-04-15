/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Friedemann Ludwig
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hil_test_quat.c
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
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>


__EXPORT int hil_test_quat_main(int argc, char *argv[]);
int hil_test_thread_main(int argc, char *argv[]);

static int mavlink_fd;

static bool thread_should_exit = false;
static bool thread_running = false;
static int mavlink_task;

static float pressure_mbar = 1013.25f;

/* terminate MAVLink on user request - disabled by default */
static bool mavlink_link_termination_allowed = false;

mavlink_system_t mavlink_system = {100, 50, MAV_TYPE_QUADROTOR, 0, 0, 0}; // System ID, 1-255, Component/Subsystem ID, 1-255
static uint8_t chan = MAVLINK_COMM_0;
static mavlink_status_t status;
static struct vehicle_status_s quad_status;

/* pthreads */
static pthread_t receive_thread;
static pthread_t uorb_receive_thread;


// ORB registrations for data that comes from the testbed simulation by mavlink
// and has to be forwarded to the firmware by orb publishing
// Those are mainly sensor measurements
struct accel_report	accel_report;
orb_advert_t		accel_topic;
struct gyro_report	gyro_report;
orb_advert_t		gyro_topic;
struct mag_report	mag_report;
orb_advert_t		mag_topic;
struct sensor_combined_s sensors_raw_report;
static orb_advert_t		sensors_raw_topic;
static orb_advert_t stat_pub;
static orb_advert_t		manual_topic;
struct manual_control_setpoint_s manual_report;
static orb_advert_t		gps_topic;
static orb_advert_t		filtered_flow_pub;
struct filtered_bottom_flow_s flow_report;
struct vehicle_gps_position_s gps_report;


static int baudrate = 57600;

/* interface mode */
static enum {
	MAVLINK_INTERFACE_MODE_OFFBOARD,
	MAVLINK_INTERFACE_MODE_ONBOARD
} mavlink_link_mode = MAVLINK_INTERFACE_MODE_OFFBOARD;

// Mavlink subscription for data that comes from the board by orb registrations
// and goes to the testbed simulation by mavlink message
static struct orb_subscriptions {
	//int sensor_sub;
	int att_sub;
	int global_pos_sub;
	int actuators_sub;
	bool initialized;
} orb_subs = {
	//.sensor_sub = 0,
	.att_sub = 0,
	.global_pos_sub = 0,
	.actuators_sub = 0,
	.initialized = false
};


void handleMessage(mavlink_message_t *msg);
static void mavlink_update_system(void);

/**
 * Translate the custom state into standard mavlink modes and state.
 */
void get_mavlink_mode_and_state(const struct vehicle_status_s *c_status, const struct actuator_armed_s *actuator, uint8_t *mavlink_state, uint8_t *mavlink_mode);

int hil_test_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);

/**
 * Print the usage
 */
static void usage(const char *reason);


void mavlink_update_system(void)
{
	static bool initialized = false;
	param_t param_system_id;
	param_t param_component_id;
	param_t param_system_type;

	if (!initialized)
	{
		param_system_id = param_find("MAV_SYS_ID");
		param_component_id = param_find("MAV_COMP_ID");
		param_system_type = param_find("MAV_TYPE");
	}

	/* update system and component id */
	int32_t system_id;
	param_get(param_system_id, &system_id);
	if (system_id > 0 && system_id < 255)
	{
		mavlink_system.sysid = system_id;
	}

	int32_t component_id;
	param_get(param_component_id, &component_id);
	if (component_id > 0 && component_id < 255)
	{
		mavlink_system.compid = component_id;
	}

	int32_t system_type;
	param_get(param_system_type, &system_type);
	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END)
	{
		mavlink_system.type = system_type;
	}
}

/**
 * Receive data from UART.
 */
static void *receiveloop(void *arg)
{
	int uart_fd = *((int*)arg);

	const int timeout = 1000;
	uint8_t ch;

	mavlink_message_t msg;

	prctl(PR_SET_NAME, "hil_test_quat uart rcv", getpid());

	while (!thread_should_exit) {

		struct pollfd fds[] = { { .fd = uart_fd, .events = POLLIN } };

		if (poll(fds, 1, timeout) > 0) {
			/* non-blocking read until buffer is empty */
			int nread = 0;

			do {
				nread = read(uart_fd, &ch, 1);

				if (mavlink_parse_char(chan, ch, &msg, &status)) { //parse the char
					/* handle generic messages and commands */
					handleMessage(&msg);
				}
			} while (nread > 0);
		}
	}

	return NULL;
}

/**
 * Listen for uORB topics and send via MAVLink.
 *
 * This pthread performs a blocking wait on selected
 * uORB topics and sends them via MAVLink to other
 * vehicles or a ground control station.
 */
static void *uorb_receiveloop(void *arg)
{
	/* obtain reference to task's subscriptions */
	struct orb_subscriptions *subs = (struct orb_subscriptions *)arg;

	/* Set thread name */
	prctl(PR_SET_NAME, "hil_test_quat orb rcv", getpid());

	union {
		struct vehicle_attitude_s att;
		struct vehicle_global_velocity_setpoint_s global;
		struct actuator_outputs_s actuators;
	} buf;

	/* --- ATTITUDE VALUE --- */
	/* subscribe to ORB for attitude */
	subs->att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/* --- GLOBAL POS VALUE --- */
	orb_subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	/* --- ACTUATOR CONTROL VALUE --- */
	subs->actuators_sub = orb_subscribe(ORB_ID_VEHICLE_CONTROLS);

	struct pollfd fds[3] = {
		{ .fd = subs->att_sub, .events = POLLIN },
		{ .fd = subs->global_pos_sub, .events = POLLIN },
		{ .fd = subs->actuators_sub, .events = POLLIN }
	};
	/* all subscriptions initialized, return success */
	subs->initialized = true;
	// this will trigger main thread to update orb interval

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	/*
	 * Last sensor loop time
	 * some outputs are better timestamped
	 * with this "global" reference.
	 */
	uint64_t last_sensor_timestamp = 0;

	while (!thread_should_exit) {

		int poll_ret = poll(fds, 3, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			//mavlink_missionlib_send_gcs_string("[hil_test_quat] No telemetry data for 1 s");
		} else if (poll_ret < 0) {
			//mavlink_missionlib_send_gcs_string("[hil_test_quat] ERROR reading uORB data");
		} else {
			/* --- ATTITUDE VALUE --- */
			if (fds[0].revents & POLLIN) {
				/* copy attitude data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), subs->att_sub, &buf.att);
				/* send sensor values */
				mavlink_msg_attitude_send(chan, last_sensor_timestamp / 1000, buf.att.roll, buf.att.pitch, buf.att.yaw, buf.att.rollspeed, buf.att.pitchspeed, buf.att.yawspeed);
			}

			/* --- VEHICLE GLOBAL POSITION --- */
			if (fds[1].revents & POLLIN) {
				/* copy global position data into local buffer */
				orb_copy(ORB_ID(vehicle_global_position), subs->global_pos_sub, &buf.global);
				//uint64_t timestamp = buf.global.timestamp;
				//int32_t lat = buf.global.lat;
				//int32_t lon = buf.global.lon;
				//int32_t alt = (int32_t)(buf.global.alt*1000);
				//int32_t relative_alt = (int32_t)(buf.global.relative_alt * 1000.0f);
				int16_t vx = (int16_t)(buf.global.vx * 100.0f);
				int16_t vy = (int16_t)(buf.global.vy * 100.0f);
				int16_t vz = (int16_t)(buf.global.vz * 100.0f);
				/* heading in degrees * 10, from 0 to 36.000) */
				//uint16_t hdg = (buf.global.yaw / M_PI_F) * (180.0f * 10.0f) + (180.0f * 10.0f);

				//mavlink_msg_global_position_int_send(chan, timestamp / 1000, lat, lon, alt,
					//relative_alt, vx, vy, vz, hdg);
			}

			/* --- VEHICLE LOCAL POSITION --- */
			//if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
			//	orb_copy(ORB_ID(vehicle_local_position), subs->local_pos_sub, &buf.local);
			//	mavlink_msg_local_position_ned_send(chan, buf.local.timestamp / 1000, buf.local.x,
			//		buf.local.y, buf.local.z, buf.local.vx, buf.local.vy, buf.local.vz);
			//}

			/* --- ACTUATOR CONTROL --- */
			if (fds[2].revents & POLLIN) {
				orb_copy(ORB_ID_VEHICLE_CONTROLS, subs->actuators_sub, &buf.actuators);
				/* HIL message as per MAVLink spec */
				mavlink_msg_set_quad_motors_setpoint_send(chan,
					hrt_absolute_time(),
					(uint16_t)buf.actuators.output[0],
					(uint16_t)buf.actuators.output[1],
					(uint16_t)buf.actuators.output[2],
					(uint16_t)buf.actuators.output[3]);
			}
		}
	}

	return NULL;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_SCALED_IMU)
	{
		mavlink_scaled_imu_t imu;
		mavlink_msg_scaled_imu_decode(msg, &imu);
		uint64_t timestamp = hrt_absolute_time();
		/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
		accel_report.x = imu.xacc*9.81f/1000.0f;
		accel_report.y = imu.yacc*9.81f/1000.0f;
		accel_report.z = imu.zacc*9.81f/1000.0f;
		sensors_raw_report.accelerometer_m_s2[0] = imu.xacc*9.81f/1000.0f;
		sensors_raw_report.accelerometer_m_s2[1] = imu.yacc*9.81f/1000.0f;
		sensors_raw_report.accelerometer_m_s2[2] = imu.zacc*9.81f/1000.0f;
		sensors_raw_report.accelerometer_timestamp = timestamp;
		accel_report.timestamp = timestamp;
		gyro_report.x = imu.xgyro/1000.0f;
		gyro_report.y = imu.ygyro/1000.0f;
		gyro_report.z = imu.zgyro/1000.0f;
		sensors_raw_report.gyro_rad_s[0] = imu.xgyro/1000.0f;
		sensors_raw_report.gyro_rad_s[1] = imu.ygyro/1000.0f;
		sensors_raw_report.gyro_rad_s[2] = imu.zgyro/1000.0f;
		sensors_raw_report.gyro_timestamp = timestamp;
		gyro_report.timestamp = timestamp;
		mag_report.x = imu.xmag/1000.0f;
		mag_report.y = imu.ymag/1000.0f;
		mag_report.z = imu.zmag/1000.0f;
		sensors_raw_report.magnetometer_ga[0] = imu.xmag/1000.0f;
		sensors_raw_report.magnetometer_ga[1] = imu.ymag/1000.0f;
		sensors_raw_report.magnetometer_ga[2] = imu.zmag/1000.0f;
		sensors_raw_report.magnetometer_timestamp = timestamp;
		sensors_raw_report.baro_pres_mbar = pressure_mbar;
		sensors_raw_report.baro_timestamp= timestamp;
		mag_report.timestamp = timestamp;
		sensors_raw_report.timestamp = timestamp;
		/* advertise sensor topics */
		/* check if topic is advertised */
		if (accel_topic <= 0)
		{
			//accel_topic = orb_advertise(ORB_ID(sensor_accel), &accel_report);
			//gyro_topic = orb_advertise(ORB_ID(sensor_gyro), &gyro_report);
			//mag_topic = orb_advertise(ORB_ID(sensor_mag), &mag_report);
			sensors_raw_topic = orb_advertise(ORB_ID(sensor_combined), &sensors_raw_report);
		}
		else
		{
			/* and publish for subscribers */
			orb_publish(ORB_ID(sensor_accel), accel_topic, &accel_report);
			orb_publish(ORB_ID(sensor_gyro), gyro_topic, &gyro_report);
			orb_publish(ORB_ID(sensor_mag), mag_topic, &mag_report);
			orb_publish(ORB_ID(sensor_combined), sensors_raw_topic, &sensors_raw_report);
		}
	}
	else if (msg->msgid == MAVLINK_MSG_ID_RAW_PRESSURE)
	{
		mavlink_raw_pressure_t pressure;
		mavlink_msg_scaled_imu_decode(msg, &pressure);
		pressure_mbar = (float)pressure.press_abs / 10.0f;
		//printf("Pressure:%8.4f\n",pressure_mbar);

	}
	else if (msg->msgid == MAVLINK_MSG_ID_SET_MODE)
	{
		/* Set mode on request */
    	//  [15] mav mode (-1: preflight, 0:standby=manual disarmed, 1:armed manual, 2:stabilized, 3:auto)
		mavlink_set_mode_t mode;
		mavlink_msg_set_mode_decode(msg, &mode);
		uint8_t base_mode = mode.base_mode;
		switch(base_mode)
		{
		case MAV_MODE_PREFLIGHT:
			// Do nothing, only initial state
			break;
		case MAV_MODE_MANUAL_DISARMED:
			//if(quad_status.state_machine == SYSTEM_STATE_STANDBY) break;
			//quad_status.flag_system_armed = false;
			//quad_status.state_machine = SYSTEM_STATE_STANDBY;
			/* publish current state machine */
			//state_machine_publish(stat_pub, &quad_status, mavlink_fd);
			//publish_armed_status(&quad_status);
			break;
		case MAV_MODE_MANUAL_ARMED:
			//if(quad_status.state_machine == SYSTEM_STATE_STANDBY){
				// set to ground ready, otherwise we can't switch to manual
			//	quad_status.state_machine = SYSTEM_STATE_GROUND_READY;
			//}
			//update_state_machine_mode_request(stat_pub,&quad_status,mavlink_fd,base_mode);
			break;
		case MAV_MODE_GUIDED_ARMED:
			// set vector flight mode valid, otherwise no guided mode is possible
			//quad_status.flag_vector_flight_mode_ok = true;
			// This should work with the base_mode parameter, for some reason MAV_MODE_GUIDED_ARMED = 216 =  11011000
			//update_state_machine_mode_request(stat_pub,&quad_status,mavlink_fd,VEHICLE_MODE_FLAG_SAFETY_ARMED | VEHICLE_MODE_FLAG_GUIDED_ENABLED);
			break;
		case MAV_MODE_AUTO_ARMED:
			//update_state_machine_mode_request(stat_pub,&quad_status,mavlink_fd,base_mode);
			break;
		default:
			// unsupported state
			mavlink_log_critical(mavlink_fd, "Trying to set unknown state!");
			break;
		}
	}
	else if (msg->msgid == MAVLINK_MSG_ID_MANUAL_CONTROL)
	{
		if (manual_topic <= 0) {
			memset(&manual_report, 0, sizeof(manual_report));
			manual_topic = orb_advertise(ORB_ID(manual_control_setpoint), &manual_report);
		} else {
			mavlink_manual_control_t control;
			mavlink_msg_manual_control_decode(msg, &control);
			manual_report.roll = ((float)control.x)/1000.0f;
			manual_report.pitch = ((float)control.y)/1000.0f;
			manual_report.yaw = ((float)control.z)/1000.0f;
			manual_report.throttle = ((float)control.r)/1000.0f;
			orb_publish(ORB_ID(manual_control_setpoint), manual_topic, &manual_report);
		}
	}
	else if (msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT)
	{
		if (gps_topic <= 0) {
			memset(&gps_report, 0, sizeof(gps_report));
			gps_topic = orb_advertise(ORB_ID(vehicle_gps_position), &gps_report);
		} else {
			mavlink_gps_raw_int_t gps_data;
			mavlink_msg_gps_raw_int_decode(msg, &gps_data);
			gps_report.alt = gps_data.alt;
			gps_report.lat = gps_data.lat;
			gps_report.lon = gps_data.lon;
			gps_report.cAcc = 0;
			gps_report.cog_rad = 0;
			gps_report.eDop = 150.0f;
			gps_report.eph_m = 2.9f;
			gps_report.epv_m = 3.0f;
			gps_report.fix_type = 3;
			gps_report.hDop = gps_data.eph;
			gps_report.nDop = 150.0f;
			gps_report.p_variance_m = 0;
			gps_report.sAcc = 1.5f;
			gps_report.s_variance_m_s = 0;
			//gps_report.satellite_azimuth = 0;
			//gps_report.satellite_elevation = 0;
			gps_report.satellite_info_available = 0;
			//gps_report.satellite_prn = 0;
			//gps_report.satellite_snr = 0;
			//gps_report.satellite_used = 0;
			gps_report.satellites_visible = 6;
			gps_report.tDop = 150.0f;
			gps_report.time_gps_usec = 0;
			gps_report.timestamp_posdilution = 0;
			gps_report.timestamp_position = hrt_absolute_time();
			gps_report.timestamp_satellites = 0;
			gps_report.timestamp_time = 0;
			gps_report.timestamp_variance = 0;
			gps_report.timestamp_velocity = hrt_absolute_time();
			gps_report.vDop = gps_data.epv;
			gps_report.vel_d_m_s = ((float)((int8_t)gps_data.fix_type))/10.0f;
			gps_report.vel_e_m_s = ((float)((int16_t)gps_data.cog))/100.0f;
			gps_report.vel_m_s = 0;
			gps_report.vel_n_m_s = ((float)((int16_t)gps_data.vel))/100.0f;
			gps_report.vel_ned_valid = 1;
			orb_publish(ORB_ID(vehicle_gps_position), gps_topic, &gps_report);
		}
	}
	if (msg->msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		flow_report.timestamp = hrt_absolute_time();
		flow_report.vx = flow.flow_comp_m_x;
		flow_report.vy = flow.flow_comp_m_y;
		flow_report.sumx = ((float)flow.flow_x)/100.0f;//misuse of parameter for the distance in x direction
		flow_report.sumy = ((float)flow.flow_y)/100.0f;//misuse of parameter for the distance in y direction
/*		flow_report.flow_comp_x_m = flow.flow_comp_m_x;
		flow_report.flow_comp_y_m = flow.flow_comp_m_y;
		flow_report.ground_distance_m = flow.ground_distance;
		flow_report.quality = flow.quality;
		flow_report.sensor_id = flow.sensor_id;*/
		//printf("flowx:%8.4f\tflowy:%8.4f\n",flow_report.vx, flow_report.vy);
		if (filtered_flow_pub <= 0) {
			filtered_flow_pub = orb_advertise(ORB_ID(filtered_bottom_flow), &flow_report);
		}
		else {
			orb_publish(ORB_ID(filtered_bottom_flow), filtered_flow_pub, &flow_report);
		}
	}
}


int hil_test_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
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
		fprintf(stderr, "[hil_test_quat] ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n", baud);
		return -EINVAL;
	}

	/* open uart */
	printf("[hil_test_quat] UART is %s, baudrate is %d\n", uart_name, baud);
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	if (strcmp(uart_name, "/dev/ttyACM0") != OK) {
		/* Back up the original uart configuration to restore it after exit */
		if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
			fprintf(stderr, "[hil_test_quat] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
			close(uart);
			return -1;
		}

		/* Fill the struct for the new configuration */
		tcgetattr(uart, &uart_config);

		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			fprintf(stderr, "[hil_test_quat] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
			close(uart);
			return -1;
		}


		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
			fprintf(stderr, "[hil_test_quat] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
			close(uart);
			return -1;
		}

	} else {
		*is_usb = true;
	}

	return uart;
}

/**
 * hil_test_quat Protocol main function.
 */
int hil_test_thread_main(int argc, char *argv[])
{

	/* print welcome text */
	printf("[hil_test_quat] MAVLink v1.0 serial interface starting..\n");

	/* default values for arguments */
	char *uart_name = "/dev/ttyS0";
	baudrate = 57600;

	/* init structs */
	memset(&quad_status, 0, sizeof(quad_status));
	quad_status.system_type = VEHICLE_TYPE_QUADROTOR;

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "hil_test_quat" */

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
		} else if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--onboard") == 0) {
			mavlink_link_mode = MAVLINK_INTERFACE_MODE_ONBOARD;
		} else {
			usage("out of order or invalid argument");
			return 1;
		}
	}

	struct termios uart_config_original;

	bool usb_uart;

	uart = hil_test_open_uart(baudrate, uart_name, &uart_config_original, &usb_uart);

	if (uart < 0) {
		printf("[hil_test_quat] FAILED to open %s, terminating.\n", uart_name);
		goto exit_cleanup;
	}

	/* Flush UART */
	fflush(stdout);

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	/* Initialize system properties */
	mavlink_update_system();

	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);
	pthread_attr_setstacksize(&receiveloop_attr, 2048);
	pthread_create(&receive_thread, &receiveloop_attr, receiveloop, &uart);

	pthread_attr_t uorb_attr;
	pthread_attr_init(&uorb_attr);
	/* Set stack size, needs more than 8000 bytes */
	pthread_attr_setstacksize(&uorb_attr, 8192);
	// Listen for orb publishing and forward to mavlink
	pthread_create(&uorb_receive_thread, &uorb_attr, uorb_receiveloop, &orb_subs);

	uint16_t counter = 0;

	/* make sure all threads have registered their subscriptions */
	while (!orb_subs.initialized) {
		usleep(500);
	}

	/* all subscriptions are now active, set up initial guess about rate limits */
	if (baudrate >= 460800)
	{
		/* 200 Hz / 5 ms */
	}
	else if (baudrate >= 230400)
	{
		/* 200 Hz / 5 ms */
		orb_set_interval(orb_subs.actuators_sub, 10);
		orb_set_interval(orb_subs.att_sub, 1000);
		orb_set_interval(orb_subs.global_pos_sub, 100);
	}
	else if (baudrate >= 115200)
	{
		/* 50 Hz / 20 ms */
		orb_set_interval(orb_subs.actuators_sub, 50);
		orb_set_interval(orb_subs.att_sub, 1000);
		orb_set_interval(orb_subs.global_pos_sub, 100);
	}
	else if (baudrate >= 57600)
	{
		/* 10 Hz / 100 ms */
		orb_set_interval(orb_subs.actuators_sub, 200);
		orb_set_interval(orb_subs.att_sub, 1000);
		orb_set_interval(orb_subs.global_pos_sub, 200);
	}
	else
	{
		/* very low baud rate, limit to 1 Hz / 1000 ms */
		orb_set_interval(orb_subs.actuators_sub, 1000);
		orb_set_interval(orb_subs.att_sub, 1000);
		orb_set_interval(orb_subs.global_pos_sub, 1000);
	}

	/* advertise to ORB */
	stat_pub = orb_advertise(ORB_ID(vehicle_status), &quad_status);
	/* publish current state machine */
	//state_machine_publish(stat_pub, &quad_status, mavlink_fd);

	thread_running = true;

	/* arm counter to go off immediately */
	int lowspeed_counter = 10;

	while (!thread_should_exit) {

		/* get local and global position */
		//orb_copy(ORB_ID(actuator_armed), orb_subs.armed_sub, &armed);

		/* 1 Hz */
		if (lowspeed_counter == 10)
		{
			mavlink_update_system();
			lowspeed_counter = 0;
		}
		lowspeed_counter++;

		/* sleep quarter the time */
		usleep(25000);

		/* sleep quarter the time */
		usleep(25000);

		/* sleep 10 ms */
		usleep(10000);

		counter++;

		/* sleep 15 ms */
		usleep(15000);
	}

	/* wait for threads to complete */
	pthread_join(receive_thread, NULL);
	pthread_join(uorb_receive_thread, NULL);

	/* Reset the UART flags to original state */
	if (!usb_uart) {
		int termios_state;

		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config_original)) < 0) {
			fprintf(stderr, "[hil_test_quat] ERROR setting baudrate / termios config for %s (tcsetattr)\r\n", uart_name);
		}

		printf("[hil_test_quat] Restored original UART config\n");
	}

exit_cleanup:

	/* close uart */
	close(uart);

	/* close subscriptions */
	close(orb_subs.global_pos_sub);
	close(orb_subs.att_sub);
	close(orb_subs.actuators_sub);

	fflush(stdout);
	fflush(stderr);

	thread_running = false;
	printf("[hil_test_quat] Exiting..\n");
	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: hil_test_quat {start|stop|status} [-d <devicename>] [-b <baudrate>] [-e/--exit-allowed]\n\n");
	exit(1);
}

int hil_test_quat_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("hil_test_quat already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		mavlink_task = task_spawn_cmd("hil_test_quat",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  6000,
					  hil_test_thread_main,
					  (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\thil_test_quat app is running\n");
		} else {
			printf("\thil_test_quat app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

