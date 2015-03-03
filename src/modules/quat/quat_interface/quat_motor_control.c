/**
 * @file quat_motor_control.c
 * Implementation of quat motor control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <systemlib/err.h>
#include "quat_motor_control.h"
#include "mkMotorDriver.h"


int quat_write_motor_commands(const bool simulation, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {

	static int initialized = FALSE;
	static orb_advert_t pub = 0;
	static struct actuator_outputs_s outputs;
	if(!initialized){
		mkMotorDriver_init();
		memset(&outputs, 0, sizeof(outputs));
		pub = orb_advertise(ORB_ID(actuator_outputs), &outputs);
		initialized = TRUE;
	}
	const unsigned int min_motor_interval = 4900;
	static uint64_t last_motor_time = 0;
	outputs.timestamp = hrt_absolute_time();
	outputs.output[0] = motor1;
	outputs.output[1] = motor2;
	outputs.output[2] = motor3;
	outputs.output[3] = motor4;
	outputs.noutputs = 4;
	uint64_t currentTime = hrt_absolute_time();
	if (currentTime - last_motor_time > min_motor_interval) {
		int ret = OK;
		if(simulation){
			ret |= mkMotorDriver_set_11bit_pwm(MOT1, 0);
			ret |= mkMotorDriver_set_11bit_pwm(MOT2, 0);
			ret |= mkMotorDriver_set_11bit_pwm(MOT3, 0);
			ret |= mkMotorDriver_set_11bit_pwm(MOT4, 0);
		}
		else{
			ret |= mkMotorDriver_set_11bit_pwm(MOT1, motor1);
			ret |= mkMotorDriver_set_11bit_pwm(MOT2, motor2);
			ret |= mkMotorDriver_set_11bit_pwm(MOT3, motor3);
			ret |= mkMotorDriver_set_11bit_pwm(MOT4, motor4);
		}
		/* publish just written values */
		orb_publish(ORB_ID(actuator_outputs), pub, &outputs);
		last_motor_time = currentTime;
		return ret;
	} else {
		return -ERROR;
	}
}

void quat_mixing_and_output(const bool simulation, const struct actuator_controls_s *actuators) {

	float roll_control = actuators->control[0];
	float pitch_control = actuators->control[1];
	float yaw_control = actuators->control[2];
	float motor_thrust = actuators->control[3];

	//printf("AMO: Roll: %4.4f, Pitch: %4.4f, Yaw: %4.4f, Thrust: %4.4f\n",roll_control, pitch_control, yaw_control, motor_thrust);

	const float min_thrust = 0.03f;			/**< 3% minimum thrust */
	const float max_thrust = 1.0f;			/**< 100% max thrust */
	const float scaling = 2040.0f;			/**< 100% thrust equals a value of 2040. Max would be 2047 */
	const float min_gas = min_thrust * scaling;	/**< value range sent to motors, minimum */
	const float max_gas = max_thrust * scaling;	/**< value range sent to motors, maximum */

	/* initialize all fields to zero */
	uint16_t motor_pwm[4] = {0};
	float motor_calc[4] = {0};

	float output_band = 0.0f;
	float band_factor = 0.75f;
	const float startpoint_full_control = 0.25f;	/**< start full control at 25% thrust */
	float yaw_factor = 1.0f;

	if (motor_thrust <= min_thrust) {
		motor_thrust = min_thrust;
		output_band = 0.0f;
	} else if (motor_thrust < startpoint_full_control && motor_thrust > min_thrust) {
		output_band = band_factor * (motor_thrust - min_thrust);
	} else if (motor_thrust >= startpoint_full_control && motor_thrust < max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * startpoint_full_control;
	} else if (motor_thrust >= max_thrust - band_factor * startpoint_full_control) {
		output_band = band_factor * (max_thrust - motor_thrust);
	}

	//add the yaw, nick and roll components to the basic thrust //TODO:this should be done by the mixer

	// FRONT (MOTOR 1)
	motor_calc[0] = motor_thrust + pitch_control  - yaw_control;

	// RIGHT (MOTOR 2)
	motor_calc[1] = motor_thrust - roll_control + yaw_control;

	// BACK (MOTOR 3)
	motor_calc[2] = motor_thrust - pitch_control - yaw_control;

	// LEFT (MOTOR 4)
	motor_calc[3] = motor_thrust + roll_control + yaw_control;

	// if we are not in the output band
	if (!(motor_calc[0] < motor_thrust + output_band && motor_calc[0] > motor_thrust - output_band
	      && motor_calc[1] < motor_thrust + output_band && motor_calc[1] > motor_thrust - output_band
	      && motor_calc[2] < motor_thrust + output_band && motor_calc[2] > motor_thrust - output_band
	      && motor_calc[3] < motor_thrust + output_band && motor_calc[3] > motor_thrust - output_band)) {

		yaw_factor = 0.5f;
		// FRONT (MOTOR 1)
		motor_calc[0] = motor_thrust + pitch_control - yaw_control * yaw_factor;

		// RIGHT (MOTOR 2)
		motor_calc[1] = motor_thrust - roll_control + yaw_control * yaw_factor;

		// BACK (MOTOR 3)
		motor_calc[2] = motor_thrust - pitch_control - yaw_control * yaw_factor;

		// LEFT (MOTOR 4)
		motor_calc[3] = motor_thrust + roll_control + yaw_control * yaw_factor;
	}

	for (int i = 0; i < 4; i++) {
		//check for limits
		if (motor_calc[i] < motor_thrust - output_band) {
			motor_calc[i] = motor_thrust - output_band;
		}

		if (motor_calc[i] > motor_thrust + output_band) {
			motor_calc[i] = motor_thrust + output_band;
		}
	}

	/* set the motor values */

	/* scale up from 0..1 to 10..512) */
	motor_pwm[0] = (uint16_t) (motor_calc[0] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[1] = (uint16_t) (motor_calc[1] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[2] = (uint16_t) (motor_calc[2] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[3] = (uint16_t) (motor_calc[3] * ((float)max_gas - min_gas) + min_gas);

	/* Keep motors spinning while armed and prevent overflows */

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] > 0) ? motor_pwm[0] : 10;
	motor_pwm[1] = (motor_pwm[1] > 0) ? motor_pwm[1] : 10;
	motor_pwm[2] = (motor_pwm[2] > 0) ? motor_pwm[2] : 10;
	motor_pwm[3] = (motor_pwm[3] > 0) ? motor_pwm[3] : 10;

	/* Failsafe logic - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] <= scaling) ? motor_pwm[0] : scaling;
	motor_pwm[1] = (motor_pwm[1] <= scaling) ? motor_pwm[1] : scaling;
	motor_pwm[2] = (motor_pwm[2] <= scaling) ? motor_pwm[2] : scaling;
	motor_pwm[3] = (motor_pwm[3] <= scaling) ? motor_pwm[3] : scaling;

	/* send motors via UART */
	quat_write_motor_commands(simulation, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}
