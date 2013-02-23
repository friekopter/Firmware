/**
 * @file quat_motor_control.h
 * Definition of quat motor control interface
 */

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>

/**
 * Write four motor commands to an already initialized port.
 *
 * Writing 0 stops a motor, values from 1-512 encode the full thrust range.
 * on some motor controller firmware revisions a minimum value of 10 is
 * required to spin the motors.
 */
int quat_write_motor_commands(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

/**
 * Mix motors and output actuators
 */
void quat_mixing_and_output(const struct actuator_controls_s *actuators);
