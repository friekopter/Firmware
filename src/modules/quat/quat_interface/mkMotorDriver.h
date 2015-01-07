/*
 * mkMotorDriver.h
 *
 *  Created on: Jan 4, 2013
 *      Author: fludwig
 *
 *      Driver for the mikrokopter brushless control
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <nuttx/i2c.h>


#include <systemlib/visibility.h>
#include <arch/board/board.h>

#ifndef MKMOTORDRIVER_H_
#define MKMOTORDRIVER_H_

#define MOT_I2C_BUS_NUMBER				PX4_I2C_BUS_ESC
#define MOT_DEVICE_PATH					"/dev/mot"
#define MOT_READMODE_STATUS  0

#define MOT1 1
#define MOT2 2
#define MOT3 3
#define MOT4 4

// Mikrokopter default addresses
#define MOT1_I2C_SLAVE_ADDRESS			0x52	///< I2C slave address of I2C motor controller number 1
#define MOT2_I2C_SLAVE_ADDRESS			0x54	///< I2C slave address of I2C motor controller number 2
#define MOT3_I2C_SLAVE_ADDRESS			0x56	///< I2C slave address of I2C motor controller number 3
#define MOT4_I2C_SLAVE_ADDRESS			0x58	///< I2C slave address of I2C motor controller number 4

typedef struct
{
	uint8_t Current;  			// in 0.1 A steps, read back from BL
	uint8_t MaxPWM;   			// read back from BL is less than 255 if BL is in current limit
	int8_t  Temperature;		// old BL-Ctrl will return a 255 here, the new version the temp. in °C
} /*__attribute__((packed))*/ MotorData_t;


__BEGIN_DECLS

	void mkMotorDriver_init(void);
	/**
	 * @brief Function to set PWM value on one of the motors with 11 bit resolution
	 *
	 * This function sends the PWM value specified in pwm to the motor
	 * controller specified by mot_i2c_dev_addr.
	 *
	 * @param mot_number motor number
	 * @param pmw PWM value to be set (Duty Cycle) with 11 bit, that is 0 to 2047
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 */
	__EXPORT int mkMotorDriver_set_11bit_pwm(uint8_t motor_number, uint16_t pwm);


	/**
	 * Read and return motor state data
	 */
	MotorData_t mkMotorDriver_get_motor_data(uint8_t motor_number);

	int mkMotorDriver_read_motor_state(uint8_t motor_number);


	__END_DECLS

#endif /* MKMOTORDRIVER_H_ */
