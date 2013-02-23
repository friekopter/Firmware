/*
 * mkMotorDriver.cpp
 *
 *  Created on: Jan 4, 2013
 *      Author: fludwig
 */

#include "mkMotorDriver.h"
#include <drivers/device/i2c.h>

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

MotorData_t motor_1;
MotorData_t motor_2;
MotorData_t motor_3;
MotorData_t motor_4;


class MkMotorDriver  : public device::I2C {
public:
	MkMotorDriver(int bus);
	virtual ~MkMotorDriver();

	virtual int	init();

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
	int set_11bit_pwm(uint8_t motor_number, uint16_t pwm);

	int read_motor_state(uint8_t motor_number);

protected:
private:

};

MkMotorDriver *motor_driver;

MkMotorDriver::MkMotorDriver(int bus) :
I2C("MK_MOTOR", MOT_DEVICE_PATH, bus, 0, 500000)
{
	// TODO FL Set interrupt number
	// TODO Auto-generated constructor stub
}

MkMotorDriver::~MkMotorDriver() {
	// TODO Auto-generated destructor stub
}


int
MkMotorDriver::init()
{
	int ret = ERROR;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	return ret;
}

/**
 * @brief Function to set PWM value on one of the motors with 11 bit resolution
 *
 * This function sends the PWM value specified in pwm to the motor
 * controller specified by mot_i2c_dev_addr.
 *
 * @param mot_i2c_dev_addr I2C slave address of a motor controller. Possible values are: MOT1_I2C_SLAVE_ADDRESS, MOT2_I2C_SLAVE_ADDRESS, MOT3_I2C_SLAVE_ADDRESS or MOT4_I2C_SLAVE_ADDRESS
 * @param pmw PWM value to be set (Duty Cycle) with 11 bit, that is 0 to 2047
 * @return		OK if the transfer was successful, -errno
 *			otherwise.
 */
int MkMotorDriver::set_11bit_pwm(uint8_t motor_number, uint16_t pwm) {
	switch(motor_number) {
		case MOT1:
			set_address(MOT1_I2C_SLAVE_ADDRESS);
			break;
		case MOT2:
			set_address(MOT2_I2C_SLAVE_ADDRESS);
			break;
		case MOT3:
			set_address(MOT3_I2C_SLAVE_ADDRESS);
			break;
		case MOT4:
			set_address(MOT4_I2C_SLAVE_ADDRESS);
			break;
		default:
			break;
	}
	uint8_t byte1 = (pwm & 2040) >> 3;				// PWM value to be set upper bits
	uint8_t byte2 = (MOT_READMODE_STATUS << 3) | (pwm & 7);// PWM value to be set lower bits
	uint8_t cmd[] = { byte1, byte2 };

	return transfer(&cmd[0], 2, nullptr, 0);
}

/**
 * Read state information for the brushless controller.
 */
int MkMotorDriver::read_motor_state(uint8_t motor_number) {
	int result = ERROR;
	switch(motor_number) {
		case MOT1:
			set_address(MOT1_I2C_SLAVE_ADDRESS);
			result = transfer(nullptr, 0, (uint8_t*)&motor_1, 3);
			break;
		case MOT2:
			set_address(MOT2_I2C_SLAVE_ADDRESS);
			result = transfer(nullptr, 0, (uint8_t*)&motor_2, 3);
			break;
		case MOT3:
			set_address(MOT3_I2C_SLAVE_ADDRESS);
			result = transfer(nullptr, 0, (uint8_t*)&motor_3, 3);
			break;
		case MOT4:
			set_address(MOT4_I2C_SLAVE_ADDRESS);
			result = transfer(nullptr, 0, (uint8_t*)&motor_4, 3);
			break;
		default:
			break;
	};
	return result;
}

/**
 * Return previously read motor state data
 */
MotorData_t mkMotorDriver_get_motor_data(uint8_t motor_number) {
	switch(motor_number) {
		case MOT1:
			return motor_1;
			break;
		case MOT2:
			return motor_2;
			break;
		case MOT3:
			return motor_3;
			break;
		case MOT4:
			return motor_4;
			break;
		default:
			break;
	};
	// Should not happen
	MotorData_t result;
	return result;
}

int mkMotorDriver_read_motor_state(uint8_t motor_number) {
	return motor_driver->read_motor_state(motor_number);
}

void mkMotorDriver_init(void){
	motor_driver = new MkMotorDriver(MOT_I2C_BUS_NUMBER);
	motor_driver->init();
}

int mkMotorDriver_set_11bit_pwm(uint8_t motor_number, uint16_t pwm) {
	return motor_driver->set_11bit_pwm(motor_number, pwm);
}

