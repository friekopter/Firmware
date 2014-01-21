#include <nuttx/config.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <float.h>

#include <nuttx/analog/adc.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>

#include <systemlib/ppm_decode.h>
#include <systemlib/airspeed.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <quat/utils/aq_math.h>

#include <mathlib/CMSIS/Include/arm_math.h>

#define GYRO_HEALTH_COUNTER_LIMIT_ERROR 20   /* 40 ms downtime at 500 Hz update rate   */
#define ACC_HEALTH_COUNTER_LIMIT_ERROR  20   /* 40 ms downtime at 500 Hz update rate   */
#define MAGN_HEALTH_COUNTER_LIMIT_ERROR 100  /* 1000 ms downtime at 100 Hz update rate  */
#define BARO_HEALTH_COUNTER_LIMIT_ERROR 50   /* 500 ms downtime at 100 Hz update rate  */
#define ADC_HEALTH_COUNTER_LIMIT_ERROR  10   /* 100 ms downtime at 100 Hz update rate  */

#define GYRO_HEALTH_COUNTER_LIMIT_OK 5
#define ACC_HEALTH_COUNTER_LIMIT_OK  5
#define MAGN_HEALTH_COUNTER_LIMIT_OK 5
#define BARO_HEALTH_COUNTER_LIMIT_OK 5
#define ADC_HEALTH_COUNTER_LIMIT_OK  5

#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	11

#define BAT_VOL_INITIAL 0.f
#define BAT_VOL_LOWPASS_1 0.99f
#define BAT_VOL_LOWPASS_2 0.01f
#define VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS 3.5f

#define RATE_CALIB_SAMPLES	30
#define IMU_STATIC_STD		0.05f
#define IMU_ROOM_TEMP		20.0f

#define PPM_INPUT_TIMEOUT_INTERVAL	50000 /**< 50 ms timeout / 20 Hz */

#define limit_minus_one_to_one(arg) (arg < -1.0f) ? -1.0f : ((arg > 1.0f) ? 1.0f : arg)

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int quat_sensors_main(int argc, char *argv[]);

class Quat_Sensors
{
public:
	/**
	 * Constructor
	 */
	Quat_Sensors();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Quat_Sensors();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	static const unsigned _rc_max_chan_count = RC_CHANNELS_MAX;	/**< maximum number of r/c channels we handle */

	hrt_abstime	_ppm_last_valid;		/**< last time we got a valid ppm signal */

	/**
	 * Gather and publish PPM input data.
	 */
	void		ppm_poll();

	/* XXX should not be here - should be own driver */
	int 		_fd_adc;			/**< ADC driver handle */
	hrt_abstime	_last_adc;			/**< last time we took input from the ADC */

	bool 		_task_should_exit;		/**< if true, sensor task should exit */
	int 		_quat_sensors_task;			/**< task handle for sensor task */

	bool		_hil_enabled;			/**< if true, HIL is active */
	bool		_publishing;			/**< if true, we are publishing sensor data */

	int		_gyro_sub;			/**< raw gyro data subscription */
	int		_accel_sub;			/**< raw accel data subscription */
	int		_mag_sub;			/**< raw mag data subscription */
	int 		_rc_sub;			/**< raw rc channels data subscription */
	int		_baro_sub;			/**< raw baro data subscription */
	int		_vcontrol_mode_sub;			/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;			/**< notification of manual control updates */

	orb_advert_t	_sensor_pub;			/**< combined sensor data topic */
	orb_advert_t	_manual_control_pub;		/**< manual control signal topic */
	orb_advert_t	_rc_pub;			/**< raw r/c control topic */
	orb_advert_t	_battery_pub;			/**< battery status */
	orb_advert_t	_airspeed_pub;			/**< airspeed */
	orb_advert_t	_diff_pres_pub;			/**< differential_pressure */

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_gyo_perf;			/**< loop performance counter */
	perf_counter_t	_acc_perf;			/**< loop performance counter */
	perf_counter_t	_mag_perf;			/**< loop performance counter */

	struct rc_channels_s _rc;			/**< r/c channel data */
	struct battery_status_s _battery_status;	/**< battery status */
	struct baro_report _barometer;			/**< barometer data */
	struct differential_pressure_s _diff_pres;
	struct airspeed_s _airspeed;

	float temp; /**< temperature for sensor correction */
	float temp2;/**< temperature² for sensor correction */
	float temp3;/**< temperature³ for sensor correction */

	struct {
		float min[_rc_max_chan_count];
		float trim[_rc_max_chan_count];
		float max[_rc_max_chan_count];
		float rev[_rc_max_chan_count];
		float dz[_rc_max_chan_count];
		// float ex[_rc_max_chan_count];
		float scaling_factor[_rc_max_chan_count];

		float gyro_bias[3];
		float gyro_bias1[3];
		float gyro_bias2[3];
		float gyro_bias3[3];
		float gyro_align_xy;
		float gyro_align_xz;
		float gyro_align_yx;
		float gyro_align_yz;
		float gyro_align_zx;
		float gyro_align_zy;
		float gyro_scale[3];

		float acc_bias[3];
		float acc_bias1[3];
		float acc_bias2[3];
		float acc_bias3[3];
		float acc_align_xy;
		float acc_align_xz;
		float acc_align_yx;
		float acc_align_yz;
		float acc_align_zx;
		float acc_align_zy;
		float acc_scale[3];
		float acc_scale1[3];
		float acc_scale2[3];
		float acc_scale3[3];

		float mag_bias[3];
		float mag_bias1[3];
		float mag_bias2[3];
		float mag_bias3[3];
		float mag_align_xy;
		float mag_align_xz;
		float mag_align_yx;
		float mag_align_yz;
		float mag_align_zx;
		float mag_align_zy;
		float mag_scale[3];
		float mag_scale1[3];
		float mag_scale2[3];
		float mag_scale3[3];
		float mag_inclination;

		float frame_rotation_xx;
		float frame_rotation_xy;
		float frame_rotation_xz;
		float frame_rotation_yx;
		float frame_rotation_yy;
		float frame_rotation_yz;
		float frame_rotation_zx;
		float frame_rotation_zy;
		float frame_rotation_zz;

		int32_t gyo_sample_rate;
		int32_t gyo_poll_rate;
		int32_t acc_sample_rate;
		int32_t acc_poll_rate;
		int32_t gyo_dlpf_freq;

		int diff_pres_offset_pa;
		float diff_pres_analog_enabled;

		int rc_type;

		int rc_map_roll;
		int rc_map_pitch;
		int rc_map_yaw;
		int rc_map_throttle;

		int rc_map_mode_sw;
		int rc_map_return_sw;
		int rc_map_assisted_sw;
		int rc_map_mission_sw;

		int rc_map_flaps;

		int rc_map_aux1;
		int rc_map_aux2;
		int rc_map_aux3;
		int rc_map_aux4;
		int rc_map_aux5;

		float rc_scale_roll;
		float rc_scale_pitch;
		float rc_scale_yaw;
		float rc_scale_flaps;

		float battery_voltage_scaling;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min[_rc_max_chan_count];
		param_t trim[_rc_max_chan_count];
		param_t max[_rc_max_chan_count];
		param_t rev[_rc_max_chan_count];
		param_t dz[_rc_max_chan_count];
		// param_t ex[_rc_max_chan_count];
		param_t rc_type;

		param_t rc_demix;

		param_t gyro_bias[3];
		param_t gyro_bias1[3];
		param_t gyro_bias2[3];
		param_t gyro_bias3[3];
		param_t gyro_align_xy;
		param_t gyro_align_xz;
		param_t gyro_align_yx;
		param_t gyro_align_yz;
		param_t gyro_align_zx;
		param_t gyro_align_zy;
		param_t gyro_scale[3];

		param_t acc_bias[3];
		param_t acc_bias1[3];
		param_t acc_bias2[3];
		param_t acc_bias3[3];
		param_t acc_align_xy;
		param_t acc_align_xz;
		param_t acc_align_yx;
		param_t acc_align_yz;
		param_t acc_align_zx;
		param_t acc_align_zy;
		param_t acc_scale[3];
		param_t acc_scale1[3];
		param_t acc_scale2[3];
		param_t acc_scale3[3];

		param_t mag_bias[3];
		param_t mag_bias1[3];
		param_t mag_bias2[3];
		param_t mag_bias3[3];
		param_t mag_align_xy;
		param_t mag_align_xz;
		param_t mag_align_yx;
		param_t mag_align_yz;
		param_t mag_align_zx;
		param_t mag_align_zy;
		param_t mag_scale[3];
		param_t mag_scale1[3];
		param_t mag_scale2[3];
		param_t mag_scale3[3];
		param_t mag_inclination;

		param_t frame_rotation_xx;
		param_t frame_rotation_xy;
		param_t frame_rotation_xz;
		param_t frame_rotation_yx;
		param_t frame_rotation_yy;
		param_t frame_rotation_yz;
		param_t frame_rotation_zx;
		param_t frame_rotation_zy;
		param_t frame_rotation_zz;

		param_t gyo_sample_rate;
		param_t gyo_poll_rate;
		param_t acc_sample_rate;
		param_t acc_poll_rate;
		param_t gyo_dlpf_freq;

		param_t diff_pres_offset_pa;
		param_t diff_pres_analog_enabled;

		param_t rc_map_roll;
		param_t rc_map_pitch;
		param_t rc_map_yaw;
		param_t rc_map_throttle;

		param_t rc_map_mode_sw;
		param_t rc_map_return_sw;
		param_t rc_map_assisted_sw;
		param_t rc_map_mission_sw;

		param_t rc_map_flaps;

		param_t rc_map_aux1;
		param_t rc_map_aux2;
		param_t rc_map_aux3;
		param_t rc_map_aux4;
		param_t rc_map_aux5;

		param_t rc_scale_roll;
		param_t rc_scale_pitch;
		param_t rc_scale_yaw;
		param_t rc_scale_flaps;

		param_t battery_voltage_scaling;
	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do accel-related initialisation.
	 */
	void		accel_init(uint16_t sampleRate, uint16_t pollRate);

	/**
	 * Do gyro-related initialisation.
	 */
	void		gyro_init(uint16_t sampleRate, uint16_t pollRate, uint16_t dlpfCutoff);

	/**
	 * Do mag-related initialisation.
	 */
	void		mag_init();

	/**
	 * Do baro-related initialisation.
	 */
	void		baro_init();

	/**
	 * Do adc-related initialisation.
	 */
	void		adc_init();

	/**
	 * Poll the accelerometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		accel_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the gyro for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		gyro_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the magnetometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		mag_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the barometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		baro_poll(struct sensor_combined_s &raw);

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll(struct sensor_combined_s &raw);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));

	/**
	 * Correct the gyro values by the temperature
	 */
	void	correctGyroMeasurement(struct  gyro_report &gyro_report);

	/**
	 * Correct the acc values by the temperature
     */
	void	correctAccMeasurement(struct accel_report &acc_report);

	/**
	 * Correct the mag values by the temperature
     */
	void	correctMagMeasurement(struct mag_report &mag_report);

	/**
	 * Calibrate gyro measurements
	 */
	void	gyro_calibrate();
};

namespace quat_sensors
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Quat_Sensors	*g_quat_sensors;
}

Quat_Sensors::Quat_Sensors() :
	_ppm_last_valid(0),

	_fd_adc(-1),
	_last_adc(0),

	_task_should_exit(false),
	_quat_sensors_task(-1),
	_hil_enabled(false),
	_publishing(true),

/* subscriptions */
	_gyro_sub(-1),
	_accel_sub(-1),
	_mag_sub(-1),
	_rc_sub(-1),
	_baro_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_sensor_pub(-1),
	_manual_control_pub(-1),
	_rc_pub(-1),
	_battery_pub(-1),
	_airspeed_pub(-1),
	_diff_pres_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "quat sensor task update")),
	_gyo_perf(perf_alloc(PC_INTERVAL, "quat sensor gyo event interval")),
	_acc_perf(perf_alloc(PC_INTERVAL, "quat sensor acc event interval")),
	_mag_perf(perf_alloc(PC_INTERVAL, "quat sensor mag event interval"))
{

	/* basic r/c parameters */
	for (unsigned i = 0; i < _rc_max_chan_count; i++) {
		char nbuf[16];

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		_parameter_handles.min[i] = param_find(nbuf);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		_parameter_handles.trim[i] = param_find(nbuf);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		_parameter_handles.max[i] = param_find(nbuf);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		_parameter_handles.rev[i] = param_find(nbuf);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		_parameter_handles.dz[i] = param_find(nbuf);

	}

	_parameter_handles.rc_type = param_find("RC_TYPE");

	/* mandatory input switched, mapped to channels 1-4 per default */
	_parameter_handles.rc_map_roll 	= param_find("RC_MAP_ROLL");
	_parameter_handles.rc_map_pitch = param_find("RC_MAP_PITCH");
	_parameter_handles.rc_map_yaw 	= param_find("RC_MAP_YAW");
	_parameter_handles.rc_map_throttle = param_find("RC_MAP_THROTTLE");

	/* mandatory mode switches, mapped to channel 5 and 6 per default */
	_parameter_handles.rc_map_mode_sw = param_find("RC_MAP_MODE_SW");
	_parameter_handles.rc_map_return_sw = param_find("RC_MAP_RETURN_SW");

	_parameter_handles.rc_map_flaps = param_find("RC_MAP_FLAPS");

	/* optional mode switches, not mapped per default */
	_parameter_handles.rc_map_assisted_sw = param_find("RC_MAP_ASSIST_SW");
	_parameter_handles.rc_map_mission_sw = param_find("RC_MAP_MISSIO_SW");

	_parameter_handles.rc_map_aux1 = param_find("RC_MAP_AUX1");
	_parameter_handles.rc_map_aux2 = param_find("RC_MAP_AUX2");
	_parameter_handles.rc_map_aux3 = param_find("RC_MAP_AUX3");
	_parameter_handles.rc_map_aux4 = param_find("RC_MAP_AUX4");
	_parameter_handles.rc_map_aux5 = param_find("RC_MAP_AUX5");

	_parameter_handles.rc_scale_roll = param_find("RC_SCALE_ROLL");
	_parameter_handles.rc_scale_pitch = param_find("RC_SCALE_PITCH");
	_parameter_handles.rc_scale_yaw = param_find("RC_SCALE_YAW");
	_parameter_handles.rc_scale_flaps = param_find("RC_SCALE_FLAPS");

	/* gyro offsets */
	_parameter_handles.gyro_scale[0] = param_find("IMU_GYO_SCAL_X");
	_parameter_handles.gyro_scale[1] = param_find("IMU_GYO_SCAL_Y");
	_parameter_handles.gyro_scale[2] = param_find("IMU_GYO_SCAL_Z");
	_parameter_handles.gyro_bias[0] = param_find("IMU_GYO_BIAS_X");
	_parameter_handles.gyro_bias[1] = param_find("IMU_GYO_BIAS_Y");
	_parameter_handles.gyro_bias[2] = param_find("IMU_GYO_BIAS_Z");
	_parameter_handles.gyro_bias1[0] = param_find("IMU_GYO_BIAS1_X");
	_parameter_handles.gyro_bias1[1] = param_find("IMU_GYO_BIAS1_Y");
	_parameter_handles.gyro_bias1[2] = param_find("IMU_GYO_BIAS1_Z");
	_parameter_handles.gyro_bias2[0] = param_find("IMU_GYO_BIAS2_X");
	_parameter_handles.gyro_bias2[1] = param_find("IMU_GYO_BIAS2_Y");
	_parameter_handles.gyro_bias2[2] = param_find("IMU_GYO_BIAS2_Z");
	_parameter_handles.gyro_bias3[0] = param_find("IMU_GYO_BIAS3_X");
	_parameter_handles.gyro_bias3[1] = param_find("IMU_GYO_BIAS3_Y");
	_parameter_handles.gyro_bias3[2] = param_find("IMU_GYO_BIAS3_Z");
	_parameter_handles.gyro_align_xy = param_find("IMU_GYO_ALGN_XY");
	_parameter_handles.gyro_align_xz = param_find("IMU_GYO_ALGN_XZ");
	_parameter_handles.gyro_align_yx = param_find("IMU_GYO_ALGN_YX");
	_parameter_handles.gyro_align_yz = param_find("IMU_GYO_ALGN_YZ");
	_parameter_handles.gyro_align_zx = param_find("IMU_GYO_ALGN_ZX");
	_parameter_handles.gyro_align_zy = param_find("IMU_GYO_ALGN_ZY");

	/* accel offsets */
	_parameter_handles.acc_scale[0] = param_find("IMU_ACC_SCAL_X");
	_parameter_handles.acc_scale[1] = param_find("IMU_ACC_SCAL_Y");
	_parameter_handles.acc_scale[2] = param_find("IMU_ACC_SCAL_Z");
	_parameter_handles.acc_scale1[0] = param_find("IMU_ACC_SCAL1_X");
	_parameter_handles.acc_scale1[1] = param_find("IMU_ACC_SCAL1_Y");
	_parameter_handles.acc_scale1[2] = param_find("IMU_ACC_SCAL1_Z");
	_parameter_handles.acc_scale2[0] = param_find("IMU_ACC_SCAL2_X");
	_parameter_handles.acc_scale2[1] = param_find("IMU_ACC_SCAL2_Y");
	_parameter_handles.acc_scale2[2] = param_find("IMU_ACC_SCAL2_Z");
	_parameter_handles.acc_scale3[0] = param_find("IMU_ACC_SCAL3_X");
	_parameter_handles.acc_scale3[1] = param_find("IMU_ACC_SCAL3_Y");
	_parameter_handles.acc_scale3[2] = param_find("IMU_ACC_SCAL3_Z");
	_parameter_handles.acc_bias[0] = param_find("IMU_ACC_BIAS_X");
	_parameter_handles.acc_bias[1] = param_find("IMU_ACC_BIAS_Y");
	_parameter_handles.acc_bias[2] = param_find("IMU_ACC_BIAS_Z");
	_parameter_handles.acc_bias1[0] = param_find("IMU_ACC_BIAS1_X");
	_parameter_handles.acc_bias1[1] = param_find("IMU_ACC_BIAS1_Y");
	_parameter_handles.acc_bias1[2] = param_find("IMU_ACC_BIAS1_Z");
	_parameter_handles.acc_bias2[0] = param_find("IMU_ACC_BIAS2_X");
	_parameter_handles.acc_bias2[1] = param_find("IMU_ACC_BIAS2_Y");
	_parameter_handles.acc_bias2[2] = param_find("IMU_ACC_BIAS2_Z");
	_parameter_handles.acc_bias3[0] = param_find("IMU_ACC_BIAS3_X");
	_parameter_handles.acc_bias3[1] = param_find("IMU_ACC_BIAS3_Y");
	_parameter_handles.acc_bias3[2] = param_find("IMU_ACC_BIAS3_Z");
	_parameter_handles.acc_align_xy = param_find("IMU_ACC_ALGN_XY");
	_parameter_handles.acc_align_xz = param_find("IMU_ACC_ALGN_XZ");
	_parameter_handles.acc_align_yx = param_find("IMU_ACC_ALGN_YX");
	_parameter_handles.acc_align_yz = param_find("IMU_ACC_ALGN_YZ");
	_parameter_handles.acc_align_zx = param_find("IMU_ACC_ALGN_ZX");
	_parameter_handles.acc_align_zy = param_find("IMU_ACC_ALGN_ZY");

	/* mag offsets */
	_parameter_handles.mag_scale[0] = param_find("IMU_MAG_SCAL_X");
	_parameter_handles.mag_scale[1] = param_find("IMU_MAG_SCAL_Y");
	_parameter_handles.mag_scale[2] = param_find("IMU_MAG_SCAL_Z");
	_parameter_handles.mag_scale1[0] = param_find("IMU_MAG_SCAL1_X");
	_parameter_handles.mag_scale1[1] = param_find("IMU_MAG_SCAL1_Y");
	_parameter_handles.mag_scale1[2] = param_find("IMU_MAG_SCAL1_Z");
	_parameter_handles.mag_scale2[0] = param_find("IMU_MAG_SCAL2_X");
	_parameter_handles.mag_scale2[1] = param_find("IMU_MAG_SCAL2_Y");
	_parameter_handles.mag_scale2[2] = param_find("IMU_MAG_SCAL2_Z");
	_parameter_handles.mag_scale3[0] = param_find("IMU_MAG_SCAL3_X");
	_parameter_handles.mag_scale3[1] = param_find("IMU_MAG_SCAL3_Y");
	_parameter_handles.mag_scale3[2] = param_find("IMU_MAG_SCAL3_Z");
	_parameter_handles.mag_bias[0] = param_find("IMU_MAG_BIAS_X");
	_parameter_handles.mag_bias[1] = param_find("IMU_MAG_BIAS_Y");
	_parameter_handles.mag_bias[2] = param_find("IMU_MAG_BIAS_Z");
	_parameter_handles.mag_bias1[0] = param_find("IMU_MAG_BIAS1_X");
	_parameter_handles.mag_bias1[1] = param_find("IMU_MAG_BIAS1_Y");
	_parameter_handles.mag_bias1[2] = param_find("IMU_MAG_BIAS1_Z");
	_parameter_handles.mag_bias2[0] = param_find("IMU_MAG_BIAS2_X");
	_parameter_handles.mag_bias2[1] = param_find("IMU_MAG_BIAS2_Y");
	_parameter_handles.mag_bias2[2] = param_find("IMU_MAG_BIAS2_Z");
	_parameter_handles.mag_bias3[0] = param_find("IMU_MAG_BIAS3_X");
	_parameter_handles.mag_bias3[1] = param_find("IMU_MAG_BIAS3_Y");
	_parameter_handles.mag_bias3[2] = param_find("IMU_MAG_BIAS3_Z");
	_parameter_handles.mag_align_xy = param_find("IMU_MAG_ALGN_XY");
	_parameter_handles.mag_align_xz = param_find("IMU_MAG_ALGN_XZ");
	_parameter_handles.mag_align_yx = param_find("IMU_MAG_ALGN_YX");
	_parameter_handles.mag_align_yz = param_find("IMU_MAG_ALGN_YZ");
	_parameter_handles.mag_align_zx = param_find("IMU_MAG_ALGN_ZX");
	_parameter_handles.mag_align_zy = param_find("IMU_MAG_ALGN_ZY");
	_parameter_handles.mag_inclination = param_find("IMU_MAG_INCL");

	_parameter_handles.frame_rotation_xx = param_find("IMU_ROT_XX");
	_parameter_handles.frame_rotation_xy = param_find("IMU_ROT_XY");
	_parameter_handles.frame_rotation_xz = param_find("IMU_ROT_XZ");
	_parameter_handles.frame_rotation_yx = param_find("IMU_ROT_YX");
	_parameter_handles.frame_rotation_yy = param_find("IMU_ROT_YY");
	_parameter_handles.frame_rotation_yz = param_find("IMU_ROT_YZ");
	_parameter_handles.frame_rotation_zx = param_find("IMU_ROT_ZX");
	_parameter_handles.frame_rotation_zy = param_find("IMU_ROT_ZY");
	_parameter_handles.frame_rotation_zz = param_find("IMU_ROT_ZZ");

	_parameter_handles.gyo_poll_rate = param_find("IMU_GYO_RA_POLL");
	_parameter_handles.gyo_sample_rate = param_find("IMU_GYO_RA_SAMP");
	_parameter_handles.acc_poll_rate = param_find("IMU_ACC_RA_POLL");
	_parameter_handles.acc_sample_rate = param_find("IMU_ACC_RA_SAMP");
	_parameter_handles.gyo_dlpf_freq = param_find("IMU_GYO_DLPF");

	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");

	_parameter_handles.battery_voltage_scaling = param_find("BAT_V_SCALING");

	/* fetch initial parameter values */
	parameters_update();
}

Quat_Sensors::~Quat_Sensors()
{
	if (_quat_sensors_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_quat_sensors_task);
				break;
			}
		} while (_quat_sensors_task != -1);
	}

	quat_sensors::g_quat_sensors = nullptr;
}

int
Quat_Sensors::parameters_update()
{
	bool rc_valid = true;
    float tmpScaleFactor = 0.0f;
    float tmpRevFactor = 0.0f;

	/* rc values */
	for (unsigned int i = 0; i < RC_CHANNELS_MAX; i++) {

		param_get(_parameter_handles.min[i], &(_parameters.min[i]));
		param_get(_parameter_handles.trim[i], &(_parameters.trim[i]));
		param_get(_parameter_handles.max[i], &(_parameters.max[i]));
		param_get(_parameter_handles.rev[i], &(_parameters.rev[i]));
		param_get(_parameter_handles.dz[i], &(_parameters.dz[i]));

		tmpScaleFactor = (1.0f / ((_parameters.max[i] - _parameters.min[i]) / 2.0f) * _parameters.rev[i]);
		tmpRevFactor = tmpScaleFactor * _parameters.rev[i];

		/* handle blowup in the scaling factor calculation */
		if (!isfinite(tmpScaleFactor) ||
		    (tmpRevFactor < 0.000001f) ||
		    (tmpRevFactor > 0.2f) ) {
			warnx("RC chan %u not sane, scaling: %8.6f, rev: %d", i, tmpScaleFactor, (int)(_parameters.rev[i]));
			/* scaling factors do not make sense, lock them down */
			_parameters.scaling_factor[i] = 0.0f;
			rc_valid = false;
		}
        else {
            _parameters.scaling_factor[i] = tmpScaleFactor;
        }
	}


	/* handle wrong values */
	if (!rc_valid)
		warnx("WARNING     WARNING     WARNING\n\nRC CALIBRATION NOT SANE!\n\n");

	/* channel mapping */
	if (param_get(_parameter_handles.rc_map_roll, &(_parameters.rc_map_roll)) != OK) {
		warnx("Failed getting roll chan index");
	}

	if (param_get(_parameter_handles.rc_map_pitch, &(_parameters.rc_map_pitch)) != OK) {
		warnx("Failed getting pitch chan index");
	}

	if (param_get(_parameter_handles.rc_map_yaw, &(_parameters.rc_map_yaw)) != OK) {
		warnx("Failed getting yaw chan index");
	}

	if (param_get(_parameter_handles.rc_map_throttle, &(_parameters.rc_map_throttle)) != OK) {
		warnx("Failed getting throttle chan index");
	}

	if (param_get(_parameter_handles.rc_map_mode_sw, &(_parameters.rc_map_mode_sw)) != OK) {
		warnx("Failed getting mode sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_return_sw, &(_parameters.rc_map_return_sw)) != OK) {
		warnx("Failed getting return sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_assisted_sw, &(_parameters.rc_map_assisted_sw)) != OK) {
		warnx("Failed getting assisted sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_mission_sw, &(_parameters.rc_map_mission_sw)) != OK) {
		warnx("Failed getting mission sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_flaps, &(_parameters.rc_map_flaps)) != OK) {
		warnx("Failed getting flaps chan index");
	}

//	if (param_get(_parameter_handles.rc_map_offboard_ctrl_mode_sw, &(_parameters.rc_map_offboard_ctrl_mode_sw)) != OK) {
//		warnx("Failed getting offboard control mode sw chan index");
//	}

	param_get(_parameter_handles.rc_map_aux1, &(_parameters.rc_map_aux1));
	param_get(_parameter_handles.rc_map_aux2, &(_parameters.rc_map_aux2));
	param_get(_parameter_handles.rc_map_aux3, &(_parameters.rc_map_aux3));
	param_get(_parameter_handles.rc_map_aux4, &(_parameters.rc_map_aux4));
	param_get(_parameter_handles.rc_map_aux5, &(_parameters.rc_map_aux5));
	param_get(_parameter_handles.rc_scale_roll, &(_parameters.rc_scale_roll));
	param_get(_parameter_handles.rc_scale_pitch, &(_parameters.rc_scale_pitch));
	param_get(_parameter_handles.rc_scale_yaw, &(_parameters.rc_scale_yaw));
	param_get(_parameter_handles.rc_scale_flaps, &(_parameters.rc_scale_flaps));

	/* update RC function mappings */
	_rc.function[THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[YAW] = _parameters.rc_map_yaw - 1;

	_rc.function[MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[ASSISTED] = _parameters.rc_map_assisted_sw - 1;
	_rc.function[MISSION] = _parameters.rc_map_mission_sw - 1;

	_rc.function[FLAPS] = _parameters.rc_map_flaps - 1;

//	_rc.function[OFFBOARD_MODE] = _parameters.rc_map_offboard_ctrl_mode_sw - 1;

	_rc.function[AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[AUX_5] = _parameters.rc_map_aux5 - 1;

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
	param_get(_parameter_handles.diff_pres_analog_enabled, &(_parameters.diff_pres_analog_enabled));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(_parameter_handles.battery_voltage_scaling, &(_parameters.battery_voltage_scaling)) != OK) {
		warnx("Failed updating voltage scaling param");
	}

	param_get(_parameter_handles.mag_align_xy, &(_parameters.mag_align_xy));
	param_get(_parameter_handles.mag_align_xz, &(_parameters.mag_align_xz));
	param_get(_parameter_handles.mag_align_yx, &(_parameters.mag_align_yx));
	param_get(_parameter_handles.mag_align_yz, &(_parameters.mag_align_yz));
	param_get(_parameter_handles.mag_align_zx, &(_parameters.mag_align_zx));
	param_get(_parameter_handles.mag_align_zy, &(_parameters.mag_align_zy));
	param_get(_parameter_handles.mag_bias[0], &(_parameters.mag_bias[0]));
	param_get(_parameter_handles.mag_bias[1], &(_parameters.mag_bias[1]));
	param_get(_parameter_handles.mag_bias[2], &(_parameters.mag_bias[2]));
	param_get(_parameter_handles.mag_bias1[0], &(_parameters.mag_bias1[0]));
	param_get(_parameter_handles.mag_bias1[1], &(_parameters.mag_bias1[1]));
	param_get(_parameter_handles.mag_bias1[2], &(_parameters.mag_bias1[2]));
	param_get(_parameter_handles.mag_bias2[0], &(_parameters.mag_bias2[0]));
	param_get(_parameter_handles.mag_bias2[1], &(_parameters.mag_bias2[1]));
	param_get(_parameter_handles.mag_bias2[2], &(_parameters.mag_bias2[2]));
	param_get(_parameter_handles.mag_bias3[0], &(_parameters.mag_bias3[0]));
	param_get(_parameter_handles.mag_bias3[1], &(_parameters.mag_bias3[1]));
	param_get(_parameter_handles.mag_bias3[2], &(_parameters.mag_bias3[2]));
	param_get(_parameter_handles.mag_scale[0], &(_parameters.mag_scale[0]));
	param_get(_parameter_handles.mag_scale[1], &(_parameters.mag_scale[1]));
	param_get(_parameter_handles.mag_scale[2], &(_parameters.mag_scale[2]));
	param_get(_parameter_handles.mag_scale1[0], &(_parameters.mag_scale1[0]));
	param_get(_parameter_handles.mag_scale1[1], &(_parameters.mag_scale1[1]));
	param_get(_parameter_handles.mag_scale1[2], &(_parameters.mag_scale1[2]));
	param_get(_parameter_handles.mag_scale2[0], &(_parameters.mag_scale2[0]));
	param_get(_parameter_handles.mag_scale2[1], &(_parameters.mag_scale2[1]));
	param_get(_parameter_handles.mag_scale2[2], &(_parameters.mag_scale2[2]));
	param_get(_parameter_handles.mag_scale3[0], &(_parameters.mag_scale3[0]));
	param_get(_parameter_handles.mag_scale3[1], &(_parameters.mag_scale3[1]));
	param_get(_parameter_handles.mag_scale3[2], &(_parameters.mag_scale3[2]));

	param_get(_parameter_handles.acc_align_xy, &(_parameters.acc_align_xy));
	param_get(_parameter_handles.acc_align_xz, &(_parameters.acc_align_xz));
	param_get(_parameter_handles.acc_align_yx, &(_parameters.acc_align_yx));
	param_get(_parameter_handles.acc_align_yz, &(_parameters.acc_align_yz));
	param_get(_parameter_handles.acc_align_zx, &(_parameters.acc_align_zx));
	param_get(_parameter_handles.acc_align_zy, &(_parameters.acc_align_zy));
	param_get(_parameter_handles.acc_bias[0], &(_parameters.acc_bias[0]));
	param_get(_parameter_handles.acc_bias[1], &(_parameters.acc_bias[1]));
	param_get(_parameter_handles.acc_bias[2], &(_parameters.acc_bias[2]));
	param_get(_parameter_handles.acc_bias1[0], &(_parameters.acc_bias1[0]));
	param_get(_parameter_handles.acc_bias1[1], &(_parameters.acc_bias1[1]));
	param_get(_parameter_handles.acc_bias1[2], &(_parameters.acc_bias1[2]));
	param_get(_parameter_handles.acc_bias2[0], &(_parameters.acc_bias2[0]));
	param_get(_parameter_handles.acc_bias2[1], &(_parameters.acc_bias2[1]));
	param_get(_parameter_handles.acc_bias2[2], &(_parameters.acc_bias2[2]));
	param_get(_parameter_handles.acc_bias3[0], &(_parameters.acc_bias3[0]));
	param_get(_parameter_handles.acc_bias3[1], &(_parameters.acc_bias3[1]));
	param_get(_parameter_handles.acc_bias3[2], &(_parameters.acc_bias3[2]));
	param_get(_parameter_handles.acc_scale[0], &(_parameters.acc_scale[0]));
	param_get(_parameter_handles.acc_scale[1], &(_parameters.acc_scale[1]));
	param_get(_parameter_handles.acc_scale[2], &(_parameters.acc_scale[2]));
	param_get(_parameter_handles.acc_scale1[0], &(_parameters.acc_scale1[0]));
	param_get(_parameter_handles.acc_scale1[1], &(_parameters.acc_scale1[1]));
	param_get(_parameter_handles.acc_scale1[2], &(_parameters.acc_scale1[2]));
	param_get(_parameter_handles.acc_scale2[0], &(_parameters.acc_scale2[0]));
	param_get(_parameter_handles.acc_scale2[1], &(_parameters.acc_scale2[1]));
	param_get(_parameter_handles.acc_scale2[2], &(_parameters.acc_scale2[2]));
	param_get(_parameter_handles.acc_scale3[0], &(_parameters.acc_scale3[0]));
	param_get(_parameter_handles.acc_scale3[1], &(_parameters.acc_scale3[1]));
	param_get(_parameter_handles.acc_scale3[2], &(_parameters.acc_scale3[2]));

	param_get(_parameter_handles.gyro_align_xy, &(_parameters.gyro_align_xy));
	param_get(_parameter_handles.gyro_align_xz, &(_parameters.gyro_align_xz));
	param_get(_parameter_handles.gyro_align_yx, &(_parameters.gyro_align_yx));
	param_get(_parameter_handles.gyro_align_yz, &(_parameters.gyro_align_yz));
	param_get(_parameter_handles.gyro_align_zx, &(_parameters.gyro_align_zx));
	param_get(_parameter_handles.gyro_align_zy, &(_parameters.gyro_align_zy));
	param_get(_parameter_handles.gyro_bias[0], &(_parameters.gyro_bias[0]));
	param_get(_parameter_handles.gyro_bias[1], &(_parameters.gyro_bias[1]));
	param_get(_parameter_handles.gyro_bias[2], &(_parameters.gyro_bias[2]));
	param_get(_parameter_handles.gyro_bias1[0], &(_parameters.gyro_bias1[0]));
	param_get(_parameter_handles.gyro_bias1[1], &(_parameters.gyro_bias1[1]));
	param_get(_parameter_handles.gyro_bias1[2], &(_parameters.gyro_bias1[2]));
	param_get(_parameter_handles.gyro_bias2[0], &(_parameters.gyro_bias2[0]));
	param_get(_parameter_handles.gyro_bias2[1], &(_parameters.gyro_bias2[1]));
	param_get(_parameter_handles.gyro_bias2[2], &(_parameters.gyro_bias2[2]));
	param_get(_parameter_handles.gyro_bias3[0], &(_parameters.gyro_bias3[0]));
	param_get(_parameter_handles.gyro_bias3[1], &(_parameters.gyro_bias3[1]));
	param_get(_parameter_handles.gyro_bias3[2], &(_parameters.gyro_bias3[2]));
	param_get(_parameter_handles.gyro_scale[0], &(_parameters.gyro_scale[0]));
	param_get(_parameter_handles.gyro_scale[1], &(_parameters.gyro_scale[1]));
	param_get(_parameter_handles.gyro_scale[2], &(_parameters.gyro_scale[2]));

	param_get(_parameter_handles.frame_rotation_xx, &(_parameters.frame_rotation_xx));
	param_get(_parameter_handles.frame_rotation_xy, &(_parameters.frame_rotation_xy));
	param_get(_parameter_handles.frame_rotation_xz, &(_parameters.frame_rotation_xz));
	param_get(_parameter_handles.frame_rotation_yx, &(_parameters.frame_rotation_yx));
	param_get(_parameter_handles.frame_rotation_yy, &(_parameters.frame_rotation_yy));
	param_get(_parameter_handles.frame_rotation_yz, &(_parameters.frame_rotation_yz));
	param_get(_parameter_handles.frame_rotation_zx, &(_parameters.frame_rotation_zx));
	param_get(_parameter_handles.frame_rotation_zy, &(_parameters.frame_rotation_zy));
	param_get(_parameter_handles.frame_rotation_zz, &(_parameters.frame_rotation_zz));

	param_get(_parameter_handles.gyo_poll_rate, &(_parameters.gyo_poll_rate));
	param_get(_parameter_handles.gyo_sample_rate, &(_parameters.gyo_sample_rate));
	param_get(_parameter_handles.acc_poll_rate, &(_parameters.acc_poll_rate));
	param_get(_parameter_handles.acc_sample_rate, &(_parameters.acc_sample_rate));
	param_get(_parameter_handles.gyo_dlpf_freq, &(_parameters.gyo_dlpf_freq));

	return OK;
}

void
Quat_Sensors::accel_init(uint16_t sampleRate, uint16_t pollRate)
{
	int	fd;

	fd = open(ACCEL_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", ACCEL_DEVICE_PATH);
		errx(1, "FATAL: no accelerometer found");

	} else {
		/* set the accel internal sampling rate up to 200Hz */
		ioctl(fd, ACCELIOCSSAMPLERATE, sampleRate);
		//warn("set acc sample rate %lu", sampleRate);

		/* set the driver to poll at 200Hz */
		ioctl(fd, SENSORIOCSPOLLRATE, pollRate);
		//warn("set acc poll rate %lu", pollRate);

		warnx("using system accel");
		close(fd);
	}
}

void
Quat_Sensors::gyro_init(uint16_t sampleRate, uint16_t pollRate, uint16_t dlpfCutoff)
{
	int	fd;

	fd = open(GYRO_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", GYRO_DEVICE_PATH);
		errx(1, "FATAL: no gyro found");

	} else {
		/* set the gyro internal sampling rate up to at leat 500Hz */
		ioctl(fd, GYROIOCSSAMPLERATE, sampleRate);

		/* set the driver to poll at 200Hz */
		ioctl(fd, SENSORIOCSPOLLRATE, pollRate);

		/* set the cutoff lowpass filter frequency */
		ioctl(fd, SENSORIOCSDLPF, dlpfCutoff);

		warnx("using system gyro");
		close(fd);
	}
}

void
Quat_Sensors::mag_init()
{
	int	fd;
	int	ret;

	fd = open(MAG_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", MAG_DEVICE_PATH);
		errx(1, "FATAL: no magnetometer found");
	}

	/* try different mag sampling rates */


	ret = ioctl(fd, MAGIOCSSAMPLERATE, 150);
	if (ret == OK) {
		/* set the pollrate accordingly */
		ioctl(fd, SENSORIOCSPOLLRATE, 150);
	} else {
		ret = ioctl(fd, MAGIOCSSAMPLERATE, 100);
		/* if the slower sampling rate still fails, something is wrong */
		if (ret == OK) {
			/* set the driver to poll also at the slower rate */
			ioctl(fd, SENSORIOCSPOLLRATE, 100);
		} else {
			errx(1, "FATAL: mag sampling rate could not be set");
		}
	}

	close(fd);
}

void
Quat_Sensors::baro_init()
{
	int	fd;

	fd = open(BARO_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", BARO_DEVICE_PATH);
		warnx("No barometer found, ignoring");
	}

	/* set the driver to poll at 150Hz */
	ioctl(fd, SENSORIOCSPOLLRATE, 150);

	close(fd);
}

void
Quat_Sensors::adc_init()
{

	_fd_adc = open(ADC_DEVICE_PATH, O_RDONLY | O_NONBLOCK);

	if (_fd_adc < 0) {
		warn(ADC_DEVICE_PATH);
		warnx("FATAL: no ADC found");
	}
}

void
Quat_Sensors::accel_poll(struct sensor_combined_s &raw)
{
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		struct accel_report	accel_report;

		perf_count(_acc_perf);

		orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel_report);

		correctAccMeasurement(accel_report);

		raw.accelerometer_m_s2[0] = accel_report.x;
		raw.accelerometer_m_s2[1] = accel_report.y;
		raw.accelerometer_m_s2[2] = accel_report.z;

		raw.accelerometer_raw[0] = accel_report.x_raw;
		raw.accelerometer_raw[1] = accel_report.y_raw;
		raw.accelerometer_raw[2] = accel_report.z_raw;

		raw.accelerometer_counter++;
	}
}

void
Quat_Sensors::correctAccMeasurement(struct accel_report &acc_report)
{
	float x,y,z, a,b,c;
	// rates
	x = -(-acc_report.x + _parameters.acc_bias[0] + _parameters.acc_bias1[0]*temp + _parameters.acc_bias2[0]*temp2 + _parameters.acc_bias3[0]*temp3);
	y = +(+acc_report.y + _parameters.acc_bias[1] + _parameters.acc_bias1[1]*temp + _parameters.acc_bias2[1]*temp2 + _parameters.acc_bias3[1]*temp3);
	z = -(-acc_report.z + _parameters.acc_bias[2] + _parameters.acc_bias1[2]*temp + _parameters.acc_bias2[2]*temp2 + _parameters.acc_bias3[2]*temp3);

	a = x + y*_parameters.acc_align_xy + z*_parameters.acc_align_xz;
	b = x*_parameters.acc_align_yx + y + z*_parameters.acc_align_yz;
	c = x*_parameters.acc_align_zx + y*_parameters.acc_align_zy + z;

	a /= _parameters.acc_scale[0] + _parameters.acc_scale1[0]*temp + _parameters.acc_scale2[0]*temp2 + _parameters.acc_scale3[0]*temp3;
	b /= _parameters.acc_scale[1] + _parameters.acc_scale1[1]*temp + _parameters.acc_scale2[1]*temp2 + _parameters.acc_scale3[1]*temp3;
	c /= _parameters.acc_scale[2] + _parameters.acc_scale1[2]*temp + _parameters.acc_scale2[2]*temp2 + _parameters.acc_scale3[2]*temp3;
/*
	acc_report.x = a;
	acc_report.y = b;
	acc_report.z = c;*/

	acc_report.x = a * _parameters.frame_rotation_xx + b * _parameters.frame_rotation_xy + c * _parameters.frame_rotation_xz;
	acc_report.y = a * _parameters.frame_rotation_yx + b * _parameters.frame_rotation_yy + c * _parameters.frame_rotation_yz;
	acc_report.z = a * _parameters.frame_rotation_zx + b * _parameters.frame_rotation_zy + c * _parameters.frame_rotation_zz;
}

void
Quat_Sensors::correctMagMeasurement(struct mag_report &mag_report)
{
	float x,y,z, a,b,c;
	// rates
	x = +(+mag_report.x + _parameters.mag_bias[0] + _parameters.mag_bias1[0]*temp + _parameters.mag_bias2[0]*temp2 + _parameters.mag_bias3[0]*temp3);
	y = +(+mag_report.y + _parameters.mag_bias[1] + _parameters.mag_bias1[1]*temp + _parameters.mag_bias2[1]*temp2 + _parameters.mag_bias3[1]*temp3);
	z = -(+mag_report.z + _parameters.mag_bias[2] + _parameters.mag_bias1[2]*temp + _parameters.mag_bias2[2]*temp2 + _parameters.mag_bias3[2]*temp3);

	a = x + y*_parameters.mag_align_xy + z*_parameters.mag_align_xz;
	b = x*_parameters.mag_align_yx + y + z*_parameters.mag_align_yz;
	c = x*_parameters.mag_align_zx + y*_parameters.mag_align_zy + z;

	a /= _parameters.mag_scale[0] + _parameters.mag_scale1[0]*temp + _parameters.mag_scale2[0]*temp2 + _parameters.mag_scale3[0]*temp3;
	b /= _parameters.mag_scale[1] + _parameters.mag_scale1[1]*temp + _parameters.mag_scale2[1]*temp2 + _parameters.mag_scale3[1]*temp3;
	c /= _parameters.mag_scale[2] + _parameters.mag_scale1[2]*temp + _parameters.mag_scale2[2]*temp2 + _parameters.mag_scale3[2]*temp3;
/*
	mag_report.x = a;
	mag_report.y = b;
	mag_report.z = c;
*/
	mag_report.x = a * _parameters.frame_rotation_xx + b * _parameters.frame_rotation_xy + c * _parameters.frame_rotation_xz;
	mag_report.y = a * _parameters.frame_rotation_yx + b * _parameters.frame_rotation_yy + c * _parameters.frame_rotation_yz;
	mag_report.z = a * _parameters.frame_rotation_zx + b * _parameters.frame_rotation_zy + c * _parameters.frame_rotation_zz;
}

void
Quat_Sensors::correctGyroMeasurement(struct  gyro_report &gyro_report)
{
	float x,y,z, a,b,c;
	// rates
	x = +(+gyro_report.x + _parameters.gyro_bias[0] + _parameters.gyro_bias1[0]*temp + _parameters.gyro_bias2[0]*temp2 + _parameters.gyro_bias3[0]*temp3);
	y = -(-gyro_report.y + _parameters.gyro_bias[1] + _parameters.gyro_bias1[1]*temp + _parameters.gyro_bias2[1]*temp2 + _parameters.gyro_bias3[1]*temp3);
	z = -(-gyro_report.z + _parameters.gyro_bias[2] + _parameters.gyro_bias1[2]*temp + _parameters.gyro_bias2[2]*temp2 + _parameters.gyro_bias3[2]*temp3);

	a = x + y*_parameters.gyro_align_xy + z*_parameters.gyro_align_xz;
	b = x*_parameters.gyro_align_yx + y + z*_parameters.gyro_align_yz;
	c = x*_parameters.gyro_align_zx + y*_parameters.gyro_align_zy + z;

	a /= _parameters.gyro_scale[0];
	b /= _parameters.gyro_scale[1];
	c /= _parameters.gyro_scale[2];

	/*gyro_report.x = a;
		gyro_report.y = b;
		gyro_report.z = c;*/

	gyro_report.x = a * _parameters.frame_rotation_xx + b * _parameters.frame_rotation_xy + c * _parameters.frame_rotation_xz;
	gyro_report.y = a * _parameters.frame_rotation_yx + b * _parameters.frame_rotation_yy + c * _parameters.frame_rotation_yz;
	gyro_report.z = a * _parameters.frame_rotation_zx + b * _parameters.frame_rotation_zy + c * _parameters.frame_rotation_zz;
}

void
Quat_Sensors::gyro_poll(struct sensor_combined_s &raw)
{
	bool gyro_updated;
	orb_check(_gyro_sub, &gyro_updated);

	if (gyro_updated) {
		perf_count(_gyo_perf);
		struct gyro_report	gyro_report;

		orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &gyro_report);

		if(!raw.gyro_counter % 100){
			temp = gyro_report.temperature - IMU_ROOM_TEMP;
			temp2 = temp*temp;
			temp3 = temp2*temp;
		}

		correctGyroMeasurement(gyro_report);

		raw.gyro_rad_s[0] = gyro_report.x;
		raw.gyro_rad_s[1] = gyro_report.y;
		raw.gyro_rad_s[2] = gyro_report.z;

		raw.gyro_raw[0] = gyro_report.x_raw;
		raw.gyro_raw[1] = gyro_report.y_raw;
		raw.gyro_raw[2] = gyro_report.z_raw;

		raw.gyro_counter++;
	}
}

void
Quat_Sensors::mag_poll(struct sensor_combined_s &raw)
{
	bool mag_updated;
	orb_check(_mag_sub, &mag_updated);

	if (mag_updated) {
		struct mag_report	mag_report;

		perf_count(_mag_perf);

		orb_copy(ORB_ID(sensor_mag), _mag_sub, &mag_report);

		correctMagMeasurement(mag_report);

		raw.magnetometer_ga[0] = mag_report.x;
		raw.magnetometer_ga[1] = mag_report.y;
		raw.magnetometer_ga[2] = mag_report.z;

		raw.magnetometer_raw[0] = mag_report.x_raw;
		raw.magnetometer_raw[1] = mag_report.y_raw;
		raw.magnetometer_raw[2] = mag_report.z_raw;

		raw.magnetometer_counter++;
	}
}

void
Quat_Sensors::baro_poll(struct sensor_combined_s &raw)
{
	bool baro_updated;
	orb_check(_baro_sub, &baro_updated);

	if (baro_updated) {

		orb_copy(ORB_ID(sensor_baro), _baro_sub, &_barometer);

		raw.baro_pres_mbar = _barometer.pressure; // Pressure in mbar
		raw.baro_alt_meter = _barometer.altitude; // Altitude in meters
		raw.baro_temp_celcius = _barometer.temperature; // Temperature in degrees celcius

		raw.baro_counter++;
	}
}

void
Quat_Sensors::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	/* Check HIL state if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);

		/* switching from non-HIL to HIL mode */
		//printf("[sensors] Vehicle mode: %i \t AND: %i, HIL: %i\n", vstatus.mode, vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED, hil_enabled);
		if (vcontrol_mode.flag_system_hil_enabled && !_hil_enabled) {
			_hil_enabled = true;
			_publishing = false;

			/* switching from HIL to non-HIL mode */

		} else if (!_publishing && !_hil_enabled) {
			_hil_enabled = false;
			_publishing = true;
		}
	}
}

void
Quat_Sensors::parameter_update_poll(bool forced)
{
	bool param_updated;

	/* Check if any parameter has changed */
	orb_check(_params_sub, &param_updated);

	if (param_updated || forced) {
		/* read from param to clear updated flag */
		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);
		/* update parameters */
		parameters_update();
#if 0
		printf("CH0: RAW MAX: %d MIN %d S: %d MID: %d FUNC: %d\n", (int)_parameters.max[0], (int)_parameters.min[0], (int)(_rc.chan[0].scaling_factor * 10000), (int)(_rc.chan[0].mid), (int)_rc.function[0]);
		printf("CH1: RAW MAX: %d MIN %d S: %d MID: %d FUNC: %d\n", (int)_parameters.max[1], (int)_parameters.min[1], (int)(_rc.chan[1].scaling_factor * 10000), (int)(_rc.chan[1].mid), (int)_rc.function[1]);
		printf("MAN: %d %d\n", (int)(_rc.chan[0].scaled * 100), (int)(_rc.chan[1].scaled * 100));
		fflush(stdout);
		usleep(5000);
#endif
	}
}

void
Quat_Sensors::adc_poll(struct sensor_combined_s &raw)
{

	/* rate limit to 100 Hz */
	if (hrt_absolute_time() - _last_adc >= 10000) {
		/* make space for a maximum of eight channels */
		struct adc_msg_s buf_adc[8];
		/* read all channels available */
		int ret = read(_fd_adc, &buf_adc, sizeof(buf_adc));

		for (unsigned i = 0; i < sizeof(buf_adc) / sizeof(buf_adc[0]); i++) {
			
			if (ret >= (int)sizeof(buf_adc[0])) {

				/* Save raw voltage values */
				if (i < (sizeof(raw.adc_voltage_v)) / sizeof(raw.adc_voltage_v[0])) {
					 raw.adc_voltage_v[i] = buf_adc[i].am_data / (4096.0f / 3.3f);
				}

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_BATTERY_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					float voltage = (buf_adc[i].am_data * _parameters.battery_voltage_scaling);

					if (voltage > VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS) {

						/* one-time initialization of low-pass value to avoid long init delays */
						if (_battery_status.voltage_v < 3.0f) {
							_battery_status.voltage_v = voltage;
						}

						_battery_status.timestamp = hrt_absolute_time();
						_battery_status.voltage_v = (BAT_VOL_LOWPASS_1 * (_battery_status.voltage_v + BAT_VOL_LOWPASS_2 * voltage));;
						/* current and discharge are unknown */
						_battery_status.current_a = -1.0f;
						_battery_status.discharged_mah = -1.0f;

						/* announce the battery voltage if needed, just publish else */
						if (_battery_pub > 0) {
							orb_publish(ORB_ID(battery_status), _battery_pub, &_battery_status);

						} else {
							_battery_pub = orb_advertise(ORB_ID(battery_status), &_battery_status);
						}
					} 

				} else if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					float voltage = (float)(buf_adc[i].am_data ) * 3.3f / 4096.0f * 2.0f; //V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor
					 */
					if (voltage > 0.4f) {

						float diff_pres_pa = voltage * 1000.0f - _parameters.diff_pres_offset_pa; //for MPXV7002DP sensor

						_diff_pres.timestamp = hrt_absolute_time();
						_diff_pres.differential_pressure_pa = diff_pres_pa;
						_diff_pres.voltage = voltage;

						/* announce the airspeed if needed, just publish else */
						if (_diff_pres_pub > 0) {
							orb_publish(ORB_ID(differential_pressure), _diff_pres_pub, &_diff_pres);

						} else {
							_diff_pres_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_pres);
						}
					}
				}

				_last_adc = hrt_absolute_time();
			}
		}
	}
}

void
Quat_Sensors::ppm_poll()
{

	/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
	struct pollfd fds[1];
	fds[0].fd = _rc_sub;
	fds[0].events = POLLIN;
	/* check non-blocking for new data */
	int poll_ret = poll(fds, 1, 0);

	if (poll_ret > 0) {
		struct rc_input_values	rc_input;

		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		struct manual_control_setpoint_s manual_control;

		/* initialize to default values */
		manual_control.roll = NAN;
		manual_control.pitch = NAN;
		manual_control.yaw = NAN;
		manual_control.throttle = NAN;

		manual_control.mode_switch = NAN;
		manual_control.return_switch = NAN;
		manual_control.assisted_switch = NAN;
		manual_control.mission_switch = NAN;
//		manual_control.auto_offboard_input_switch = NAN;

		manual_control.flaps = NAN;
		manual_control.aux1 = NAN;
		manual_control.aux2 = NAN;
		manual_control.aux3 = NAN;
		manual_control.aux4 = NAN;
		manual_control.aux5 = NAN;

		/* require at least four channels to consider the signal valid */
		if (rc_input.channel_count < 4)
			return;

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > _rc_max_chan_count)
			channel_limit = _rc_max_chan_count;

		/* we are accepting this message */
		_ppm_last_valid = rc_input.timestamp;

		/* Read out values from raw message */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			if (rc_input.values[i] < _parameters.min[i])
				rc_input.values[i] = _parameters.min[i];
			if (rc_input.values[i] > _parameters.max[i])
				rc_input.values[i] = _parameters.max[i];

			/*
			 * 2) Scale around the mid point differently for lower and upper range.
			 *
			 * This is necessary as they don't share the same endpoints and slope.
			 *
			 * First normalize to 0..1 range with correct sign (below or above center),
			 * the total range is 2 (-1..1).
			 * If center (trim) == min, scale to 0..1, if center (trim) == max,
			 * scale to -1..0.
			 *
			 * As the min and max bounds were enforced in step 1), division by zero
			 * cannot occur, as for the case of center == min or center == max the if
			 * statement is mutually exclusive with the arithmetic NaN case.
			 *
			 * DO NOT REMOVE OR ALTER STEP 1!
			 */
			if (rc_input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
				_rc.chan[i].scaled = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(_parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

			} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
				_rc.chan[i].scaled = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(_parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

			} else {
				/* in the configured dead zone, output zero */
				_rc.chan[i].scaled = 0.0f;
			}

			_rc.chan[i].scaled *= _parameters.rev[i];

			/* handle any parameter-induced blowups */
			if (!isfinite(_rc.chan[i].scaled))
				_rc.chan[i].scaled = 0.0f;
		}

		_rc.chan_count = rc_input.channel_count;
		_rc.timestamp = rc_input.timestamp;

		manual_control.timestamp = rc_input.timestamp;

		/* roll input - rolling right is stick-wise and rotation-wise positive */
		manual_control.roll = limit_minus_one_to_one(_rc.chan[_rc.function[ROLL]].scaled);
		/*
		 * pitch input - stick down is negative, but stick down is pitching up (pos) in NED,
		 * so reverse sign.
		 */
		manual_control.pitch = limit_minus_one_to_one(-1.0f * _rc.chan[_rc.function[PITCH]].scaled);
		/* yaw input - stick right is positive and positive rotation */
		manual_control.yaw = limit_minus_one_to_one(_rc.chan[_rc.function[YAW]].scaled);
		/* throttle input */
		manual_control.throttle = _rc.chan[_rc.function[THROTTLE]].scaled;

		if (manual_control.throttle < 0.0f) manual_control.throttle = 0.0f;

		if (manual_control.throttle > 1.0f) manual_control.throttle = 1.0f;

		/* scale output */
		if (isfinite(_parameters.rc_scale_roll) && _parameters.rc_scale_roll > 0.0f) {
			manual_control.roll *= _parameters.rc_scale_roll;
		}

		if (isfinite(_parameters.rc_scale_pitch) && _parameters.rc_scale_pitch > 0.0f) {
			manual_control.pitch *= _parameters.rc_scale_pitch;
		}

		if (isfinite(_parameters.rc_scale_yaw) && _parameters.rc_scale_yaw > 0.0f) {
			manual_control.yaw *= _parameters.rc_scale_yaw;
		}

		/* mode switch input */
		manual_control.mode_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MODE]].scaled);

		/* land switch input */
		manual_control.return_switch = limit_minus_one_to_one(_rc.chan[_rc.function[RETURN]].scaled);

		/* assisted switch input */
		manual_control.assisted_switch = limit_minus_one_to_one(_rc.chan[_rc.function[ASSISTED]].scaled);

		/* mission switch input */
		manual_control.mission_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MISSION]].scaled);

		/* flaps */
		if (_rc.function[FLAPS] >= 0) {

			manual_control.flaps = limit_minus_one_to_one(_rc.chan[_rc.function[FLAPS]].scaled);

			if (isfinite(_parameters.rc_scale_flaps) && _parameters.rc_scale_flaps > 0.0f) {
				manual_control.flaps *= _parameters.rc_scale_flaps;
			}
		}

		if (_rc.function[MODE] >= 0) {
			manual_control.mode_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MODE]].scaled);
		}

		if (_rc.function[MISSION] >= 0) {
			manual_control.mission_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MISSION]].scaled);
		}

//		if (_rc.function[OFFBOARD_MODE] >= 0) {
//			manual_control.auto_offboard_input_switch = limit_minus_one_to_one(_rc.chan[_rc.function[OFFBOARD_MODE]].scaled);
//		}

		/* aux functions, only assign if valid mapping is present */
		if (_rc.function[AUX_1] >= 0) {
			manual_control.aux1 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_1]].scaled);
		}

		if (_rc.function[AUX_2] >= 0) {
			manual_control.aux2 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_2]].scaled);
		}

		if (_rc.function[AUX_3] >= 0) {
			manual_control.aux3 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_3]].scaled);
		}

		if (_rc.function[AUX_4] >= 0) {
			manual_control.aux4 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_4]].scaled);
		}

		if (_rc.function[AUX_5] >= 0) {
			manual_control.aux5 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_5]].scaled);
		}

		/* check if ready for publishing */
		if (_rc_pub > 0) {
			orb_publish(ORB_ID(rc_channels), _rc_pub, &_rc);

		} else {
			/* advertise the rc topic */
			_rc_pub = orb_advertise(ORB_ID(rc_channels), &_rc);
		}

		/* check if ready for publishing */
		if (_manual_control_pub > 0) {
			orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual_control);

		} else {
			_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);
		}
	}
}

void
Quat_Sensors::task_main_trampoline(int argc, char *argv[])
{
	quat_sensors::g_quat_sensors->task_main();
}

void
Quat_Sensors::task_main()
{

	/* inform about start */
	printf("[quat_sensors] Initializing..\n");
	fflush(stdout);

	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	parameter_update_poll(true /* forced */);

	/* start individual sensors */
	accel_init((uint16_t)_parameters.acc_sample_rate, (uint16_t)_parameters.acc_poll_rate);
	printf("[quat_sensors] Init gyro...\n");
	gyro_init((uint16_t)_parameters.gyo_sample_rate, (uint16_t)_parameters.gyo_poll_rate, (uint16_t)_parameters.gyo_dlpf_freq);
	printf("[quat_sensors] Init mag...\n");
	mag_init();
	printf("[quat_sensors] Init baro...\n");
	baro_init();
	printf("[quat_sensors] Init adc...\n");
	adc_init();
	fflush(stdout);
	/*
	 * do subscriptions
	 */
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);

	/* rate limit gyro to 250 Hz (the gyro signal is lowpassed accordingly earlier) */
	orb_set_interval(_gyro_sub, 4);

	/*
	 * do advertisements
	 */
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	raw.timestamp = hrt_absolute_time();
	raw.adc_voltage_v[0] = 0.0f;
	raw.adc_voltage_v[1] = 0.0f;
	raw.adc_voltage_v[2] = 0.0f;
	raw.adc_voltage_v[3] = 0.0f;

	memset(&_battery_status, 0, sizeof(_battery_status));
	_battery_status.voltage_v = BAT_VOL_INITIAL;

	/* get a set of initial values */
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(raw);
	baro_poll(raw);


	/* calibrate sensors */
	gyro_calibrate();

	/* advertise the sensor_combined topic and make the initial publication */
	_sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = _gyro_sub;
	fds[0].events = POLLIN;

	printf("[quat_sensors] Init finished...\n");
	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* check parameters for updates */
		parameter_update_poll();

		/* store the time closest to all measurements (this is bogus, sensor timestamps should be propagated...) */
		raw.timestamp = hrt_absolute_time();

		/* copy most recent sensor data */
		gyro_poll(raw);
		accel_poll(raw);
		mag_poll(raw);
		baro_poll(raw);

		/* check battery voltage */
		adc_poll(raw);

		/* Inform other processes that new data is available to copy */
		if (_publishing)
			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);

		/* Look for new r/c input data */
		ppm_poll();
		perf_end(_loop_perf);
	}

	printf("[quat_sensors] exiting.\n");

	_quat_sensors_task = -1;
	_exit(0);
}

int
Quat_Sensors::start()
{
	ASSERT(_quat_sensors_task == -1);

	/* start the task */
	_quat_sensors_task = task_spawn_cmd("quat_sensors_task",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_MAX - 5,
				   2048,
				   (main_t)&Quat_Sensors::task_main_trampoline,
				   nullptr);

	if (_quat_sensors_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
Quat_Sensors::gyro_calibrate()
{
    float32_t stdX, stdY, stdZ;
    float32_t x[RATE_CALIB_SAMPLES];
    float32_t y[RATE_CALIB_SAMPLES];
    float32_t z[RATE_CALIB_SAMPLES];
    int i = 0;
    int j = 0;
    float32_t meanRate[3];
	/* set offsets to zero */
	int fd = open(GYRO_DEVICE_PATH, 0);
	struct gyro_scale gscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale_null)) {
		warn("WARNING: failed to set scale / offsets for gyro");
	}
	close(fd);

	while(i <= 3 * RATE_CALIB_SAMPLES ||
			(stdX + stdY + stdZ) > IMU_STATIC_STD)
	{
		bool gyro_updated;
		orb_check(_gyro_sub, &gyro_updated);

		if (gyro_updated) {
			struct gyro_report	gyro_report;
			orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &gyro_report);
			x[j] = gyro_report.x;
			y[j] = gyro_report.y;
			z[j] = gyro_report.z;
			temp = gyro_report.temperature - IMU_ROOM_TEMP;
			temp2 = temp*temp;
			temp3 = temp2*temp;
			if (i >= RATE_CALIB_SAMPLES) {
			    arm_std_f32(x, RATE_CALIB_SAMPLES, &stdX);
			    arm_std_f32(y, RATE_CALIB_SAMPLES, &stdY);
			    arm_std_f32(z, RATE_CALIB_SAMPLES, &stdZ);
			}

			i++;
			j = (j + 1) % RATE_CALIB_SAMPLES;
		}
		usleep(10000);
	}

   	arm_mean_f32(x,RATE_CALIB_SAMPLES,&meanRate[0]);
   	arm_mean_f32(y,RATE_CALIB_SAMPLES,&meanRate[1]);
   	arm_mean_f32(z,RATE_CALIB_SAMPLES,&meanRate[2]);

    float rateBiasX = +(+meanRate[0] + _parameters.gyro_bias[0] + _parameters.gyro_bias1[0]*temp + _parameters.gyro_bias2[0]*temp2 + _parameters.gyro_bias3[0]*temp3);
    float rateBiasY = -(-meanRate[1] + _parameters.gyro_bias[1] + _parameters.gyro_bias1[1]*temp + _parameters.gyro_bias2[1]*temp2 + _parameters.gyro_bias3[1]*temp3);
    float rateBiasZ = -(-meanRate[2] + _parameters.gyro_bias[2] + _parameters.gyro_bias1[2]*temp + _parameters.gyro_bias2[2]*temp2 + _parameters.gyro_bias3[2]*temp3);

    // set offsets
	fd = open(GYRO_DEVICE_PATH, 0);
	struct gyro_scale gscale = {
		rateBiasX,
		1.0f,
		rateBiasY,
		1.0f,
		rateBiasZ,
		1.0f,
	};
	printf("[Quat Sensors]: Set gyro bias: %8.4f\t %8.4f\t %8.4f\n",rateBiasX,rateBiasY,rateBiasZ);

	if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale)) {
		warn("WARNING: failed to set scale / offsets for gyro");
	}
	close(fd);
}


int quat_sensors_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: quat_sensors {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (quat_sensors::g_quat_sensors != nullptr)
			errx(1, "quat_sensors task already running");

		quat_sensors::g_quat_sensors = new Quat_Sensors;

		if (quat_sensors::g_quat_sensors == nullptr)
			errx(1, "quat_sensors task alloc failed");

		if (OK != quat_sensors::g_quat_sensors->start()) {
			delete quat_sensors::g_quat_sensors;
			quat_sensors::g_quat_sensors = nullptr;
			err(1, "quat_sensors task start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (quat_sensors::g_quat_sensors == nullptr)
			errx(1, "quat_sensors task not running");

		delete quat_sensors::g_quat_sensors;
		quat_sensors::g_quat_sensors = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (quat_sensors::g_quat_sensors) {
			errx(0, "task is running");

		} else {
			errx(1, "task is not running");
		}
	}

	errx(1, "unrecognized command");
}

