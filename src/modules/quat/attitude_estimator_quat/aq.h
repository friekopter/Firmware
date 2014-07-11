
#ifndef _aq_h
#define _aq_h

#ifndef M_PI_F
#define M_PI_F			3.14159265f
#endif



#define AQ_CYC_PER_SEC		1000000		// originally calibrated by GPS timepulse

// these define where to get the data when the above event triggers
#define AQ_YAW			quatData.yaw*RAD_TO_DEG
#define AQ_PITCH		quatData.pitch
#define AQ_ROLL			quatData.roll
#define AQ_VIN			global_data.battery_voltage
#define AQ_TEMP			adcData.temperature
#define AQ_ALTITUDE		global_data.altitude
#define AQ_RANGE		rangeData.range
#define AQ_PRESSURE		global_data.pressure_bmp_si
#define AQ_PRES_ADJ		adcPressureAdjust
#define AQ_HRATEX		global_data.hgyros_si.x
#define AQ_HRATEY		global_data.hgyros_si.y
#define AQ_HRATEZ		global_data.hgyros_si.z
#define AQ_RATEX		global_data.gyros_si.x
#define AQ_RATEY		global_data.gyros_si.y
#define AQ_RATEZ		global_data.gyros_si.z
#define AQ_ACCX			global_data.accel_si.x
#define AQ_ACCY			global_data.accel_si.y
#define AQ_ACCZ			global_data.accel_si.z
#define AQ_MAGX			global_data.magnet_si.x
#define AQ_MAGY			global_data.magnet_si.y
#define AQ_MAGZ			global_data.magnet_si.z
#define AQ_TIMESTEP		quatData.timestep
#define AQ_LASTUPD		quatData.lastUpdate

extern volatile unsigned long counter;
extern volatile unsigned long minCycles;

#endif
