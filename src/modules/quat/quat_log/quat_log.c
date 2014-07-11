/*
 * quat_log.c
 *
 *  Created on: Jun 2, 2013
 *      Author: fludwig
 */


#include "quat_log.h"
#include <string.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <quat/quat_flow_position_control/nav_flow_ukf.h>
#include <quat/utils/util.h>

logStruct_t logData __attribute__((section(".ccm")));
static int32_t head = 0;
static int32_t tail = 0;
int logCopy1(void *to, void *from);
int logCopy2(void *to, void *from);
int logCopy4(void *to, void *from);
int logCopy8(void *to, void *from);
void logSetup(struct gyro_report* gyro_report,
		struct mag_report* mag_report,
		struct battery_status_s* battery_status,
		struct accel_report* accel_report,
		struct baro_report* barometer,
		struct sensor_combined_s* raw,
		struct ukf_state_vector_s* ukfState);
static float dummyFloat = 0.0f;
static double dummyDouble = 0.0f;
static uint32_t dummyUint32 = 0;
static int16_t dummyint16 = 0;
static uint16_t dummyUint16 = 0;
//static int8_t dummyint8 = 0;
static int8_t dummyint8_Value1 = 1;

logFields_t logFields[] = {
		{LOG_LASTUPDATE, LOG_TYPE_U32},
	    {LOG_VOLTAGE0, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE1, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE2, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE3, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE4, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE5, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE6, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE7, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE8, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE9, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE10, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE11, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE12, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE13, LOG_TYPE_FLOAT},
	    {LOG_VOLTAGE14, LOG_TYPE_FLOAT},
	    {LOG_IMU_RATEX, LOG_TYPE_FLOAT},
	    {LOG_IMU_RATEY, LOG_TYPE_FLOAT},
	    {LOG_IMU_RATEZ, LOG_TYPE_FLOAT},
	    {LOG_IMU_ACCX, LOG_TYPE_FLOAT},
	    {LOG_IMU_ACCY, LOG_TYPE_FLOAT},
	    {LOG_IMU_ACCZ, LOG_TYPE_FLOAT},
	    {LOG_IMU_MAGX, LOG_TYPE_FLOAT},
	    {LOG_IMU_MAGY, LOG_TYPE_FLOAT},
	    {LOG_IMU_MAGZ, LOG_TYPE_FLOAT},
	    {LOG_GPS_PDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_HDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_VDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_TDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_NDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_EDOP, LOG_TYPE_FLOAT},
	    {LOG_GPS_ITOW, LOG_TYPE_U32},
	    {LOG_GPS_POS_UPDATE, LOG_TYPE_U32},
	    {LOG_GPS_LAT, LOG_TYPE_DOUBLE},
	    {LOG_GPS_LON, LOG_TYPE_DOUBLE},
	    {LOG_GPS_HEIGHT, LOG_TYPE_FLOAT},
	    {LOG_GPS_HACC, LOG_TYPE_FLOAT},
	    {LOG_GPS_VACC, LOG_TYPE_FLOAT},
	    {LOG_GPS_VEL_UPDATE, LOG_TYPE_U32},
	    {LOG_GPS_VELN, LOG_TYPE_FLOAT},
	    {LOG_GPS_VELE, LOG_TYPE_FLOAT},
	    {LOG_GPS_VELD, LOG_TYPE_FLOAT},
	    {LOG_GPS_SACC, LOG_TYPE_FLOAT},
	    {LOG_ADC_PRESSURE1, LOG_TYPE_FLOAT},
	    {LOG_ADC_PRESSURE2, LOG_TYPE_FLOAT},
	    {LOG_ADC_TEMP0, LOG_TYPE_FLOAT},
	    {LOG_ADC_VIN, LOG_TYPE_FLOAT},
	    {LOG_ADC_MAG_SIGN, LOG_TYPE_S8},
	    {LOG_UKF_Q1, LOG_TYPE_FLOAT},
	    {LOG_UKF_Q2, LOG_TYPE_FLOAT},
	    {LOG_UKF_Q3, LOG_TYPE_FLOAT},
	    {LOG_UKF_Q4, LOG_TYPE_FLOAT},
	    {LOG_UKF_POSN, LOG_TYPE_FLOAT},
	    {LOG_UKF_POSE, LOG_TYPE_FLOAT},
	    {LOG_UKF_POSD, LOG_TYPE_FLOAT},
	    {LOG_UKF_PRES_ALT, LOG_TYPE_FLOAT},
	    {LOG_UKF_ALT, LOG_TYPE_FLOAT},/*
	    {LOG_UKF_VELN, LOG_TYPE_FLOAT},
	    {LOG_UKF_VELE, LOG_TYPE_FLOAT},
	    {LOG_UKF_VELD, LOG_TYPE_FLOAT},
	    {LOG_MOT_MOTOR0, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR1, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR2, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR3, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR4, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR5, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR6, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR7, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR8, LOG_TYPE_S16},
//	#if (PWM_HIGH_PORT > 9)
	    {LOG_MOT_MOTOR9, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR10, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR11, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR12, LOG_TYPE_S16},
	    {LOG_MOT_MOTOR13, LOG_TYPE_S16},
//	#endif
	    {LOG_MOT_THROTTLE, LOG_TYPE_FLOAT},
	    {LOG_MOT_PITCH, LOG_TYPE_FLOAT},
	    {LOG_MOT_ROLL, LOG_TYPE_FLOAT},
	    {LOG_MOT_YAW, LOG_TYPE_FLOAT},
	    {LOG_RADIO_QUALITY, LOG_TYPE_FLOAT},
	    {LOG_RADIO_CHANNEL0, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL1, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL2, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL3, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL4, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL5, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL6, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL7, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL8, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL9, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL10, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL11, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL12, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL13, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL14, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL15, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL16, LOG_TYPE_S16},
	    {LOG_RADIO_CHANNEL17, LOG_TYPE_S16},
	    {LOG_RADIO_ERRORS, LOG_TYPE_U16}*/
    };

int logCopy8(void *to, void *from) {
    *(uint32_t *)(to + 0) = *(uint32_t *)(from + 0);
    *(uint32_t *)(to + 4) = *(uint32_t *)(from + 4);

    return 8;
}

int logCopy4(void *to, void *from) {
    *(uint32_t *)to = *(uint32_t *)from;

    return 4;
}

int logCopy2(void *to, void *from) {
    *(uint16_t *)to = *(uint16_t *)from;

    return 2;
}

int logCopy1(void *to, void *from) {
    *(uint8_t *)to = *(uint8_t *)from;

    return 1;
}

void logDoHeader(void) {
    char *buf;
    char ckA, ckB;
    int i;

    buf = logData.logBuf + head;

    // log header signature
    *buf++ = 'A';
    *buf++ = 'q';
    *buf++ = 'H';

    // number of fields
    *buf++ = logData.numFields;

    // fields and types
    memcpy(buf, logFields, sizeof(logFields));

    // calc checksum
    ckA = ckB = 0;
    buf = logData.logBuf + head + 3;
    for (i = 0; i < (int)(1 + sizeof(logFields)); i++) {
	ckA += buf[i];
	ckB += ckA;
    }
    buf[i++] = ckA;
    buf[i++] = ckB;

    // block size is the actual data packet size
    head = (head + logData.packetSize) % (LOGGER_BUF_SIZE * logData.packetSize);
}

void logDo(void) {
    char *buf;
    char ckA, ckB;
    int i;

    buf = logData.logBuf + head;

    // log header signature
    *buf++ = 'A';
    *buf++ = 'q';
    *buf++ = 'M';

    // number of fields
    for (i = 0; i < logData.numFields; i++) {
    	buf += logData.fp[i].copyFunc(buf, logData.fp[i].fieldPointer);
    }

    ckA = ckB = 0;
    buf = logData.logBuf + head;
    for (i = 3; i < (int)logData.packetSize - 2; i++) {
		ckA += buf[i];
		ckB += ckA;
    }
    buf[i++] = ckA;
    buf[i++] = ckB;

    head = (head + logData.packetSize) % (LOGGER_BUF_SIZE * logData.packetSize);
}

size_t logWrite(FILE* loggingFile) {
    uint32_t size;
    uint32_t block = 512u;
    size_t res;
    uint32_t bufferLength = LOGGER_BUF_SIZE * logData.packetSize;
	if (head > tail) {
		size = head - tail;
		if(size < block) return 0;
	}
	else {
		size = bufferLength - tail;
	}
	// try to write 512 byte or more blocks
	if (size > block) {
		size = size / block * block;
	}

	res = fwrite(logData.logBuf + tail,1,size,loggingFile);
	tail = (tail + res) % bufferLength;
	return res;
}

void logSetup(struct gyro_report* gyro_report,
		struct mag_report* mag_report,
		struct battery_status_s* battery_status,
		struct accel_report* accel_report,
		struct baro_report* barometer,
		struct sensor_combined_s* raw,
		struct ukf_state_vector_s* ukfState) {
    int i;

    logData.numFields = sizeof(logFields) / sizeof(logFields_t);
    logData.packetSize = 3 + 2;  // signature + checksum
    logData.fp = (fieldData_t *)aqDataCalloc(logData.numFields, sizeof(fieldData_t));

    for (i = 0; i < logData.numFields; i++) {
	switch (logFields[i].fieldId) {
	    case LOG_LASTUPDATE:
		logData.fp[i].fieldPointer = (void *)&gyro_report->timestamp;
		break;
	    case LOG_VOLTAGE0:
		logData.fp[i].fieldPointer = (void *)&gyro_report->x;
		break;
	    case LOG_VOLTAGE1:
		logData.fp[i].fieldPointer = (void *)&gyro_report->y;
		break;
	    case LOG_VOLTAGE2:
		logData.fp[i].fieldPointer = (void *)&gyro_report->z;
		break;
	    case LOG_VOLTAGE3:
		logData.fp[i].fieldPointer = (void *)&mag_report->x;
		break;
	    case LOG_VOLTAGE4:
		logData.fp[i].fieldPointer = (void *)&mag_report->y;
		break;
	    case LOG_VOLTAGE5:
		logData.fp[i].fieldPointer = (void *)&mag_report->z;
		break;
	    case LOG_VOLTAGE6:
		logData.fp[i].fieldPointer = (void *)&gyro_report->temperature;
		break;
	    case LOG_VOLTAGE7:
		logData.fp[i].fieldPointer = (void *)&battery_status->voltage_v;
		break;
	    case LOG_VOLTAGE8:
		logData.fp[i].fieldPointer = (void *)&accel_report->x;
		break;
	    case LOG_VOLTAGE9:
		logData.fp[i].fieldPointer = (void *)&accel_report->y;
		break;
	    case LOG_VOLTAGE10:
		logData.fp[i].fieldPointer = (void *)&accel_report->z;
		break;
	    case LOG_VOLTAGE11:
		logData.fp[i].fieldPointer = (void *)&barometer->pressure;
		break;
	    case LOG_VOLTAGE12:
		logData.fp[i].fieldPointer = (void *)&barometer->pressure;
		break;
	    case LOG_VOLTAGE13:
		logData.fp[i].fieldPointer = (void *)&barometer->temperature;
		break;
	    case LOG_VOLTAGE14:
		logData.fp[i].fieldPointer = (void *)&accel_report->temperature;
		break;
	    case LOG_IMU_RATEX:
		logData.fp[i].fieldPointer = (void *)&raw->gyro_rad_s[0];//(void *)&IMU_RATEX;
		break;
	    case LOG_IMU_RATEY:
		logData.fp[i].fieldPointer = (void *)&raw->gyro_rad_s[1];//(void *)&IMU_RATEY;
		break;
	    case LOG_IMU_RATEZ:
		logData.fp[i].fieldPointer = (void *)&raw->gyro_rad_s[2];//(void *)&IMU_RATEZ;
		break;
	    case LOG_IMU_ACCX:
		logData.fp[i].fieldPointer = (void *)&raw->accelerometer_m_s2[0];//(void *)&IMU_ACCX;
		break;
	    case LOG_IMU_ACCY:
		logData.fp[i].fieldPointer = (void *)&raw->accelerometer_m_s2[1];//(void *)&IMU_ACCY;
		break;
	    case LOG_IMU_ACCZ:
		logData.fp[i].fieldPointer = (void *)&raw->accelerometer_m_s2[2];//(void *)&IMU_ACCZ;
		break;
	    case LOG_IMU_MAGX:
		logData.fp[i].fieldPointer = (void *)&raw->magnetometer_ga[0];//(void *)&IMU_MAGX;
		break;
	    case LOG_IMU_MAGY:
		logData.fp[i].fieldPointer = (void *)&raw->magnetometer_ga[1];//(void *)&IMU_MAGY;
		break;
	    case LOG_IMU_MAGZ:
		logData.fp[i].fieldPointer = (void *)&raw->magnetometer_ga[2];//(void *)&IMU_MAGZ;
		break;
	    case LOG_GPS_PDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.pDOP;
		break;
	    case LOG_GPS_HDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.hDOP;
		break;
	    case LOG_GPS_VDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.vDOP;
		break;
	    case LOG_GPS_TDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.tDOP;
		break;
	    case LOG_GPS_NDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.nDOP;
		break;
	    case LOG_GPS_EDOP:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.eDOP;
		break;
	    case LOG_GPS_ITOW:
		logData.fp[i].fieldPointer = (void *)&dummyUint32;//(void *)&gpsData.iTOW;
		break;
	    case LOG_GPS_POS_UPDATE:
		logData.fp[i].fieldPointer = (void *)&dummyUint32;//(void *)&gpsData.lastPosUpdate;
		break;
	    case LOG_GPS_LAT:
		logData.fp[i].fieldPointer = (void *)&dummyDouble;//(void *)&gpsData.lat;
		break;
	    case LOG_GPS_LON:
		logData.fp[i].fieldPointer = (void *)&dummyDouble;//(void *)&gpsData.lon;
		break;
	    case LOG_GPS_HEIGHT:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.height;
		break;
	    case LOG_GPS_HACC:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.hAcc;
		break;
	    case LOG_GPS_VACC:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.vAcc;
		break;
	    case LOG_GPS_VEL_UPDATE:
		logData.fp[i].fieldPointer = (void *)&dummyUint32;//(void *)&gpsData.lastVelUpdate;
		break;
	    case LOG_GPS_VELN:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.velN;
		break;
	    case LOG_GPS_VELE:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.velE;
		break;
	    case LOG_GPS_VELD:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.velD;
		break;
	    case LOG_GPS_SACC:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&gpsData.sAcc;
		break;
	    case LOG_ADC_PRESSURE1:
		logData.fp[i].fieldPointer = (void *)&barometer->pressure;
		break;
	    case LOG_ADC_PRESSURE2:
		logData.fp[i].fieldPointer = (void *)&barometer->pressure;//(void *)&adcData.pressure2;
		break;
	    case LOG_ADC_TEMP0:
		logData.fp[i].fieldPointer = (void *)&gyro_report->temperature;//(void *)&IMU_TEMP; //original board temp used for all calculations
		break;
	    case LOG_ADC_VIN:
		logData.fp[i].fieldPointer = (void *)&battery_status->voltage_v;//(void *)&adcData.vIn;
		break;
	    case LOG_ADC_MAG_SIGN:
		logData.fp[i].fieldPointer = (void *)&dummyint8_Value1;//(void *)&adcData.magSign;
		break;
	    case LOG_UKF_Q1:
		logData.fp[i].fieldPointer = (void *)&ukfState->q1;
		break;
	    case LOG_UKF_Q2:
		logData.fp[i].fieldPointer = (void *)&ukfState->q2;//(void *)&UKF_Q2;
		break;
	    case LOG_UKF_Q3:
		logData.fp[i].fieldPointer = (void *)&ukfState->q3;//(void *)&UKF_Q3;
		break;
	    case LOG_UKF_Q4:
		logData.fp[i].fieldPointer = (void *)&ukfState->q4;//(void *)&UKF_Q4;
		break;
	    case LOG_UKF_POSN:
		logData.fp[i].fieldPointer = (void *)&ukfState->pos_x;//(void *)&UKF_POSN;
		break;
	    case LOG_UKF_POSE:
		logData.fp[i].fieldPointer = (void *)&ukfState->pos_y;//(void *)&UKF_POSE;
		break;
	    case LOG_UKF_POSD:
		logData.fp[i].fieldPointer = (void *)&ukfState->pos_d;//(void *)&UKF_POSD;
		break;
	    case LOG_UKF_PRES_ALT:
		logData.fp[i].fieldPointer = (void *)&raw->baro_pres_mbar;//(void *)&UKF_PRES_ALT;
		break;
	    case LOG_UKF_ALT:
		logData.fp[i].fieldPointer = (void *)&raw->baro_alt_meter;//(void *)&UKF_ALTITUDE;
		break;
	    case LOG_UKF_VELN:
		logData.fp[i].fieldPointer = (void *)&ukfState->vel_x;//(void *)&UKF_VELN;
		break;
	    case LOG_UKF_VELE:
		logData.fp[i].fieldPointer = (void *)&ukfState->vel_y;//(void *)&UKF_VELE;
		break;
	    case LOG_UKF_VELD:
		logData.fp[i].fieldPointer = (void *)&ukfState->vel_d;//(void *)&UKF_VELD;
		break;
	    case LOG_MOT_MOTOR0:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[0];
		break;
	    case LOG_MOT_MOTOR1:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[1];
		break;
	    case LOG_MOT_MOTOR2:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[2];
		break;
	    case LOG_MOT_MOTOR3:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[3];
		break;
	    case LOG_MOT_MOTOR4:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[4];
		break;
	    case LOG_MOT_MOTOR5:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[5];
		break;
	    case LOG_MOT_MOTOR6:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[6];
		break;
	    case LOG_MOT_MOTOR7:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[7];
		break;
	    case LOG_MOT_MOTOR8:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[8];
		break;
	    case LOG_MOT_MOTOR9:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[9];
		break;
	    case LOG_MOT_MOTOR10:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[10];
		break;
	    case LOG_MOT_MOTOR11:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[11];
		break;
	    case LOG_MOT_MOTOR12:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[12];
		break;
	    case LOG_MOT_MOTOR13:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&motorsData.value[13];
		break;
	    case LOG_MOT_THROTTLE:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&motorsData.throttle;
		break;
	    case LOG_MOT_PITCH:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&motorsData.pitch;
		break;
	    case LOG_MOT_ROLL:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&motorsData.roll;
		break;
	    case LOG_MOT_YAW:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&motorsData.yaw;
		break;
	    case LOG_RADIO_QUALITY:
		logData.fp[i].fieldPointer = (void *)&dummyFloat;//(void *)&RADIO_QUALITY;
		break;
	    case LOG_RADIO_CHANNEL0:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[0];
		break;
	    case LOG_RADIO_CHANNEL1:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[1];
		break;
	    case LOG_RADIO_CHANNEL2:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[2];
		break;
	    case LOG_RADIO_CHANNEL3:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[3];
		break;
	    case LOG_RADIO_CHANNEL4:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[4];
		break;
	    case LOG_RADIO_CHANNEL5:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[5];
		break;
	    case LOG_RADIO_CHANNEL6:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[6];
		break;
	    case LOG_RADIO_CHANNEL7:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[7];
		break;
	    case LOG_RADIO_CHANNEL8:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[8];
		break;
	    case LOG_RADIO_CHANNEL9:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[9];
		break;
	    case LOG_RADIO_CHANNEL10:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[10];
		break;
	    case LOG_RADIO_CHANNEL11:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[11];
		break;
	    case LOG_RADIO_CHANNEL12:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[12];
		break;
	    case LOG_RADIO_CHANNEL13:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[13];
		break;
	    case LOG_RADIO_CHANNEL14:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[14];
		break;
	    case LOG_RADIO_CHANNEL15:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[15];
		break;
	    case LOG_RADIO_CHANNEL16:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[16];
		break;
	    case LOG_RADIO_CHANNEL17:
		logData.fp[i].fieldPointer = (void *)&dummyint16;//(void *)&radioData.channels[17];
		break;
        case LOG_RADIO_ERRORS:
        logData.fp[i].fieldPointer = (void *)&dummyUint16;//RADIO_ERROR_COUNT;
        break;
	}

	switch (logFields[i].fieldType) {
	    case LOG_TYPE_DOUBLE:
		logData.fp[i].copyFunc = logCopy8;
		logData.packetSize += 8;
		break;
	    case LOG_TYPE_FLOAT:
	    case LOG_TYPE_U32:
	    case LOG_TYPE_S32:
		logData.fp[i].copyFunc = logCopy4;
		logData.packetSize += 4;
		break;
	    case LOG_TYPE_U16:
	    case LOG_TYPE_S16:
		logData.fp[i].copyFunc = logCopy2;
		logData.packetSize += 2;
		break;
	    case LOG_TYPE_U8:
	    case LOG_TYPE_S8:
		logData.fp[i].copyFunc = logCopy1;
		logData.packetSize += 1;
		break;
	}
    }

    logData.logBuf = (char *)aqCalloc(LOGGER_BUF_SIZE, logData.packetSize);
}

void logInit(struct gyro_report* gyro_report,
		struct mag_report* mag_report,
		struct battery_status_s* battery_status,
		struct accel_report* accel_report,
		struct baro_report* barometer,
		struct sensor_combined_s* raw,
		struct ukf_state_vector_s* ukfState) {
    memset((void *)&logData, 0, sizeof(logData));

    logSetup(gyro_report,mag_report,battery_status,accel_report,barometer,raw, ukfState);

    logDoHeader();
}
