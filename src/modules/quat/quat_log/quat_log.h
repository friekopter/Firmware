/*
 * quat_log.h
 *
 *  Created on: Jun 2, 2013
 *      Author: fludwig
 */

#ifndef QUAT_LOG_H_
#define QUAT_LOG_H_

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/ukf_state_vector.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>

#define LOGGER_BUF_SIZE			100u

enum {
    LOG_LASTUPDATE = 0,
    LOG_VOLTAGE0,
    LOG_VOLTAGE1,
    LOG_VOLTAGE2,
    LOG_VOLTAGE3,
    LOG_VOLTAGE4,
    LOG_VOLTAGE5,
    LOG_VOLTAGE6,
    LOG_VOLTAGE7,
    LOG_VOLTAGE8,
    LOG_VOLTAGE9,
    LOG_VOLTAGE10,
    LOG_VOLTAGE11,
    LOG_VOLTAGE12,
    LOG_VOLTAGE13,
    LOG_VOLTAGE14,
    LOG_IMU_RATEX,
    LOG_IMU_RATEY,
    LOG_IMU_RATEZ,
    LOG_IMU_ACCX,
    LOG_IMU_ACCY,
    LOG_IMU_ACCZ,
    LOG_IMU_MAGX,
    LOG_IMU_MAGY,
    LOG_IMU_MAGZ,
    LOG_GPS_PDOP,
    LOG_GPS_HDOP,
    LOG_GPS_VDOP,
    LOG_GPS_TDOP,
    LOG_GPS_NDOP,
    LOG_GPS_EDOP,
    LOG_GPS_ITOW,
    LOG_GPS_POS_UPDATE,
    LOG_GPS_LAT,
    LOG_GPS_LON,
    LOG_GPS_HEIGHT,
    LOG_GPS_HACC,
    LOG_GPS_VACC,
    LOG_GPS_VEL_UPDATE,
    LOG_GPS_VELN,
    LOG_GPS_VELE,
    LOG_GPS_VELD,
    LOG_GPS_SACC,
    LOG_ADC_PRESSURE1,
    LOG_ADC_PRESSURE2,
    LOG_ADC_TEMP0,
    LOG_ADC_TEMP1,
    LOG_ADC_TEMP2,
    LOG_ADC_VIN,
    LOG_ADC_MAG_SIGN,
    LOG_UKF_Q1,
    LOG_UKF_Q2,
    LOG_UKF_Q3,
    LOG_UKF_Q4,
    LOG_UKF_POSN,
    LOG_UKF_POSE,
    LOG_UKF_POSD,
    LOG_UKF_PRES_ALT,
    LOG_UKF_ALT,
    LOG_UKF_VELN,
    LOG_UKF_VELE,
    LOG_UKF_VELD,
    LOG_MOT_MOTOR0,
    LOG_MOT_MOTOR1,
    LOG_MOT_MOTOR2,
    LOG_MOT_MOTOR3,
    LOG_MOT_MOTOR4,
    LOG_MOT_MOTOR5,
    LOG_MOT_MOTOR6,
    LOG_MOT_MOTOR7,
    LOG_MOT_MOTOR8,
    LOG_MOT_MOTOR9,
    LOG_MOT_MOTOR10,
    LOG_MOT_MOTOR11,
    LOG_MOT_MOTOR12,
    LOG_MOT_MOTOR13,
    LOG_MOT_THROTTLE,
    LOG_MOT_PITCH,
    LOG_MOT_ROLL,
    LOG_MOT_YAW,
    LOG_RADIO_QUALITY,
    LOG_RADIO_CHANNEL0,
    LOG_RADIO_CHANNEL1,
    LOG_RADIO_CHANNEL2,
    LOG_RADIO_CHANNEL3,
    LOG_RADIO_CHANNEL4,
    LOG_RADIO_CHANNEL5,
    LOG_RADIO_CHANNEL6,
    LOG_RADIO_CHANNEL7,
    LOG_RADIO_CHANNEL8,
    LOG_RADIO_CHANNEL9,
    LOG_RADIO_CHANNEL10,
    LOG_RADIO_CHANNEL11,
    LOG_RADIO_CHANNEL12,
    LOG_RADIO_CHANNEL13,
    LOG_RADIO_CHANNEL14,
    LOG_RADIO_CHANNEL15,
    LOG_RADIO_CHANNEL16,
    LOG_RADIO_CHANNEL17,
    LOG_RADIO_ERRORS,
    LOG_NUM_IDS
};

enum {
    LOG_TYPE_DOUBLE = 0,
    LOG_TYPE_FLOAT,
    LOG_TYPE_U32,
    LOG_TYPE_S32,
    LOG_TYPE_U16,
    LOG_TYPE_S16,
    LOG_TYPE_U8,
    LOG_TYPE_S8
};

typedef struct {
    uint8_t fieldId;
    uint8_t fieldType;
} logFields_t;

typedef struct {
    void *fieldPointer;
    int (*copyFunc)(void *, void *);
} fieldData_t;

typedef struct {
    char *logBuf;
    fieldData_t *fp;
    uint32_t packetSize;
    uint8_t numFields;
} logStruct_t;

extern logStruct_t logData;
extern void logInit(struct gyro_report* gyro_report,
		struct mag_report* mag_report,
		struct battery_status_s* battery_status,
		struct accel_report* accel_report,
		struct baro_report* barometer,
		struct sensor_combined_s* raw,
		struct ukf_state_vector_s* ukfState);
extern void logDo(void);
extern void logDoHeader(void);
size_t logWrite(FILE* loggingFile);



#endif /* QUAT_LOGGER_H_ */
