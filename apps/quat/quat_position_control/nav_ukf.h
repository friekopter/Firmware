/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012  Bill Nesbitt
*/

#ifndef _nav_ukf_h
#define _nav_ukf_h

#include "srcdkf.h"
#include "quat_pos_control_params.h"
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>

#define SIM_S                   17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		20

#define UKF_VELN		navUkfData.x[0]
#define UKF_VELE		navUkfData.x[1]
#define UKF_VELD		navUkfData.x[2]
#define UKF_POSN		navUkfData.x[3]
#define UKF_POSE		navUkfData.x[4]
#define UKF_POSD		navUkfData.x[5]
#define UKF_ACC_BIAS_X		navUkfData.x[6]
#define UKF_ACC_BIAS_Y		navUkfData.x[7]
#define UKF_ACC_BIAS_Z		navUkfData.x[8]
#define UKF_GYO_BIAS_X		navUkfData.x[9]
#define UKF_GYO_BIAS_Y		navUkfData.x[10]
#define UKF_GYO_BIAS_Z		navUkfData.x[11]
#define UKF_Q1			navUkfData.x[12]
#define UKF_Q2			navUkfData.x[13]
#define UKF_Q3			navUkfData.x[14]
#define UKF_Q4			navUkfData.x[15]
#define UKF_PRES_ALT		navUkfData.x[16]

#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif

#define UKF_HIST		40

#define UKF_P0			101325.0f		    // standard static pressure at sea level

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    float r1, r2;
    float posN[UKF_HIST];
    float posE[UKF_HIST];
    float posD[UKF_HIST];
    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    float yawCos, yawSin;
    float mat3x3[3*3];
    float *x;			// states
    float presAltOffset;
    int logPointer;
    uint8_t logHandle;
} navUkfStruct_t;

extern navUkfStruct_t navUkfData;

bool isFlying(const struct vehicle_status_s *current_status);
extern void navUkfInit(	const struct quat_position_control_UKF_params* params,
						const struct sensor_combined_s* sensors);
extern float navUkfInertialUpdate(const struct sensor_combined_s* raw);
extern void simDoPresUpdate(float pres,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params);
extern void simDoAccUpdate(float accX, float accY, float accZ,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params);
extern void simDoMagUpdate(float magX, float magY, float magZ,
		 const struct vehicle_status_s *current_status,
		 const struct quat_position_control_UKF_params* params);
extern void navUkfGpsPosUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params);
extern void navUkfGpsVelUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_UKF_params* params);
extern void navUkfSetGlobalPositionTarget(double lat, double lon);
extern void UKFPressureAdjust(float altitude);
extern void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
extern void navUkfZeroRate(float zRate, int axis);
extern void navUkfFinish(void);

#endif
