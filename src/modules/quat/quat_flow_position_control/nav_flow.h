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

    Copyright � 2011, 2012  Bill Nesbitt
*/

#ifndef _nav_flow_h
#define _nav_flow_h

//#include <CoOS.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <quat/utils/quat_pos_control_params.h>
#include <quat/utils/pid.h>

#define NAV_MAX_FIX_AGE		((int)1e6f)				    // 1 second

#define NAV_LANDING_VEL		0.33f					    // default landing/takeoff vertical velocity
#define NAV_LANDING_DECEL	(-1.5f * GRAVITY)			    // deceleration needed to indicate a landing (1.5g)

#define NAV_EQUATORIAL_RADIUS	(6378.137f * 1000.0f)			    // meters
#define NAV_FLATTENING		(1.0f / 298.257223563f)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0f - NAV_FLATTENING))

#define NAV_STATUS_MANUAL	0x00					    // full manual control
#define NAV_STATUS_ALTHOLD	0x01					    // altitude hold only
#define NAV_STATUS_POSHOLD	0x02					    // altitude & position hold
#define NAV_STATUS_DVH		0x03					    // dynamic velocity hold cut through
#define NAV_STATUS_MISSION	0x04					    // autonomous mission

#define IMU_STATIC_STD		0.05f						// Standard deviation

typedef struct {
    float poiAngle;			// pitch angle for gimbal to center POI
    float holdAlt;			// altitude to hold
    float targetHeading;		// navigation heading target (>= 0 is absolute, < 0 is relative to target bearing)
    float holdHeading;			// heading to hold
    float holdCourse;			// course to hold position
    float holdDistance;			// distance to hold position (straight line)
    float holdMaxHorizSpeed;		// maximum N/E speed allowed to achieve position
    float holdMaxVertSpeed;		// maximum UP/DOWN speed allowed to achieve altitude
    float holdPositionX;		// hold position X
    float holdPositionY;		// hold position Y
    float holdSpeedX;			// required speed (North/South)
    float holdSpeedY;			// required speed (East/West)
    float holdTiltX;			// required tilt (North/South)
    float holdTiltY;			// required tilt (East/West)
    float holdSpeedAlt;			// required speed (Up/Down)
    float targetHoldSpeedAlt;

    pidStruct_t *distanceXPID;
    pidStruct_t *distanceYPID;
    pidStruct_t *speedXPID;		// PID to control N/S speed - output tilt in degrees
    pidStruct_t *speedYPID;		// PID to control E/W speed - output tilt in degrees
    pidStruct_t *altSpeedPID;		// PID to control U/D speed - output speed in m/s
    pidStruct_t *altPosPID;		// PID to control U/D distance - output error in meters

    uint32_t lastUpdate;

    uint8_t mode;			// navigation mode
    uint8_t navCapable;
} navFlowStruct_t;

extern navFlowStruct_t navFlowData;

extern void navFlowInit(const struct quat_position_control_NAV_params* params,
		float holdYaw,
		float holdAlt);
extern void navFlowNavigate(
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_NAV_params* params,
		const struct manual_control_setpoint_s* manual_control,
		const struct vehicle_local_position_s* local_position_data,
		struct vehicle_attitude_s* att,
		uint64_t imu_timestamp
		);
extern void navFlowResetHoldAlt(float delta);
extern void navFlowSetHoldHeading(float targetHeading);
extern void navPublishSystemInfo(void);

#endif
