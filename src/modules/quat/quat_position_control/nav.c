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

#include "nav.h"
#include "nav_ukf.h"
#include <quat/utils/util.h>
#include <quat/utils/quat_constants.h>
#include <quat/utils/pid.h>
#include "nav_ukf.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include <quat/utils/aq_math.h>
#include <stdio.h>
#include <uORB/topics/manual_control_setpoint.h>

#define CTRL_DEAD_BAND (5.0f/1000.0f)

void navSetHoldAlt(float alt, uint8_t relative);
navStruct_t navData __attribute__((section(".ccm")));

void navResetHoldAlt(float delta) {
    navData.holdAlt += delta;
}

void navSetHoldAlt(float alt, uint8_t relative) {
    if (relative)
	navData.holdAlt += alt;
    else
	navData.holdAlt = alt;
}

void navSetHoldHeading(float targetHeading) {
    // signbit() returns true if negative sign
    if (signbit(targetHeading))
	// use targetHeading as relative to bearing to target
	navData.holdHeading = compassNormalizeRad(navData.holdCourse + targetHeading);
    else
	// use targetHeading as absolute heading
	navData.holdHeading = targetHeading;
}

void navSetHomeCurrent(	const struct vehicle_gps_position_s* gps_position,
						const struct quat_position_control_NAV_params* params) {
    navData.homeLeg.type = NAV_LEG_GOTO;
    navData.homeLeg.relativeAlt = 0;
    navData.homeLeg.targetAlt = UKF_ALTITUDE;
    navData.homeLeg.targetLat = gps_position->lat;
    navData.homeLeg.targetLon = gps_position->lon;
    navData.homeLeg.maxHorizSpeed = params->nav_max_speed;
    // navData.homeLeg.poiHeading = AQ_YAW; //TODO FL set yaw
}


void navRecallHome(const struct quat_position_control_NAV_params* params) {
    navUkfSetGlobalPositionTarget(navData.homeLeg.targetLat, navData.homeLeg.targetLon);
    navSetHoldAlt(navData.homeLeg.targetAlt, navData.homeLeg.relativeAlt);
    navData.holdMaxHorizSpeed = navData.homeLeg.maxHorizSpeed;
    navData.holdMaxVertSpeed = navData.homeLeg.maxVertSpeed;
    navSetHoldHeading(navData.homeLeg.poiHeading);

    if (fabsf(navData.holdMaxHorizSpeed) < FLT_MIN)
	navData.holdMaxHorizSpeed = params->nav_max_speed;
    if (fabsf(navData.holdMaxVertSpeed) < FLT_MIN)
	navData.holdMaxVertSpeed = params->nav_alt_pos_om;
}
/*

void navLoadLeg(uint8_t leg) {
    // invalid type?
    if (!navData.missionLegs[leg].type ||
    		navData.missionLegs[leg].type >= NAV_NUM_LEG_TYPES)
	return;

    // common
    if (navData.missionLegs[leg].relativeAlt)
	navSetHoldAlt(navData.missionLegs[leg].targetAlt, navData.missionLegs[leg].relativeAlt);
    else
	navSetHoldAlt(navData.missionLegs[leg].targetAlt - navUkfData.presAltOffset, navData.missionLegs[leg].relativeAlt);

    navData.holdMaxHorizSpeed = navData.missionLegs[leg].maxHorizSpeed;
    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;

    // type specific
    if (navData.missionLegs[leg].type == NAV_LEG_HOME) {
	navSetHoldAlt(navData.homeLeg.targetAlt, navData.homeLeg.relativeAlt);
	navUkfSetGlobalPositionTarget(navData.homeLeg.targetLat, navData.homeLeg.targetLon);
	navData.targetHeading = navData.homeLeg.poiHeading;

	navData.holdMaxHorizSpeed = navData.homeLeg.maxHorizSpeed;
	navData.holdMaxVertSpeed = navData.homeLeg.maxVertSpeed;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_GOTO) {
	if (navData.missionLegs[leg].targetLat != 0.0 && navData.missionLegs[leg].targetLon != 0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_ORBIT) {
	if (navData.missionLegs[leg].targetLat != 0.0 && navData.missionLegs[leg].targetLon != 0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_TAKEOFF) {
	// store this position as the takeoff position
	navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);
	navData.targetHeading = AQ_YAW;

	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = NAV_LANDING_VEL;
	else
	    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;

	// set the launch location as home
	navSetHomeCurrent();
	navData.homeLeg.targetAlt = navData.holdAlt;
	navData.homeLeg.poiHeading = -0.0f;		// relative
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_LAND) {
	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = NAV_LANDING_VEL;
	else
	    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;
    }

    if (navData.holdMaxHorizSpeed == 0.0f)
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    if (navData.holdMaxVertSpeed == 0.0f)
	navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];

    navData.loiterCompleteTime = 0;

    navData.missionLeg = leg;

#ifdef USE_MAVLINK
    // notify ground
    mavlinkWpAnnounceCurrent(leg);
#endif
}
*/

void navNavigate(
		const struct vehicle_gps_position_s* gps_position,
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_NAV_params* params,
		const struct manual_control_setpoint_s* manual_control,
		const struct filtered_bottom_flow_s* flow_data,
		uint64_t imu_timestamp
		) {
    uint64_t currentTime = imu_timestamp;
    unsigned char leg = navData.missionLeg;
    float tmp;

    // do we have a sufficient, recent fix?
    if ((currentTime - gps_position->timestamp_position) < NAV_MAX_FIX_AGE
    		&& (gps_position->eph_m/* * runData.accMask*/) < NAV_MIN_GPS_ACC
    		) {
    	navData.navCapable = 1;
    }
    else if ((currentTime - flow_data->timestamp) < NAV_MAX_FIX_AGE) {
    	navData.navCapable = 1;
    }
    // do not drop out of mission due to (hopefully) temporary GPS degradation
    else if (navData.mode < NAV_STATUS_POSHOLD) {
	// Cannot Navigate
    	navData.navCapable = 0;
    }
/*
    // Can we navigate && do we want to be in mission mode?
    if (navData.navCapable && RADIO_FLAPS > 250) {
	//  are we currently in position hold mode && do we have a clear mission ahead of us?
    	if (navData.mode == NAV_STATUS_POSHOLD && leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
    		navLoadLeg(leg);
    		navData.mode = NAV_STATUS_MISSION;
    	}
    }else */
    // do we want to be in position hold mode?
    if (current_status->state_machine == SYSTEM_STATE_STABILIZED) {
		// always allow alt hold
		if (navData.mode < NAV_STATUS_ALTHOLD) {
			// record this altitude as the hold altitude
			navSetHoldAlt(UKF_ALTITUDE, 0);

			// set integral to current RC throttle setting
			pidZeroIntegral(navData.altSpeedPID, -UKF_VELD, manual_control->throttle);
			pidZeroIntegral(navData.altPosPID, UKF_ALTITUDE, 0.0f);

			navData.mode = NAV_STATUS_ALTHOLD;
			navData.holdSpeedAlt = -UKF_VELD;
			printf("[quat_pos_control]: Altitude hold activated\n");
		}

		// are we not in position hold mode now?
		if (navData.navCapable &&
			navData.mode != NAV_STATUS_POSHOLD &&
			navData.mode != NAV_STATUS_DVH) {
			// store this position as hold position
			navUkfSetGlobalPositionTarget(gps_position->lat, gps_position->lon);

			// set this position as home if we have none
			if (navData.homeLeg.targetLat == 0.0 ||
				navData.homeLeg.targetLon == 0.0) {
				navSetHomeCurrent(gps_position, params);
			}

			// only zero bias if coming from lower mode
			if (navData.mode < NAV_STATUS_POSHOLD) {
				navData.holdTiltN = 0.0f;
				navData.holdTiltE = 0.0f;

				// speed
				pidZeroIntegral(navData.speedNPID, UKF_VELN, 0.0f);
				pidZeroIntegral(navData.speedEPID, UKF_VELE, 0.0f);

				// pos
				pidZeroIntegral(navData.distanceNPID, UKF_POSN, 0.0f);
				pidZeroIntegral(navData.distanceEPID, UKF_POSE, 0.0f);
			}

			navData.holdMaxHorizSpeed = params->nav_max_speed;
			navData.holdMaxVertSpeed = params->nav_alt_pos_om;

			// activate pos hold
			navData.mode = NAV_STATUS_POSHOLD;
			printf("[quat_pos_control]: Position hold activated\n");
		}
		// DVH
		else if (navData.navCapable && (
			manual_control->pitch > CTRL_DEAD_BAND ||
			manual_control->pitch < -CTRL_DEAD_BAND ||
			manual_control->roll > CTRL_DEAD_BAND ||
			manual_control->roll < -CTRL_DEAD_BAND)) {
				navData.mode = NAV_STATUS_DVH;
				//printf("[quat_pos_control]: DVH activated\n");
		}
		else if (navData.navCapable && navData.mode == NAV_STATUS_DVH) {
			// allow speed to drop before holding position (or if RTH engaged)
			//if (
			//	   (UKF_VELN < +0.1f &&
			//		UKF_VELN > -0.1f &&
			//		UKF_VELE < +0.1f &&
			//		UKF_VELE > -0.1f)
			//		||
			//		RADIO_AUX2 < -250) {
				navUkfSetGlobalPositionTarget(gps_position->lat, gps_position->lon);
				navData.mode = NAV_STATUS_POSHOLD;
				printf("[quat_pos_control]: Position hold activated from DVH\n");
			//}
		}
    }
    else {
		// switch to manual mode
		navData.mode = NAV_STATUS_MANUAL;
		// reset mission legs
		navData.missionLeg = leg = 0;
		// keep up with changing altitude
		navSetHoldAlt(UKF_ALTITUDE, 0);
    }

    /*
    // home set
    if (RADIO_AUX2 > 250) {
    	navSetHomeCurrent();
    }
    // recall home
    else if (RADIO_AUX2 < -250) {
    	navRecallHome();
    }
    */

    if (UKF_POSN != 0.0f || UKF_POSE != 0.0f) {
		navData.holdCourse = compassNormalizeRad(atan2f(-UKF_POSE, -UKF_POSN));
		navData.holdDistance = aq_sqrtf(UKF_POSN*UKF_POSN + UKF_POSE*UKF_POSE);
    }
    else {
		navData.holdCourse = 0.0f;
		navData.holdDistance = 0.0f;
    }

    /*
    if (navData.mode == NAV_STATUS_MISSION) {
		// have we arrived yet?
		if (navData.loiterCompleteTime == 0) {
			// are we close enough (distance and altitude)?
			// goto/home test
			if (((navData.missionLegs[leg].type == NAV_LEG_GOTO ||
					navData.missionLegs[leg].type == NAV_LEG_HOME) &&
			navData.holdDistance < navData.missionLegs[leg].targetRadius &&
			fabsf(navData.holdAlt - UKF_ALTITUDE) < navData.missionLegs[leg].targetRadius) ||
			// orbit test
			(navData.missionLegs[leg].type == NAV_LEG_ORBIT &&
			fabsf(navData.holdDistance - navData.missionLegs[leg].targetRadius) +
			fabsf(navData.holdAlt - UKF_ALTITUDE) < 2.0f)  ||
			// takeoff test
			(navData.missionLegs[leg].type == NAV_LEG_TAKEOFF &&
			navData.holdDistance < navData.missionLegs[leg].targetRadius &&
			fabsf(navData.holdAlt - UKF_ALTITUDE) < navData.missionLegs[leg].targetRadius)
			) {
				// start the loiter clock
				navData.loiterCompleteTime = currentTime + navData.missionLegs[leg].loiterTime;
			}
		}
		// have we loitered long enough?
		else if (currentTime > navData.loiterCompleteTime && navData.missionLegs[leg].type != NAV_LEG_LAND) {
			// next leg
			if (++leg < NAV_MAX_MISSION_LEGS &&
					navData.missionLegs[leg].type > 0) {
			navLoadLeg(leg);
			}
			else {
			navData.mode = NAV_STATUS_POSHOLD;
			}
		}
    }
*/
    // DVH
    if (navData.mode == NAV_STATUS_DVH) {
		float factor = (1.0f / 500.0f) * navData.holdMaxHorizSpeed;
		float x = 0.0f;
		float y = 0.0f;

		if (manual_control->pitch > CTRL_DEAD_BAND)
			x = -(manual_control->pitch - CTRL_DEAD_BAND) * factor;
		if (manual_control->pitch < -CTRL_DEAD_BAND)
			x = -(manual_control->pitch + CTRL_DEAD_BAND) * factor;
		if (manual_control->roll > CTRL_DEAD_BAND)
			y = +(manual_control->roll - CTRL_DEAD_BAND) * factor;
		if (manual_control->roll < -CTRL_DEAD_BAND)
			y = +(manual_control->roll + CTRL_DEAD_BAND) * factor;

		// rotate to earth frame
		navData.holdSpeedN = x * navUkfData.yawCos - y * navUkfData.yawSin;
		navData.holdSpeedE = y * navUkfData.yawCos + x * navUkfData.yawSin;
    }
    /*
    // orbit POI
    else if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].type == NAV_LEG_ORBIT) {
		float velX, velY;

		// maintain orbital radius
		velX = -pidUpdate(navData.distanceNPID, navData.missionLegs[leg].targetRadius, navData.holdDistance);

		// maintain orbital velocity (clock wise)
		velY = -navData.missionLegs[leg].maxHorizSpeed;

		// rotate to earth frame
		navData.holdSpeedN = velX * navUkfData.yawCos - velY * navUkfData.yawSin;
		navData.holdSpeedE = velY * navUkfData.yawCos + velX * navUkfData.yawSin;
    }*/
    else {
		// distance => velocity
		navData.holdSpeedN = pidUpdate(navData.distanceNPID, 0.0f, UKF_POSN);
		navData.holdSpeedE = pidUpdate(navData.distanceEPID, 0.0f, UKF_POSE);
    }

    if (fabs(navData.holdSpeedE) > FLT_MIN || fabs(navData.holdSpeedE) > FLT_MIN) {
        // normalize N/E speed requests to fit below max nav speed
        tmp = aq_sqrtf(navData.holdSpeedN*navData.holdSpeedN + navData.holdSpeedE*navData.holdSpeedE);
        if (tmp > navData.holdMaxHorizSpeed) {
    		navData.holdSpeedN = (navData.holdSpeedN / tmp) * navData.holdMaxHorizSpeed;
    		navData.holdSpeedE = (navData.holdSpeedE / tmp) * navData.holdMaxHorizSpeed;
        }
    }

    // velocity => tilt
    navData.holdTiltN = -pidUpdate(navData.speedNPID, navData.holdSpeedN, UKF_VELN);
    navData.holdTiltE = +pidUpdate(navData.speedEPID, navData.holdSpeedE, UKF_VELE);

    if (navData.mode > NAV_STATUS_MANUAL) {
		float vertStick;

		// Throttle controls vertical speed
		// Throttle is 0 ... 1
		vertStick = (manual_control->throttle * 2.0f) - 1.0f;
		if (vertStick > CTRL_DEAD_BAND || vertStick < -CTRL_DEAD_BAND) {
			// altitude velocity proportional to throttle stick
			// TODO: Assume that throttle control is normalized to -1 ... 1
			if (vertStick > 0.0f)
			navData.targetHoldSpeedAlt = (vertStick - CTRL_DEAD_BAND) * params->nav_alt_pos_om;
			else
			navData.targetHoldSpeedAlt = (vertStick + CTRL_DEAD_BAND) * params->nav_max_decent;

			// set new hold altitude to wherever we are during vertical speed overrides
			if (navData.mode != NAV_STATUS_MISSION)
			navSetHoldAlt(UKF_ALTITUDE, 0);
		}
		/*// are we trying to land?
		else if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].type == NAV_LEG_LAND) {
			navData.targetHoldSpeedAlt = -navData.holdMaxVertSpeed;
		}*/
		else {
			navData.targetHoldSpeedAlt = pidUpdate(navData.altPosPID, navData.holdAlt, UKF_ALTITUDE);
		}

		// constrain vertical velocity
		navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt,
				(navData.holdMaxVertSpeed < params->nav_max_decent) ? -navData.holdMaxVertSpeed : -params->nav_max_decent, navData.holdMaxVertSpeed);

		// smooth vertical velocity changes
		navData.holdSpeedAlt += (navData.targetHoldSpeedAlt - navData.holdSpeedAlt) * 0.01f;
    }

    /*// calculate POI angle
    if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].poiAltitude != 0.0f) {
		float a, b, c;

		a = navData.holdDistance;
		b = UKF_ALTITUDE - navData.missionLegs[leg].poiAltitude;
		c = __sqrtf(a*a + b*b);

		navData.poiAngle = asinf(a/c) * RAD_TO_DEG - 90.0f;
    }
    else {
    	navData.poiAngle = 0.0f;
    }

    if (navData.mode == NAV_STATUS_MISSION) {
		// recalculate autonomous heading
		navSetHoldHeading(navData.targetHeading);

		// watch for bump if landing
		if (navData.missionLegs[leg].type == NAV_LEG_LAND && IMU_ACCZ < NAV_LANDING_DECEL)
			// shut everything down (sure hope we are really on the ground :)
			supervisorDisarm();
    }*/

    navData.lastUpdate = currentTime;
}

void navInit(const struct quat_position_control_NAV_params* params,
				float holdYaw,
				float holdAlt) {
    int i = 0;

    memset(&navData,0,sizeof(navData));
	navData.holdMaxHorizSpeed = params->nav_max_speed;
	navData.holdMaxVertSpeed = params->nav_alt_pos_om;

    navData.speedNPID = pidInit(&params->nav_speed_p, &params->nav_speed_i, 0, 0, &params->nav_speed_pm, &params->nav_speed_im, 0, &params->nav_speed_om, 0, 0, 0, 0);
    navData.speedEPID = pidInit(&params->nav_speed_p, &params->nav_speed_i, 0, 0, &params->nav_speed_pm, &params->nav_speed_im, 0, &params->nav_speed_om, 0, 0, 0, 0);
    navData.distanceNPID = pidInit(&params->nav_dist_p, &params->nav_dist_i, 0, 0, &params->nav_dist_pm, &params->nav_dist_im, 0, &params->nav_dist_om, 0, 0, 0, 0);
    navData.distanceEPID = pidInit(&params->nav_dist_p, &params->nav_dist_i, 0, 0, &params->nav_dist_pm, &params->nav_dist_im, 0, &params->nav_dist_om, 0, 0, 0, 0);
    navData.altSpeedPID = pidInit(&params->nav_alt_speed_p, &params->nav_alt_speed_i, 0, 0, &params->nav_alt_speed_pm, &params->nav_alt_speed_im, 0, &params->nav_alt_speed_om, 0, 0, 0, 0);
    navData.altPosPID =   pidInit(&params->nav_alt_pos_p, &params->nav_alt_pos_i, 0, 0, &params->nav_alt_pos_pm, &params->nav_alt_pos_im, 0, &params->nav_alt_pos_om, 0, 0, 0, 0);

    navData.mode = NAV_STATUS_MANUAL;
    navSetHoldHeading(holdYaw);
    navSetHoldAlt(holdAlt, 0);

    // HOME
    navData.missionLegs[i].type = NAV_LEG_HOME;
    navData.missionLegs[i].targetRadius = 0.10f;
    navData.missionLegs[i].loiterTime = (uint32_t)0.0e6f;
    navData.missionLegs[i].poiAltitude = holdAlt;
    i++;

    // land
    navData.missionLegs[i].type = NAV_LEG_LAND;
    navData.missionLegs[i].maxHorizSpeed = 1.0f;
    navData.missionLegs[i].poiAltitude = 0.0f;
    i++;
}

unsigned int navGetWaypointCount(void) {
    int i;

    for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
	if (navData.missionLegs[i].type == 0)
	    break;

    return i;
}

unsigned char navClearWaypoints(void) {
    unsigned char ack = 0;
    int i;

    if (navData.mode != NAV_STATUS_MISSION) {
	for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
	    navData.missionLegs[i].type = 0;
	ack = 1;
    }

    return ack;
}

navMission_t *navGetWaypoint(int seqId) {
    return &navData.missionLegs[seqId];
}

navMission_t *navGetHomeWaypoint(void) {
    return &navData.homeLeg;
}
