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

#include "nav_flow.h"
#include "nav_flow_ukf.h"
#include <quat/utils/util.h>
#include <quat/utils/quat_constants.h>
#include <quat/utils/pid.h>
#include "nav_flow_ukf.h"
#include <string.h>
#include <math.h>
#include <float.h>
#include <quat/utils/aq_math.h>
#include <stdio.h>
#include <uORB/topics/manual_control_setpoint.h>

#define CTRL_DEAD_BAND (5.0f/1000.0f)

void navFlowSetHoldAlt(float alt, uint8_t relative);
void navFlowSetHoldPosition(const struct filtered_bottom_flow_s* flow_data);
navFlowStruct_t navFlowData __attribute__((section(".ccm")));

void navFlowResetHoldAlt(float delta) {
    navFlowData.holdAlt += delta;
}

void navFlowSetHoldAlt(float alt, uint8_t relative) {
    if (relative)
	navFlowData.holdAlt += alt;
    else
	navFlowData.holdAlt = alt;
}

void navFlowSetHoldPosition(const struct filtered_bottom_flow_s* flow_data) {
	navFlowData.holdPositionX = flow_data->sumx;
	navFlowData.holdPositionY = flow_data->sumy;
}

void navFlowSetHoldHeading(float targetHeading) {
    // signbit() returns true if negative sign
    if (signbit(targetHeading))
	// use targetHeading as relative to bearing to target
	navFlowData.holdHeading = compassNormalizeRad(navFlowData.holdCourse + targetHeading);
    else
	// use targetHeading as absolute heading
	navFlowData.holdHeading = targetHeading;
}

void navFlowNavigate(
		const struct vehicle_status_s *current_status,
		const struct quat_position_control_NAV_params* params,
		const struct manual_control_setpoint_s* manual_control,
		const struct filtered_bottom_flow_s* flow_data,
		uint64_t imu_timestamp
		) {
    uint64_t currentTime = imu_timestamp;
    float tmp;


    if ((currentTime - flow_data->timestamp) < NAV_MAX_FIX_AGE) {
    	navFlowData.navCapable = 1;
    }
    // do not drop out of mission due to (hopefully) temporary GPS degradation
    else if (navFlowData.mode < NAV_STATUS_POSHOLD) {
	// Cannot Navigate
    	navFlowData.navCapable = 0;
    }

    // do we want to be in position hold mode?
    if (current_status->state_machine == SYSTEM_STATE_STABILIZED) {
		// always allow alt hold
		if (navFlowData.mode < NAV_STATUS_ALTHOLD) {
			// record this altitude as the hold altitude
			navFlowSetHoldAlt(UKF_FLOW_PRES_ALT, 0);

			// set integral to current RC throttle setting
			pidZeroIntegral(navFlowData.altSpeedPID, -UKF_FLOW_VELD, manual_control->throttle);
			pidZeroIntegral(navFlowData.altPosPID, UKF_FLOW_PRES_ALT, 0.0f);

			navFlowData.mode = NAV_STATUS_ALTHOLD;
			navFlowData.holdSpeedAlt = -UKF_FLOW_VELD;
			printf("[quat_flow_pos_control]: Altitude hold activated\n");
		}

		// are we not in position hold mode now?
		if (navFlowData.navCapable &&
			navFlowData.mode != NAV_STATUS_POSHOLD &&
			navFlowData.mode != NAV_STATUS_DVH) {

			navFlowSetHoldPosition(flow_data);

			// only zero bias if coming from lower mode
			if (navFlowData.mode < NAV_STATUS_POSHOLD) {
				navFlowData.holdTiltX = 0.0f;
				navFlowData.holdTiltY = 0.0f;
				// distance
				pidZeroIntegral(navFlowData.distanceXPID, 0.0f, 0.0f);
				pidZeroIntegral(navFlowData.distanceYPID, 0.0f, 0.0f);

				// speed
				pidZeroIntegral(navFlowData.speedXPID, UKF_FLOW_VELX, 0.0f);
				pidZeroIntegral(navFlowData.speedYPID, UKF_FLOW_VELY, 0.0f);
			}

			navFlowData.holdMaxHorizSpeed = params->nav_max_speed;
			navFlowData.holdMaxVertSpeed = params->nav_alt_pos_om;

			// activate pos hold
			navFlowData.mode = NAV_STATUS_POSHOLD;
			printf("[quat_flow_pos_control]: Position hold activated\n");
		}
		// DVH
		else if (navFlowData.navCapable && (
			manual_control->pitch > CTRL_DEAD_BAND ||
			manual_control->pitch < -CTRL_DEAD_BAND ||
			manual_control->roll > CTRL_DEAD_BAND ||
			manual_control->roll < -CTRL_DEAD_BAND)) {
				navFlowData.mode = NAV_STATUS_DVH;
				//printf("[quat_flow_pos_control]: DVH activated\n");
		}
		else if (navFlowData.navCapable && navFlowData.mode == NAV_STATUS_DVH) {
			// allow speed to drop before holding position (or if RTH engaged)
			//if (
			//	   (UKF_VELN < +0.1f &&
			//		UKF_VELN > -0.1f &&
			//		UKF_VELE < +0.1f &&
			//		UKF_VELE > -0.1f)
			//		||
			//		RADIO_AUX2 < -250) {
				navFlowData.mode = NAV_STATUS_POSHOLD;
				navFlowData.holdSpeedX = 0.0f;
				navFlowData.holdSpeedY = 0.0f;
				navFlowSetHoldPosition(flow_data);
				printf("[quat_flow_pos_control]: Position hold activated from DVH\n");
			//}
		}
    }
    else {
		// switch to manual mode
		navFlowData.mode = NAV_STATUS_MANUAL;
		// keep up with changing altitude
		navFlowSetHoldAlt(UKF_FLOW_PRES_ALT, 0);
    }

    // DVH
    if (navFlowData.mode == NAV_STATUS_DVH) {
		float factor = navFlowData.holdMaxHorizSpeed;
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

		navFlowData.holdSpeedX = x;
		navFlowData.holdSpeedY = y;
    }
    else {
		// distance => velocity
    	navFlowData.holdSpeedX = pidUpdate(navFlowData.distanceXPID, navFlowData.holdPositionX, flow_data->sumx);
    	navFlowData.holdSpeedY = pidUpdate(navFlowData.distanceYPID, navFlowData.holdPositionY, flow_data->sumy);
    }

    if (fabs(navFlowData.holdSpeedX) > FLT_MIN || fabs(navFlowData.holdSpeedY) > FLT_MIN) {
        // normalize N/E speed requests to fit below max nav speed
        tmp = aq_sqrtf(navFlowData.holdSpeedX*navFlowData.holdSpeedX + navFlowData.holdSpeedY*navFlowData.holdSpeedY);
        if (tmp > navFlowData.holdMaxHorizSpeed) {
    		navFlowData.holdSpeedX = (navFlowData.holdSpeedX / tmp) * navFlowData.holdMaxHorizSpeed;
    		navFlowData.holdSpeedY = (navFlowData.holdSpeedY / tmp) * navFlowData.holdMaxHorizSpeed;
        }
    }
    // velocity => tilt
    navFlowData.holdTiltX = -pidUpdate(navFlowData.speedXPID, navFlowData.holdSpeedX, UKF_FLOW_VELX);
    navFlowData.holdTiltY = +pidUpdate(navFlowData.speedYPID, navFlowData.holdSpeedY, UKF_FLOW_VELY);

    if (navFlowData.mode > NAV_STATUS_MANUAL) {
		float vertStick;

		// Throttle controls vertical speed
		// Throttle is 0 ... 1
		vertStick = (manual_control->throttle * 2.0f) - 1.0f;
		if (vertStick > CTRL_DEAD_BAND || vertStick < -CTRL_DEAD_BAND) {
			// altitude velocity proportional to throttle stick
			// TODO: Assume that throttle control is normalized to -1 ... 1
			if (vertStick > 0.0f)
			navFlowData.targetHoldSpeedAlt = (vertStick - CTRL_DEAD_BAND) * params->nav_alt_pos_om;
			else
			navFlowData.targetHoldSpeedAlt = (vertStick + CTRL_DEAD_BAND) * params->nav_max_decent;

			// set new hold altitude to wherever we are during vertical speed overrides
			if (navFlowData.mode != NAV_STATUS_MISSION)
			navFlowSetHoldAlt(UKF_FLOW_PRES_ALT, 0);
		}
		/*// are we trying to land?
		else if (navFlowData.mode == NAV_STATUS_MISSION && navFlowData.missionLegs[leg].type == NAV_LEG_LAND) {
			navFlowData.targetHoldSpeedAlt = -navFlowData.holdMaxVertSpeed;
		}*/
		else {
			navFlowData.targetHoldSpeedAlt = pidUpdate(navFlowData.altPosPID, navFlowData.holdAlt, UKF_FLOW_PRES_ALT);
		}

		// constrain vertical velocity
		navFlowData.targetHoldSpeedAlt = constrainFloat(navFlowData.targetHoldSpeedAlt,
				(navFlowData.holdMaxVertSpeed < params->nav_max_decent) ? -navFlowData.holdMaxVertSpeed : -params->nav_max_decent, navFlowData.holdMaxVertSpeed);

		// smooth vertical velocity changes
		navFlowData.holdSpeedAlt += (navFlowData.targetHoldSpeedAlt - navFlowData.holdSpeedAlt) * 0.01f;
    }
    navFlowData.lastUpdate = currentTime;
}

void navFlowInit(const struct quat_position_control_NAV_params* params,
				float holdYaw,
				float holdAlt) {
    memset(&navFlowData,0,sizeof(navFlowData));
	navFlowData.holdMaxHorizSpeed = params->nav_max_speed;
	navFlowData.holdMaxVertSpeed = params->nav_alt_pos_om;

    navFlowData.speedXPID = pidInit(&params->nav_speed_p, &params->nav_speed_i, 0, 0, &params->nav_speed_pm, &params->nav_speed_im, 0, &params->nav_speed_om, 0, 0, 0, 0);
    navFlowData.speedYPID = pidInit(&params->nav_speed_p, &params->nav_speed_i, 0, 0, &params->nav_speed_pm, &params->nav_speed_im, 0, &params->nav_speed_om, 0, 0, 0, 0);
    navFlowData.distanceXPID = pidInit(&params->nav_dist_p, &params->nav_dist_i, 0, 0, &params->nav_dist_pm, &params->nav_dist_im, 0, &params->nav_dist_om, 0, 0, 0, 0);
    navFlowData.distanceYPID = pidInit(&params->nav_dist_p, &params->nav_dist_i, 0, 0, &params->nav_dist_pm, &params->nav_dist_im, 0, &params->nav_dist_om, 0, 0, 0, 0);
    navFlowData.altSpeedPID = pidInit(&params->nav_alt_speed_p, &params->nav_alt_speed_i, 0, 0, &params->nav_alt_speed_pm, &params->nav_alt_speed_im, 0, &params->nav_alt_speed_om, 0, 0, 0, 0);
    navFlowData.altPosPID =   pidInit(&params->nav_alt_pos_p, &params->nav_alt_pos_i, 0, 0, &params->nav_alt_pos_pm, &params->nav_alt_pos_im, 0, &params->nav_alt_pos_om, 0, 0, 0, 0);

    navFlowData.mode = NAV_STATUS_MANUAL;
    navFlowSetHoldHeading(holdYaw);
    navFlowSetHoldAlt(holdAlt, 0);
}
