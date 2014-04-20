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
#include <quat/utils/compass_utils.h>
#include <stdio.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/subsystem_info.h>

#define CTRL_DEAD_BAND (60.0f/1000.0f)

static orb_advert_t subsystem_info_pub = -1;
navFlowStruct_t navFlowData __attribute__((section(".ccm")));

void navFlowSetHoldAlt(float alt, uint8_t relative);
void navFlowSetHoldPosition(const float ned_x, const float ned_y);
void navFlowResetHoldPosition(void);
void navCalculateTilt(	const struct vehicle_control_mode_s *control_mode,
						const failsafe_state_t failsafeState,
						const struct quat_position_control_NAV_params* params,
						const struct position_setpoint_s *position_setpoint,
						const struct vehicle_local_position_s* position_data,
						const float manualRoll,
						const float manualPitch,
						const uint8_t navigationMode,
						navFlowStruct_t *navDataResult);
void navCalculateThrust (	const struct vehicle_control_mode_s *control_mode,
							const failsafe_state_t failsafeState,
							const struct quat_position_control_NAV_params* params,
							const struct position_setpoint_s *position_setpoint,
							const float measuredAltitude,
							const float measuredVerticalVelocityNED,
							const float manualThrottle,
							const uint8_t navigationMode,
							const bool preciseAltitude,
							navFlowStruct_t *navDataResult);
float navCalculateTakeoffThrust(const uint64_t timestamp, const float takeoffThrust);

struct subsystem_info_s altitude_control_info = {
	true,
	false,
	false,
	SUBSYSTEM_TYPE_ALTITUDECONTROL
};
struct subsystem_info_s position_control_info = {
	true,
	false,
	false,
	SUBSYSTEM_TYPE_POSITIONCONTROL
};

void navFlowResetHoldAlt(float delta) {
    navFlowData.holdAlt += delta;
}

void navFlowSetHoldAlt(float alt, uint8_t relative) {
    if (relative)
	navFlowData.holdAlt += alt;
    else
	navFlowData.holdAlt = alt;
}

void navFlowSetHoldPosition(const float ned_x, const float ned_y) {
	navFlowData.holdPositionX = ned_x;
	navFlowData.holdPositionY = ned_y;
}

void navFlowResetHoldPosition(void) {
	navFlowData.holdPositionX = 0.0f;
	navFlowData.holdPositionY = 0.0f;
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
		const struct vehicle_control_mode_s *control_mode,
		const struct vehicle_status_s *vstatus,
		const struct quat_position_control_NAV_params* params,
		const struct manual_control_setpoint_s* manual_control,
		const struct vehicle_local_position_s* local_position_data,
		const struct position_setpoint_s *position_setpoint,
		struct vehicle_attitude_s* att,
		const bool preciseAltitude,
		uint64_t imu_timestamp
		) {
    uint64_t currentTime = imu_timestamp;
    const float measured_altitude = local_position_data->z;//-UKF_FLOW_PRES_ALT;//local_position->z;//=UKF_FLOW_PRES_ALT
    const struct vehicle_local_position_s* position_data = local_position_data;

    // 1. Calculate mode
	bool altCapable = control_mode->flag_control_altitude_enabled;
	bool navCapable = altCapable &&
    		control_mode->flag_control_position_enabled &&
    		control_mode->flag_control_velocity_enabled;
	bool missionCapable = control_mode->flag_control_auto_enabled;
	/*if (control_mode->auto_state == NAVIGATION_STATE_AUTO_RTL) {
		navFlowResetHoldPosition();
	}*/

	if (missionCapable){
		if (navFlowData.mode != NAV_STATUS_MISSION) {
			navFlowData.mode = NAV_STATUS_MISSION;
			navPublishSystemInfo();
			printf("[quat_flow_pos_control]: Mission mode activated\n");
		}
	}
   // do we want to be in position hold mode?
	else if (navCapable) {
		// are we not in position hold mode now?
		if (navFlowData.mode != NAV_STATUS_POSHOLD &&
			navFlowData.mode != NAV_STATUS_DVH) {

			// record this position as hold position
			navFlowSetHoldPosition(position_data->x, position_data->y);
			// record this altitude as the hold altitude
			navFlowSetHoldAlt(measured_altitude, 0);

			// only zero bias if coming from lower mode
			if (navFlowData.mode < NAV_STATUS_POSHOLD) {
				navFlowData.holdTiltX = 0.0f;
				navFlowData.holdTiltY = 0.0f;
				// distance
				pidZeroIntegral(navFlowData.distanceXPID, 0.0f, 0.0f);
				pidZeroIntegral(navFlowData.distanceYPID, 0.0f, 0.0f);
				// speed
				pidZeroIntegral(navFlowData.speedXPID, position_data->vx, 0.0f);
				pidZeroIntegral(navFlowData.speedYPID, position_data->vy, 0.0f);
			}
			// only reset integral if coming from lower state
			if(navFlowData.mode < NAV_STATUS_ALTHOLD) {
				// set integral to current RC throttle setting
				// there is a minus because the pidUpdate also has a minus
				pidZeroIntegral(navFlowData.altSpeedPID, -position_data->vz, manual_control->throttle);
				pidZeroIntegral(navFlowData.altPosPID, measured_altitude, 0.0f);
				navFlowData.holdSpeedAlt = -position_data->vz;
			}

			navFlowData.holdMaxHorizSpeed = params->nav_max_speed;
			navFlowData.holdMaxVertSpeed = params->nav_alt_pos_om;

			// activate pos hold
			navFlowData.mode = NAV_STATUS_POSHOLD;
			navPublishSystemInfo();
			printf("[quat_flow_pos_control]: Position hold activated\n");
		}
		// DVH
		else if (navFlowData.mode != NAV_STATUS_DVH &&
			(fabsf(manual_control->pitch) > CTRL_DEAD_BAND ||
			 fabsf(manual_control->roll) > CTRL_DEAD_BAND)) {
				navFlowData.mode = NAV_STATUS_DVH;
				navPublishSystemInfo();
				//printf("[quat_flow_pos_control]: DVH activated\n");
		}
		else if (navFlowData.mode == NAV_STATUS_DVH &&
				fabsf(manual_control->pitch) < CTRL_DEAD_BAND &&
				fabsf(manual_control->roll) < CTRL_DEAD_BAND) {
			// allow speed to drop before holding position (or if RTH engaged)
			//if (
			//	   (UKF_VELN < +0.1f &&
			//		UKF_VELN > -0.1f &&
			//		UKF_VELE < +0.1f &&
			//		UKF_VELE > -0.1f)
			//		||
			//		RADIO_AUX2 < -250) {
				navFlowData.mode = NAV_STATUS_POSHOLD;
				navPublishSystemInfo();
				navFlowData.holdSpeedX = 0.0f;
				navFlowData.holdSpeedY = 0.0f;
				navFlowSetHoldPosition(position_data->x, position_data->y);
				printf("[quat_flow_pos_control]: Position hold activated from DVH\n");
			//}
		}
	}
	// not nav capable
	else if ( altCapable &&
			navFlowData.mode != NAV_STATUS_ALTHOLD) {
		// only reset integral if coming from lower state
		if(navFlowData.mode < NAV_STATUS_ALTHOLD) {
			// set integral to current RC throttle setting
			// there is a minus because the pidUpdate also has a minus
			pidZeroIntegral(navFlowData.altSpeedPID, -position_data->vz, manual_control->throttle);
			pidZeroIntegral(navFlowData.altPosPID, measured_altitude, 0.0f);
			navFlowData.holdSpeedAlt = -position_data->vz;
		}
		// record this altitude as the hold altitude
		navFlowSetHoldAlt(measured_altitude, 0);
		// switch to althold
		navFlowData.mode = NAV_STATUS_ALTHOLD;
		navPublishSystemInfo();
		printf("[quat_flow_pos_control]: Altitude hold activated\n");
	}
	// not nav capable and not alt capable
    else if ( !altCapable &&
    		navFlowData.mode >= NAV_STATUS_ALTHOLD){
		// switch to manual mode
		navFlowData.mode = NAV_STATUS_MANUAL;
		navPublishSystemInfo();
    }

    // 2. Do navigation
	navCalculateTilt( 	control_mode,
						vstatus->failsafe_state,
						params,
    					position_setpoint,
    					position_data,
						manual_control->roll,
						manual_control->pitch,
    					navFlowData.mode,
    					&navFlowData);

    navCalculateThrust(	control_mode,
						vstatus->failsafe_state,
    					params,
    					position_setpoint,
    					measured_altitude,
    					local_position_data->vz,
    					manual_control->throttle,
    					navFlowData.mode,
    					preciseAltitude,
    					&navFlowData);

    navFlowData.lastUpdate = currentTime;
}

void navCalculateTilt(	const struct vehicle_control_mode_s *control_mode,
						const failsafe_state_t failsafeState,
						const struct quat_position_control_NAV_params* params,
						const struct position_setpoint_s *position_setpoint,
						const struct vehicle_local_position_s* position_data,
						const float manualRoll,
						const float manualPitch,
						const uint8_t navigationMode,
						navFlowStruct_t *navDataResult) {
	if (failsafeState > FAILSAFE_STATE_NORMAL) {
		navDataResult->holdSpeedX = 0.0f;
		navDataResult->holdSpeedY = 0.0f;
	}
	else if (navigationMode == NAV_STATUS_MISSION) {
    	if (position_setpoint->type == SETPOINT_TYPE_NORMAL) {
			//navDataResult->holdSpeedX = pidUpdate(navDataResult->distanceXPID, local_position_setpoint->x, position_data->x);
			//navDataResult->holdSpeedY = pidUpdate(navDataResult->distanceYPID, local_position_setpoint->y, position_data->y);
    		navDataResult->holdSpeedX = pidUpdate(navDataResult->distanceXPID, navDataResult->holdPositionX, position_data->x);
    		navDataResult->holdSpeedY = pidUpdate(navDataResult->distanceYPID, navDataResult->holdPositionY, position_data->y);
    	} else if (position_setpoint->type == SETPOINT_TYPE_LOITER) {
    		navDataResult->holdSpeedX = pidUpdate(navDataResult->distanceXPID, navDataResult->holdPositionX, position_data->x);
    		navDataResult->holdSpeedY = pidUpdate(navDataResult->distanceYPID, navDataResult->holdPositionY, position_data->y);
    	} else if (position_setpoint->type == SETPOINT_TYPE_TAKEOFF) {
			navDataResult->holdSpeedX = 0.0f;
			navDataResult->holdSpeedY = 0.0f;
    	} else if (position_setpoint->type == SETPOINT_TYPE_LAND) {
			navDataResult->holdSpeedX = 0.0f;
			navDataResult->holdSpeedY = 0.0f;
    	} else if (position_setpoint->type == SETPOINT_TYPE_IDLE) {
			navDataResult->holdSpeedX = 0.0f;
			navDataResult->holdSpeedY = 0.0f;
    	}
    }
    else if (navigationMode == NAV_STATUS_DVH) {
        // DVH
		float factor = navDataResult->holdMaxHorizSpeed;
		float x = 0.0f;
		float y = 0.0f;

		if (manualPitch > CTRL_DEAD_BAND)
			x = -(manualPitch - CTRL_DEAD_BAND) * factor;
		if (manualPitch < -CTRL_DEAD_BAND)
			x = -(manualPitch + CTRL_DEAD_BAND) * factor;
		if (manualRoll > CTRL_DEAD_BAND)
			y = +(manualRoll - CTRL_DEAD_BAND) * factor;
		if (manualRoll < -CTRL_DEAD_BAND)
			y = +(manualRoll + CTRL_DEAD_BAND) * factor;

		navDataResult->holdSpeedX = x;
		navDataResult->holdSpeedY = y;
		navFlowSetHoldPosition(position_data->x, position_data->y);
    }
    else {
		// distance => velocity
    	navDataResult->holdSpeedX = pidUpdate(navDataResult->distanceXPID, navDataResult->holdPositionX, position_data->x);
    	navDataResult->holdSpeedY = pidUpdate(navDataResult->distanceYPID, navDataResult->holdPositionY, position_data->y);
    }

    if (fabs(navDataResult->holdSpeedX) > FLT_MIN || fabs(navDataResult->holdSpeedY) > FLT_MIN) {
        // normalize N/E speed requests to fit below max nav speed
        float speedValue = aq_sqrtf(navDataResult->holdSpeedX*navDataResult->holdSpeedX + navDataResult->holdSpeedY*navDataResult->holdSpeedY);
        if (speedValue > navDataResult->holdMaxHorizSpeed) {
    		navDataResult->holdSpeedX = (navDataResult->holdSpeedX / speedValue) * navDataResult->holdMaxHorizSpeed;
    		navDataResult->holdSpeedY = (navDataResult->holdSpeedY / speedValue) * navDataResult->holdMaxHorizSpeed;
        }
    }
    // velocity => tilt
    navDataResult->holdTiltX = pidUpdate(navDataResult->speedXPID, navDataResult->holdSpeedX, position_data->vx);
    navDataResult->holdTiltY = pidUpdate(navDataResult->speedYPID, navDataResult->holdSpeedY, position_data->vy);
}

void navCalculateThrust(	const struct vehicle_control_mode_s *control_mode,
							const failsafe_state_t failsafeState,
							const struct quat_position_control_NAV_params* params,
							const struct position_setpoint_s *position_setpoint,
							const float measuredAltitude,
							const float measuredVerticalVelocityNED,
							const float manualThrottle,
							const uint8_t navigationMode,
							const bool preciseAltitude,
							navFlowStruct_t *navDataResult) {
    // Calculate thrust
    // Reset result
    navDataResult->autoThrust = 0.0f;
    static float throttle_middle_position = 1.0f;

	if (failsafeState > FAILSAFE_STATE_NORMAL) {
		navDataResult->targetHoldSpeedAlt = +0.5f;
	}
	else if (navigationMode == NAV_STATUS_MISSION) {
    	if (position_setpoint->type == SETPOINT_TYPE_NORMAL) {
			//navDataResult->targetHoldSpeedAlt = pidUpdate(navDataResult->altPosPID, local_position_setpoint->z, measuredAltitude);
			navDataResult->targetHoldSpeedAlt = pidUpdate(navDataResult->altPosPID, navDataResult->holdAlt, measuredAltitude);
    	} else if (position_setpoint->type == SETPOINT_TYPE_LOITER) {
			navDataResult->targetHoldSpeedAlt = pidUpdate(navDataResult->altPosPID, navDataResult->holdAlt, measuredAltitude);
    	} else if (position_setpoint->type == SETPOINT_TYPE_TAKEOFF) {
			// set thrust hard coded
			navDataResult->autoThrust = navCalculateTakeoffThrust(navDataResult->lastUpdate,params->nav_takeoff_thrust);
			pidZeroIntegral(navFlowData.altSpeedPID, -measuredVerticalVelocityNED, navDataResult->autoThrust);
			pidZeroIntegral(navFlowData.altPosPID, measuredAltitude, 0.0f);
			navFlowSetHoldAlt(measuredAltitude, 0);
    	} else if (position_setpoint->type == SETPOINT_TYPE_LAND) {
    		navDataResult->targetHoldSpeedAlt = +0.5f;
    	} else if (position_setpoint->type == SETPOINT_TYPE_IDLE) {
			navDataResult->targetHoldSpeedAlt = pidUpdate(navDataResult->altPosPID, navDataResult->holdAlt, measuredAltitude);
    	}
    	// constrain horizontal velocity
		navDataResult->targetHoldSpeedAlt = constrainFloat(navDataResult->targetHoldSpeedAlt,
    					(navDataResult->holdMaxVertSpeed < params->nav_max_decent) ? -navDataResult->holdMaxVertSpeed : -params->nav_max_decent, navDataResult->holdMaxVertSpeed);
    	navDataResult->holdSpeedAlt += (navDataResult->targetHoldSpeedAlt - navDataResult->holdSpeedAlt) * 0.01f;
    }
    else if (navigationMode > NAV_STATUS_MANUAL) {
		float vertStick;

		// Throttle controls vertical speed
		// Throttle is 0 ... 1
		// Make sure that current throttle gets the hold position
		// But also assure that remaining throttle range is not too small
		if(throttle_middle_position < 0.5f || throttle_middle_position > 1.5f) throttle_middle_position = 1.0f;
		vertStick = (manualThrottle * 2.0f) - throttle_middle_position;
		if (vertStick > CTRL_DEAD_BAND || vertStick < -CTRL_DEAD_BAND) {
			// altitude velocity negative proportional to throttle stick
			if (vertStick > 0.0f) {
				// positive stick
				vertStick = vertStick / (2.0f - throttle_middle_position) / (1.0f - CTRL_DEAD_BAND);
				if((vertStick - CTRL_DEAD_BAND) > +1.0f) {
					vertStick = +1.0f;
				}
				navDataResult->targetHoldSpeedAlt = -(vertStick - CTRL_DEAD_BAND) * params->nav_alt_pos_om;
			}
			else {
				// negative stick
				vertStick = vertStick / (throttle_middle_position) / (1.0f - CTRL_DEAD_BAND);
				if((vertStick + CTRL_DEAD_BAND) < -1.0f) {
					vertStick = -1.0f;
				}
				navDataResult->targetHoldSpeedAlt = -(vertStick + CTRL_DEAD_BAND) * params->nav_max_decent;
			}
			// set new hold altitude to wherever we are during vertical speed overrides
			navFlowSetHoldAlt(measuredAltitude, 0);
		}
		else {
			// for positive and negative altitude: if measured > hold -> speed negative
			// PID: p-term ~ setpoint - position = holdAlt - measured -> pidUpdate has + sign
			navDataResult->targetHoldSpeedAlt = pidUpdate(navDataResult->altPosPID, navDataResult->holdAlt, measuredAltitude);
		}

		// constrain vertical velocity
		navDataResult->targetHoldSpeedAlt = constrainFloat(navDataResult->targetHoldSpeedAlt,
				(navDataResult->holdMaxVertSpeed < params->nav_max_decent) ? -navDataResult->holdMaxVertSpeed : -params->nav_max_decent, navDataResult->holdMaxVertSpeed);

		// smooth vertical velocity changes
		//float smoothfactor = 0.01f;
//		if (preciseAltitude) {
//			smoothfactor *= 10.0f;
//		}
		navDataResult->holdSpeedAlt += (navDataResult->targetHoldSpeedAlt - navDataResult->holdSpeedAlt) * 0.1f;
    }
    else
    {
    	// In manual mode: remember last throttle position
		// set this throttle position as middle position
		throttle_middle_position = manualThrottle * 2.0f;
    }
    if(navDataResult->autoThrust < FLT_MIN){
    	// no thrust override
    	// speed down is negative, if -navDataResult.holdSpeedAlt > -measuredVerticalVelocityNED -> thrust positive
        navDataResult->autoThrust = pidUpdate(navDataResult->altSpeedPID, -navDataResult->holdSpeedAlt, -measuredVerticalVelocityNED);
    }
	if ( navDataResult->autoThrust < 0.0f ) {
		navDataResult->autoThrust = 0.0f;
	}
}


float navCalculateTakeoffThrust(const uint64_t timestamp, const float takeoffThrust){
	// set thrust hard coded
	float thrustResult = 0.0f;
	static uint64_t takeoffStart = 0;
	static uint64_t lastTakeoffCycle = 0;
	if((timestamp - lastTakeoffCycle) > 1000000) {
		// one second since start, assume new takeoff
		takeoffStart = timestamp;
	}
	lastTakeoffCycle = timestamp;
	// Calculate motor thrust to accelerate in 5s to full speed
	static const uint64_t accelerationTimeMicroSeconds = 5000000;
	int64_t timeToFullSpeed = accelerationTimeMicroSeconds - (timestamp - takeoffStart);
	if(timeToFullSpeed < 0) timeToFullSpeed = 0;
	thrustResult = takeoffThrust *
			(float)(accelerationTimeMicroSeconds - timeToFullSpeed)/(float)accelerationTimeMicroSeconds;
	if(thrustResult < 0.1f) thrustResult = 0.1f;
	return thrustResult;
}

void navPublishSystemInfo(void) {
	static uint8_t oldMode = NAV_STATUS_MANUAL;
	if(navFlowData.mode == oldMode) {
		return;
	}
	oldMode = navFlowData.mode;

	if(navFlowData.mode > NAV_STATUS_MANUAL) {
		// alt hold enabled
		/* notify about state change */
		altitude_control_info.enabled = true;
		altitude_control_info.ok = true;
		orb_publish(ORB_ID(subsystem_info), subsystem_info_pub, &altitude_control_info);
	}
	else {
		altitude_control_info.enabled = false;
		altitude_control_info.ok = false;
		orb_publish(ORB_ID(subsystem_info), subsystem_info_pub, &altitude_control_info);
	}
	if(navFlowData.mode > NAV_STATUS_ALTHOLD) {
		// alt hold enabled
		/* notify about state change */
		position_control_info.enabled = true;
		position_control_info.ok = true;
		orb_publish(ORB_ID(subsystem_info), subsystem_info_pub, &position_control_info);
	}
	else {
		position_control_info.enabled = false;
		position_control_info.ok = false;
		orb_publish(ORB_ID(subsystem_info), subsystem_info_pub, &position_control_info);
	}
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
	/* notify about state change */
    subsystem_info_pub = orb_advertise(ORB_ID(subsystem_info), &altitude_control_info);
    subsystem_info_pub = orb_advertise(ORB_ID(subsystem_info), &position_control_info);
}
