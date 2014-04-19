#include "nav_flow_ukf.h"
#include <systemlib/conversions.h>
#include <quat/utils/quat_constants.h>
#include "nav_flow.h"
#include <float.h>
#include <math.h>
#include <geo/geo.h>
#include <quat/utils/aq_math.h>
#include <quat/utils/util.h>
#include <quat/utils/compass_utils.h>
#include <quat/utils/srcdkf.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/filtered_bottom_flow.h>

navFlowUkfStruct_t navFlowUkfData __attribute__((section(".ccm")));
static uint64_t navFlowLastInertialUpdate = 0;

void navFlowUkfCalcEarthRadius(double lat);
void navFlowUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast);
void navFlowUkfResetPosition(float deltaN, float deltaE, float deltaD);
void navFlowUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, bool updateBias);
void navFlowUkfRateUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfInitState(const struct sensor_combined_s* sensors);
void navFlowUkfAccUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfMagUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfPresUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfAltitudeUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfAltitudeVelocityUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfVelocityUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfPositionUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfVelocityPositionUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfVelAltUpdate(float *u, float *x, float *noise, float *y);
void navFlowUkfPosAltUpdate(float *u, float *x, float *noise, float *y);
void navFlowCalculateOffsets(
		const struct filtered_bottom_flow_s* bottom_flow,
		float baroAltitude,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);

bool navFlowIsArmed(const struct vehicle_control_mode_s *control_mode){
	return (control_mode->flag_armed);
}

void navFlowUkfSetSonarOffset(const float sonarDistanceToEarth, const float altitude, const float kSonarBaro) {
	float offsetError = sonarDistanceToEarth + navFlowUkfData.sonarAltOffset - altitude;
	//sonarAltOffset =  -sonarDistanceToEarth + altitude);
	//altitude = sonarAltOffset + sonarDistanceToEarth
	navFlowUkfData.sonarAltOffset -= (offsetError * kSonarBaro);
}

void navFlowUkfSetPressAltOffset(const float baroAltitude, const float altitude,  const float kSonarBaro) {
	float offsetError = baroAltitude + navFlowUkfData.pressAltOffset - altitude;
	navFlowUkfData.pressAltOffset -= (offsetError * kSonarBaro);
}

void navFlowUkfCalcEarthRadius(double lat) {
    float sinLat2;

    sinLat2 = __aq_sinf(lat * DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navFlowUkfData.r1 = NAV_EQUATORIAL_RADIUS * DEG_TO_RAD * (1.0f - NAV_E_2) / powf(1.0f - (NAV_E_2 * sinLat2), (3.0f / 2.0f));
    navFlowUkfData.r2 = NAV_EQUATORIAL_RADIUS * DEG_TO_RAD / aq_sqrtf(1.0f - (NAV_E_2 * sinLat2));
}

void navFlowUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navFlowUkfData.holdLat) * navFlowUkfData.r1;
    *posEast = (lon - navFlowUkfData.holdLon) * __aq_cosf(lat * DEG_TO_RAD) * navFlowUkfData.r2;
}

void navFlowUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
    	navFlowUkfData.posX[i] += deltaN;
    	navFlowUkfData.posY[i] += deltaE;
    	navFlowUkfData.posD[i] += deltaD;
    }

    UKF_FLOW_POSX += deltaN;
    UKF_FLOW_POSY += deltaE;
    UKF_FLOW_POSD += deltaD;

//    UKF_PRES_BIAS += deltaD;

    navFlowResetHoldAlt(deltaD);
}

void navFlowUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosX, oldPosY;
    float newPosX, newPosY;

    navFlowUkfCalcDistance(lat, lon, &oldPosX, &oldPosY);

    navFlowUkfData.holdLat = lat;
    navFlowUkfData.holdLon = lon;

    navFlowUkfCalcDistance(lat, lon, &newPosX, &newPosY);

    navFlowUkfResetPosition(newPosX - oldPosX, newPosY - oldPosY, 0.0f);
}

void navFlowUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, bool updateBias) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];
    float *noiseVector;
    if(true) {
		//with bias noise
		// acc bias
		out[0] = in[0] + noise[0] * dt;
		out[1] = in[1] + noise[1] * dt;
		out[2] = in[2] + noise[2] * dt;

		// gbias
		out[3] = in[3] + noise[3] * dt;
		out[4] = in[4] + noise[4] * dt;
		out[5] = in[5] + noise[5] * dt;

		noiseVector = &noise[6];
    }
    else {
        // acc bias
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];

        // gbias
        out[3] = in[3];
        out[4] = in[4];
        out[5] = in[5];

        noiseVector = noise;
    }

    // rate = rate + bias + noise
    rate[0] = (u[3] + out[3] + noiseVector[0]) * dt;
    rate[1] = (u[4] + out[4] + noiseVector[1]) * dt;
    rate[2] = (u[5] + out[5] + noiseVector[2]) * dt;

    // rotate
    utilRotateQuat(&out[9], &in[9], rate, dt);
    utilQuatToMatrix(mat3x3, &out[9], 1);

    /*
	printf("In 1:%8.4f 2:%8.4f 3:%8.4f 4:%8.4f  R: 1:%8.4f 2:%8.4f 3:%8.4f 4:%8.4f 5:%8.4f 6:%8.4f 7:%8.4f 8:%8.4f 9:%8.4f dt:%8.4f 1:%8.4f 2:%8.4f 3:%8.4f 4:%8.4f\n",
			in[9],in[10],in[11],in[12],
			u[3],out[3],noiseVector[0],
			u[4],out[4],noiseVector[1],
			u[5],out[5],noiseVector[2],dt,
			out[9],out[10],out[11],out[12]);
*/
    // acc
    tmp[0] = u[0] + out[0];
    tmp[1] = u[1] + out[1];
    tmp[2] = u[2] + out[2];

    // rotate acc to world ned frame
    utilRotateVecByMatrix(acc, tmp, mat3x3);
    acc[2] += CONSTANTS_ONE_G;

    // vel ned, down acc positiv, down speed positiv
    out[6] = in[6] + acc[0] * dt + noiseVector[3];
    out[7] = in[7] + acc[1] * dt + noiseVector[4];
    out[8] = in[8] + acc[2] * dt + noiseVector[5];

    // pres alt
    out[13] = in[13] - (in[8] + out[8]) * 0.5f * dt + noiseVector[6];

    if(UKF_FLOW_CALCULATES_POSITION) {
    	// pos
    	out[14] = in[14] + (in[6] + out[6]) * 0.5f * dt + noiseVector[7];
    	out[15] = in[15] + (in[7] + out[7]) * 0.5f * dt + noiseVector[8];
    }
    if(UKF_FLOW_CALCULATES_ALTITUDE){
    	// up vel negativ, up position negativ
    	out[16] = in[16] + (in[8] + out[8]) * 0.5f * dt + noiseVector[9];
    }
}

void navFlowUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[3+(int)u[0]] + noise[0];
}

void navFlowUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    utilRotateVectorByRevQuat(y, navFlowUkfData.v0a, &x[9]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navFlowUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    utilRotateVectorByRevQuat(y, navFlowUkfData.v0m, &x[9]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navFlowUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[13] + noise[0]; // return altitude
}

void navFlowUkfAltitudeUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[16] + noise[0]; // return altitude
}

void navFlowUkfAltitudeVelocityUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[8] + noise[0]; // return velocity
    y[1] = x[16] + noise[1];// return position
}

void navFlowUkfVelocityUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[6] + noise[0]; // return velocity
    y[1] = x[7] + noise[1];
}

void navFlowUkfPositionUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[14] + noise[0]; // return position
    y[1] = x[15] + noise[1];
}

void navFlowUkfVelocityPositionUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[6] + noise[0]; // return velocity
    y[1] = x[7] + noise[1];
    y[2] = x[14] + noise[2]; // return position
    y[3] = x[15] + noise[3];
}

void navFlowUkfVelAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[6] + noise[0]; // return velocity
    y[1] = x[7] + noise[1];
    y[2] = x[8] + noise[2];
}

void navFlowUkfPosAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[14] + noise[0]; // return position
    y[1] = x[15] + noise[1];
    y[2] = x[16] + noise[2];
}

void navFlowUkfFinish(void) {
	float yaw, pitch, roll;
    utilNormalizeQuat(&UKF_FLOW_Q1, &UKF_FLOW_Q1);
    utilQuatExtractEuler(&UKF_FLOW_Q1, &yaw, &pitch, &roll);
    navFlowUkfData.roll = roll;
    navFlowUkfData.pitch = pitch;
    yaw = compassNormalizeRad(yaw);
    navFlowUkfData.yaw = yaw;

    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f
    //navFlowUkfData.yawCos = cosf(navFlowUkfData.yaw);
    //navFlowUkfData.yawSin = sinf(navFlowUkfData.yaw);
}

float navFlowUkfInertialUpdate(const struct sensor_combined_s* raw, bool updateBias) {

	/* Calculate data time difference in seconds */
	float dt = 0;
	uint64_t currentTime = hrt_absolute_time();
	if(navFlowLastInertialUpdate != 0) {
		dt = ((float)(currentTime - navFlowLastInertialUpdate)) / 1e6f;
	}
	navFlowLastInertialUpdate = currentTime;
	if(dt < FLT_MIN) return 0.0f;

    float u[6];

    u[0] = raw->accelerometer_m_s2[0];
    u[1] = raw->accelerometer_m_s2[1];
    u[2] = raw->accelerometer_m_s2[2];

    u[3] = raw->gyro_rad_s[0];
    u[4] = raw->gyro_rad_s[1];
    u[5] = raw->gyro_rad_s[2];

    srcdkfTimeUpdateReduced(navFlowUkfData.kf, u, dt, updateBias);

    // store history
    if(UKF_FLOW_CALCULATES_POSITION) {
    	navFlowUkfData.posX[navFlowUkfData.navHistIndex] = UKF_FLOW_POSX;
    	navFlowUkfData.posY[navFlowUkfData.navHistIndex] = UKF_FLOW_POSY;
    }
    if(UKF_FLOW_CALCULATES_ALTITUDE) {
    	navFlowUkfData.posD[navFlowUkfData.navHistIndex] = UKF_FLOW_POSD;
    }

    navFlowUkfData.velX[navFlowUkfData.navHistIndex] = UKF_FLOW_VELX;
    navFlowUkfData.velY[navFlowUkfData.navHistIndex] = UKF_FLOW_VELY;
    navFlowUkfData.velD[navFlowUkfData.navHistIndex] = UKF_FLOW_VELD;

    navFlowUkfData.navHistIndex = (navFlowUkfData.navHistIndex + 1) % UKF_HIST;

    return dt;
}

void navFlowUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navFlowUkfData.kf, u, y, 1, 1, noise, navFlowUkfRateUpdate);
}

void navFlowDoPresUpdate(float pres,
					 const struct vehicle_control_mode_s *control_mode,
					 const struct quat_position_control_UKF_params* params) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)

    noise[0] = params->ukf_alt_n;

    y[0] = utilPresToAlt(pres) + navFlowUkfData.pressAltOffset;

   	srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 1, 1, noise, navFlowUkfPresUpdate);
}

float navFlowDoAccUpdate(float accX, float accY, float accZ,
		 const struct vehicle_control_mode_s *control_mode,
		 const struct quat_position_control_UKF_params* params) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_FLOW_ACC_BIAS_X;
    accY += UKF_FLOW_ACC_BIAS_Y;
    accZ += UKF_FLOW_ACC_BIAS_Z;

    // normalize vector
    norm =  aq_sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = params->ukf_acc_n + fabsf(CONSTANTS_ONE_G - norm) * params->ukf_dist_n;
    //printf("Noise:%8.4f\n",noise[0]);
    if (!control_mode->flag_armed) {
    	noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfAccUpdate);
    return noise[0];
}

void navFlowDoMagUpdate(float magX, float magY, float magZ,
		 const struct vehicle_control_mode_s *control_mode,
		 const struct quat_position_control_UKF_params* params) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = params->ukf_mag_n;

   // if (!control_mode->flag_armed) {
    //	noise[0] = 0.001f;
    //}

    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / aq_sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    //srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfMagUpdate);


    float est[3];
	// rotate mags to body frame of reference
	utilRotateVectorByRevQuat(est,navFlowUkfData.v0m,&UKF_FLOW_Q1);

	float rotError[3];
	// measured error
	rotError[0] = -(y[2] * est[1] - est[2] * y[1]) * params->ukf_mag_n;
	rotError[1] = -(y[0] * est[2] - est[0] * y[2]) * params->ukf_mag_n;
	rotError[2] = -(y[1] * est[0] - est[1] * y[0]) * params->ukf_mag_n;
	/* Calculate data time difference in seconds */
	float dt = 0;
	uint64_t currentTime = hrt_absolute_time();
	static uint64_t lastUpdate = 0;
	if(lastUpdate != 0) {
		dt = ((float)(currentTime - lastUpdate)) / 1e6f;
	}
    utilRotateQuat(&UKF_FLOW_Q1, &UKF_FLOW_Q1, rotError, dt);
/*
    static int printcounter = 0;
    if (!(printcounter % 20)){
    	float32_t *x = &UKF_FLOW_Q1;
    	utilRotateVectorByRevQuat(y, navFlowUkfData.v0m, x);
    	printf("m: %8.4f %8.4f %8.4f\ne: %8.4f %8.4f %8.4f\n0: %8.4f %8.4f %8.4fe: %8.4f %8.4f %8.4f\n",
    			magX * norm,magY * norm,magZ * norm,
    			est[0],est[1],est[2],
    			navFlowUkfData.v0m[0],navFlowUkfData.v0m[1],navFlowUkfData.v0m[2],
    			rotError[0], rotError[1], rotError[2]);
    }
    printcounter++;*/
}

void navFlowCalculateOffsets(
		const struct filtered_bottom_flow_s* bottom_flow,
		float baroAltitude,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {

	//static uint8_t printcounter = 0;
	static bool sonarInvalid = true;

	// Calculate offsets
	const float distanceToEarth = -bottom_flow->ned_z; //Positive value
	float referenceAltitude;
	if(UKF_FLOW_CALCULATES_ALTITUDE){
		referenceAltitude = -UKF_FLOW_POSD;
	} else {
		referenceAltitude = UKF_FLOW_PRES_ALT;
	}
	if((bottom_flow->ned_z_valid == 255) && sonarInvalid)  {
		sonarInvalid = false;
		// firstTime Valid
		// set offset
		navFlowUkfSetSonarOffset(distanceToEarth, referenceAltitude, 1.0f);
	}
	if(bottom_flow->ned_z_valid < 255) {
		sonarInvalid = true;
		// sonar not valid
		// don't change pressure alt offset because this is used
		// TODO: Change pressure offset according to GPS
		if(!control_mode->flag_armed) {
			//not armed, so we know distance to earth must be 0.0
			navFlowUkfSetSonarOffset(0.0f, referenceAltitude, 1.0f);

			/*if (!(printcounter % 10)) printf("Set Sonar Offset to 0.0 reference: %8.4fm\n",referenceAltitude);
			printcounter++;*/
		}
		//navFlowUkfSetSonarOffset(0.3f, UKF_FLOW_PRES_ALT, 1.0f);
	} else if (control_mode->flag_control_altitude_enabled) {
		// altitude hold mode using sonar
		// don't change sonar offset because it is primary altitude source
		// But change pressure offset slowly
		navFlowUkfSetPressAltOffset(baroAltitude, referenceAltitude, params->ukf_pres_alt_k);

		//if (!(printcounter % 10)) printf("Set pressure Offset to %8.4fm reference: %8.4fm\n",baroAltitude,referenceAltitude);
		//printcounter++;
	} else {
		//no altitude hold but sonar working
		//Change sonar offset slowly
		//TODO: use GPS altitude here
		//altitude = sonarAltOffset + bottom_flow->ned_z
		navFlowUkfSetSonarOffset(distanceToEarth, UKF_FLOW_PRES_ALT, params->ukf_pres_alt_k);

		//if (!(printcounter % 10)) printf("Set Sonar Offset to %8.4fm reference: %8.4fm\n",distanceToEarth,referenceAltitude);
		//printcounter++;
	}
}

void navFlowUkfSonarUpdate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const float dt,
		float baroAltitude,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {

    float y[2];
    float noise[2];
    float posDelta, velDelta;
    posDelta = 0.0f;
    velDelta = 0.0f;
    //static uint32_t sonarCount = 0;
    //static uint8_t printcounter = 0;

    /*if(bottom_flow->sonar_counter <= sonarCount) {
    	return;
    }
    sonarCount = bottom_flow->sonar_counter;
*/
    navFlowCalculateOffsets(bottom_flow, baroAltitude, control_mode, params);
	const float distanceToEarth = -bottom_flow->ned_z; //Positive value
	if (UKF_FLOW_CALCULATES_ALTITUDE){
		if(bottom_flow->ned_z_valid == 255) {
			// determine how far back this sonar position update came from
			int histIndexPos = (hrt_absolute_time() - (bottom_flow->timestamp + params->ukf_sonar_pos_delay)) / (int)(1e6f * dt);
			histIndexPos = navFlowUkfData.navHistIndex - histIndexPos;
			if (histIndexPos < 0) histIndexPos += UKF_HIST;
			if (histIndexPos < 0 || histIndexPos >= UKF_HIST) histIndexPos = 0;
			int histIndexVel = (hrt_absolute_time() - (bottom_flow->timestamp + params->ukf_sonar_vel_delay)) / (int)(1e6f * dt);
			histIndexVel = navFlowUkfData.navHistIndex - histIndexVel;
			if (histIndexVel < 0) histIndexVel += UKF_HIST;
			if (histIndexVel < 0 || histIndexVel >= UKF_HIST) histIndexVel = 0;
			// calculate delta from current position
			posDelta = UKF_FLOW_POSD - navFlowUkfData.posD[histIndexPos];
			// calculate delta from current velocity
			velDelta = UKF_FLOW_VELD - navFlowUkfData.velD[histIndexVel];
			// set current position state to historic data
			UKF_FLOW_POSD = navFlowUkfData.posD[histIndexPos];
			// set current velocity state to historic data
			UKF_FLOW_VELD = navFlowUkfData.velD[histIndexVel];

			y[0] = bottom_flow->ned_vz;
			y[1] = -(distanceToEarth + navFlowUkfData.sonarAltOffset);
			noise[0] = params->ukf_flow_vel_n;
			noise[1] = params->ukf_flow_alt_n;
			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfAltitudeVelocityUpdate);

			UKF_FLOW_POSD += posDelta;
			UKF_FLOW_VELD += velDelta;
			/*if (!(printcounter % 10)) printf("Seas Up: %8.4f, posd: %8.4f, baroalt: %8.4f\n",y[1],UKF_FLOW_POSD, baroAltitude);
			printcounter++;*/
		} else {
			y[0] = -UKF_FLOW_PRES_ALT;
			noise[0] = params->ukf_alt_n;
			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 1, 1, noise, navFlowUkfAltitudeUpdate);
			/*if (!(printcounter % 10)) printf("Meas Up: %8.4f, posd: %8.4f, baroalt: %8.4f\n",y[0],UKF_FLOW_POSD, baroAltitude);
			printcounter++;*/
		}
	} else {
		if(bottom_flow->ned_z_valid == 255) {
			y[0] = distanceToEarth + navFlowUkfData.sonarAltOffset;
			noise[0] = params->ukf_flow_alt_n;
			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 1, 1, noise, navFlowUkfPresUpdate);
		}
	}
}

void navFlowUkfFlowPosUpate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {

    float y[2];
    float noise[2];
	static float zeroPositionX = 0.0f;
	static float zeroPositionY = 0.0f;

	if(!UKF_FLOW_CALCULATES_POSITION){
		return;
	}
	if (control_mode->flag_armed) {
		if(bottom_flow->ned_xy_valid > 200u){
			y[0] = bottom_flow->ned_x - zeroPositionX;
			y[1] = bottom_flow->ned_y - zeroPositionY;
			noise[0] = params->ukf_flow_vel_n +
					(params->ukf_flow_vel_max_n-params->ukf_flow_vel_n) * (1.0f - (float)bottom_flow->ned_xy_valid/255.0f);
			noise[1] = noise[0];
			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfPositionUpdate);
		}
		else {
			zeroPositionX = bottom_flow->ned_x - UKF_FLOW_POSX;
			zeroPositionY = bottom_flow->ned_y - UKF_FLOW_POSY;
			y[0] = UKF_FLOW_POSX;
			y[1] = UKF_FLOW_POSY;
			noise[0] = params->ukf_flow_vel_max_n;
			noise[1] = noise[0];
			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfPositionUpdate);
		}
	} else {
		y[0] = 0.0f;
		y[1] = 0.0f;
		noise[0] = 1e-7f;
		noise[1] = noise[0];
		srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfPositionUpdate);
		zeroPositionX = bottom_flow->ned_x - UKF_FLOW_POSX;
		zeroPositionY = bottom_flow->ned_y - UKF_FLOW_POSY;
	}
}

void navFlowUkfFlowVelUpate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {
    float y[2];
    float noise[2];

	noise[0] = params->ukf_flow_vel_max_n;
	noise[1] = params->ukf_flow_vel_max_n;

    if(bottom_flow->ned_v_xy_valid > 0) {
    	// velocity in earth frame
        y[0] = bottom_flow->ned_vx;
        y[1] = bottom_flow->ned_vy;
    	noise[0] = params->ukf_flow_vel_n +
    			(params->ukf_flow_vel_max_n-params->ukf_flow_vel_n) * (1.0f - (float)bottom_flow->ned_v_xy_valid/255.0f);
    	noise[1] = noise[0];
    } else {
    	y[0] = 0.0f;
    	y[1] = 0.0f;
    }

    if(!control_mode->flag_armed) {
    	noise[0] = 1e-5f;
    	noise[1] = noise[0];
    }
	srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 2, 2, noise, navFlowUkfVelocityUpdate);
}

void navFlowUkfFlowUpdate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const float dt,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {
    float y[4];
    float noise[4];
    float posDelta[3];
    float velDelta[3];
    posDelta[0] = 0.0f;
    posDelta[1] = 0.0f;
    posDelta[2] = 0.0f;
    velDelta[0] = 0.0f;
    velDelta[1] = 0.0f;
    velDelta[2] = 0.0f;
	static float zeroPositionX = 0.0f;
	static float zeroPositionY = 0.0f;
	if(!UKF_FLOW_CALCULATES_POSITION){
		return;
	}/*
	if(bottom_flow->ned_xy_valid > 0 && control_mode->flag_armed) {
		// determine how far back this flow position update came from
		int histIndexPos = (hrt_absolute_time() - (bottom_flow->timestamp + params->ukf_flow_pos_delay)) / (int)(1e6f * dt);
		histIndexPos = navFlowUkfData.navHistIndex - histIndexPos;
		if (histIndexPos < 0) histIndexPos += UKF_HIST;
		if (histIndexPos < 0 || histIndexPos >= UKF_HIST) histIndexPos = 0;
		int histIndexVel = (hrt_absolute_time() - (bottom_flow->timestamp + params->ukf_flow_vel_delay)) / (int)(1e6f * dt);
		histIndexVel = navFlowUkfData.navHistIndex - histIndexVel;
		if (histIndexVel < 0) histIndexVel += UKF_HIST;
		if (histIndexVel < 0 || histIndexVel >= UKF_HIST) histIndexVel = 0;

		// calculate delta from current position
		posDelta[0] = UKF_FLOW_POSX - navFlowUkfData.posX[histIndexPos];
		posDelta[1] = UKF_FLOW_POSY - navFlowUkfData.posY[histIndexPos];
		posDelta[2] = UKF_FLOW_POSD - navFlowUkfData.posD[histIndexPos];
		// calculate delta from current velocity
		velDelta[0] = UKF_FLOW_VELX - navFlowUkfData.velX[histIndexVel];
		velDelta[1] = UKF_FLOW_VELY - navFlowUkfData.velY[histIndexVel];
		velDelta[2] = UKF_FLOW_VELD - navFlowUkfData.velD[histIndexVel];

		// set current position state to historic data
		UKF_FLOW_POSX = navFlowUkfData.posX[histIndexPos];
		UKF_FLOW_POSY = navFlowUkfData.posY[histIndexPos];
		UKF_FLOW_POSD = navFlowUkfData.posD[histIndexPos];
		// set current velocity state to historic data
		UKF_FLOW_VELX = navFlowUkfData.velX[histIndexVel];
		UKF_FLOW_VELY = navFlowUkfData.velY[histIndexVel];
		UKF_FLOW_VELD = navFlowUkfData.velD[histIndexVel];
	}*/

	if (control_mode->flag_armed) {
		y[0] = bottom_flow->ned_vx;
		y[1] = bottom_flow->ned_vy;
		y[2] = bottom_flow->ned_x - zeroPositionX;
		y[3] = bottom_flow->ned_y - zeroPositionY;
		static uint8_t printcounter = 0;
		if (!((printcounter++) % 10)) {
						printf("va: %d ned vx: %8.4f vy: %8.4f\n",bottom_flow->ned_v_xy_valid,bottom_flow->ned_vx,bottom_flow->ned_vy);
		}
	    if(bottom_flow->ned_v_xy_valid > 100) {
	    	// velocity in earth frame
	    	noise[0] = params->ukf_flow_vel_n +
	    			(params->ukf_flow_vel_max_n-params->ukf_flow_vel_n) * (1.0f - (float)bottom_flow->ned_v_xy_valid/255.0f);
	    	noise[1] = noise[0];
			noise[2] = noise[0];
			noise[3] = noise[0];
	    }
	    else {
	    	y[0] = 0.0f;
	    	y[1] = 0.0f;
			y[2] = UKF_FLOW_POSX;
			y[3] = UKF_FLOW_POSY;
			noise[0] = params->ukf_flow_vel_max_n;
			noise[1] = noise[0];
			noise[2] = noise[0];
			noise[3] = noise[0];
			zeroPositionX = bottom_flow->ned_x - UKF_FLOW_POSX;
			zeroPositionY = bottom_flow->ned_y - UKF_FLOW_POSY;
			// TODO: Don't do updates here if GPS is valid !?
	    }
	} else {
        y[0] = 0.0f;
        y[1] = 0.0f;
		y[2] = 0.0f;
		y[3] = 0.0f;
    	noise[0] = 1e-2f;
    	noise[1] = noise[0];
		noise[2] = 1e-2f;
		noise[3] = noise[2];
		zeroPositionX = bottom_flow->ned_x - UKF_FLOW_POSX;
		zeroPositionY = bottom_flow->ned_y - UKF_FLOW_POSY;
	}
	srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 4, 4, noise, navFlowUkfVelocityPositionUpdate);
    // add the historic  delta back to the current state
	/*UKF_FLOW_POSX += posDelta[0];
	UKF_FLOW_POSY += posDelta[1];
	UKF_FLOW_POSD += posDelta[2];
	UKF_FLOW_VELX += velDelta[0];
	UKF_FLOW_VELY += velDelta[1];
	UKF_FLOW_VELD += velDelta[2];*/
}

void navFlowUkfGpsPosUpate(
		const struct vehicle_gps_position_s* gps_position,
		struct vehicle_local_position_s* local_position_data,
		float dt,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {
    float y[3];
    float noise[3];
    float posDelta[3];
    static bool positionInitialized = false;
    int histIndex;
    if (dt < FLT_MIN) return;
    if (gps_position->eph_m >= 4.0f || fabsf(gps_position->tDop) <= FLT_MIN ||
    	(control_mode->flag_armed && !positionInitialized)) {
    	// no or not good enough signal quality
    	// or already armed and not yet initialized
    	return;
    	/*
		y[0] = 0.0f;
		y[1] = 0.0f;
		y[2] = UKF_FLOW_PRES_ALT;

		// TODO: This was isFlying
		if (navFlowIsArmed(control_mode)) {
			noise[0] = 1e1f;
			noise[1] = 1e1f;
			noise[2] = 1e2f;
		}
		else {
			noise[0] = 1e-7f;
			noise[1] = 1e-7f;
			noise[2] = 1e2f;
		}
		srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfPosUpdate);
		*/
    }
    else {
    	//convert to local position

		if (! positionInitialized) {
			positionInitialized = true;
			navFlowUkfData.holdLat = gps_position->lat;
			navFlowUkfData.holdLon = gps_position->lon;
			navFlowUkfCalcEarthRadius(gps_position->lat);
			//navFlowUkfSetGlobalPositionTarget(gps_position->lat, gps_position->lon);
			//navFlowUkfResetPosition(-UKF_FLOW_POSX, -UKF_FLOW_POSY, gps_position->alt - UKF_FLOW_POSD);
			//reinit local position reference
			local_position_data->ref_lat = gps_position->lat;
			local_position_data->ref_lon = gps_position->lon;
			//reinit map projection
			double lat_home = (float)local_position_data->ref_lat * 1e-7f;
			double lon_home = (float)local_position_data->ref_lon * 1e-7f;
			map_projection_init(lat_home, lon_home);
		}
		else {
			map_projection_project(gps_position->lat,gps_position->lon,&y[0], &y[1]);
			//navFlowUkfCalcDistance(gps_position->lat, gps_position->lon, &y[0], &y[1]);
			y[2] = gps_position->alt;

			// determine how far back this GPS position update came from
			histIndex = (hrt_absolute_time() - (gps_position->timestamp_position + params->ukf_pos_delay)) / (int)(1e6f * dt);
			histIndex = navFlowUkfData.navHistIndex - histIndex;
			if (histIndex < 0)
			histIndex += UKF_HIST;
			if (histIndex < 0 || histIndex >= UKF_HIST)
			histIndex = 0;

			// calculate delta from current position
			posDelta[0] = UKF_FLOW_POSX - navFlowUkfData.posX[histIndex];
			posDelta[1] = UKF_FLOW_POSY - navFlowUkfData.posY[histIndex];
			posDelta[2] = UKF_FLOW_POSD - navFlowUkfData.posD[histIndex];

			// set current position state to historic data
			UKF_FLOW_POSX = navFlowUkfData.posX[histIndex];
			UKF_FLOW_POSY = navFlowUkfData.posY[histIndex];
			UKF_FLOW_POSD = navFlowUkfData.posD[histIndex];

			noise[0] = params->ukf_gps_pos_n + gps_position->eph_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->nDop*gps_position->nDop) * params->ukf_gps_pos_m_n;
			noise[1] = params->ukf_gps_pos_n + gps_position->eph_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->eDop*gps_position->eDop) * params->ukf_gps_pos_m_n;
			noise[2] = params->ukf_gps_alt_n + gps_position->epv_m * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->vDop*gps_position->vDop) * params->ukf_gps_alt_m_n;

			srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfPosAltUpdate);

			// add the historic position delta back to the current state
			UKF_FLOW_POSX += posDelta[0];
			UKF_FLOW_POSY += posDelta[1];
			UKF_FLOW_POSD += posDelta[2];
		}
    }
}

void navFlowUkfGpsVelUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params) {
	// Don't do anything for invalid dt
	if(dt < FLT_MIN) return;
    float y[3];
    float noise[3];
    float velDelta[3];
    int histIndex;
    if (gps_position->sAcc >= 2.0f || fabsf(gps_position->tDop) <= FLT_MIN) {
    	// no or not good enough signal quality
    	return;
    }
    else {
		y[0] = gps_position->vel_n_m_s;
		y[1] = gps_position->vel_e_m_s;
		y[2] = gps_position->vel_d_m_s;

		// determine how far back this GPS velocity update came from
		histIndex = (hrt_absolute_time() - (gps_position->timestamp_velocity + params->ukf_vel_delay)) / (int)(1e6f * dt);
		histIndex = navFlowUkfData.navHistIndex - histIndex;
		if (histIndex < 0)
			histIndex += UKF_HIST;
		if (histIndex < 0 || histIndex >= UKF_HIST)
			histIndex = 0;

		// calculate delta from current position
		velDelta[0] = UKF_FLOW_VELX - navFlowUkfData.velX[histIndex];
		velDelta[1] = UKF_FLOW_VELY - navFlowUkfData.velY[histIndex];
		velDelta[2] = UKF_FLOW_VELD - navFlowUkfData.velD[histIndex];

		// set current position state to historic data
		UKF_FLOW_VELX = navFlowUkfData.velX[histIndex];
		UKF_FLOW_VELY = navFlowUkfData.velY[histIndex];
		UKF_FLOW_VELD = navFlowUkfData.velD[histIndex];

		noise[0] = params->ukf_gps_vel_n + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->nDop*gps_position->nDop) * params->ukf_gps_vel_m_n;
		noise[1] = params->ukf_gps_vel_n + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->eDop*gps_position->eDop) * params->ukf_gps_vel_m_n;
		noise[2] = params->ukf_gps_vd_n  + gps_position->sAcc * aq_sqrtf(gps_position->tDop*gps_position->tDop + gps_position->vDop*gps_position->vDop) * params->ukf_gps_vd_m_n;

		srcdkfMeasurementUpdate(navFlowUkfData.kf, 0, y, 3, 3, noise, navFlowUkfVelAltUpdate);

		// add the historic position delta back to the current state
		UKF_FLOW_VELX += velDelta[0];
		UKF_FLOW_VELY += velDelta[1];
		UKF_FLOW_VELD += velDelta[2];
    }
}

void navFlowUkfInitState(const struct sensor_combined_s* sensors) {

    // vel
    UKF_FLOW_VELX = 0.0f;
    UKF_FLOW_VELY = 0.0f;
    UKF_FLOW_VELD = 0.0f;

    if(UKF_FLOW_CALCULATES_POSITION) {
        // pos
        UKF_FLOW_POSX = 0.0f;
        UKF_FLOW_POSY = 0.0f;
    }
    if(UKF_FLOW_CALCULATES_ALTITUDE) {
    	UKF_FLOW_POSD = -utilPresToAlt(sensors->baro_pres_mbar);
    }

    // acc bias
    UKF_FLOW_ACC_BIAS_X = 0.0f;
    UKF_FLOW_ACC_BIAS_Y = 0.0f;
    UKF_FLOW_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_FLOW_GYO_BIAS_X = 0.0f;
    UKF_FLOW_GYO_BIAS_Y = 0.0f;
    UKF_FLOW_GYO_BIAS_Z = 0.0f;

    // quat
    UKF_FLOW_Q1 =  1.0f;
    UKF_FLOW_Q2 =  0.0f;
    UKF_FLOW_Q3 =  0.0f;
    UKF_FLOW_Q4 =  0.0f;

    UKF_FLOW_PRES_ALT = utilPresToAlt(sensors->baro_pres_mbar);
}

void navFlowLogVariance(void) {
	float Q[navFlowUkfData.numberOfStates];
	float32_t *Sx = navFlowUkfData.kf->Sx.pData;
	for (int i = 0; i < navFlowUkfData.numberOfStates; i++) {
		Q[i] = Sx[i*navFlowUkfData.numberOfStates + i]*Sx[i*navFlowUkfData.numberOfStates + i];
	}
	printf("Q:1:%15.1f\t2:%15.1f\t3:%15.1f\t4:%15.1f\t5:%15.1f\t6:%15.1f\t7:%15.1f\t8:%15.1f\t9:%15.1f\t10:%15.1f\t11:%15.1f\t12:%15.1f\t13:%15.1f\t14:%15.1f\n",
			Q[0],Q[1],Q[2],Q[3],Q[4],Q[5],Q[6],Q[7],Q[8],Q[9],Q[10],Q[11],Q[12],Q[13]);

	float V[navFlowUkfData.numberOfProcessNoiseVariables];
	float32_t *Sv = navFlowUkfData.kf->Sv.pData;
	for (int i = 0; i < navFlowUkfData.numberOfProcessNoiseVariables; i++) {
		V[i] = Sv[i*navFlowUkfData.numberOfProcessNoiseVariables + i]*Sv[i*navFlowUkfData.numberOfProcessNoiseVariables + i];
	}
	printf("V:1:%15.1f\t2:%15.1f\t3:%15.1f\t4:%15.1f\t5:%15.1f\t6:%15.1f\t7:%15.1f\t8:%15.1f\t9:%15.1f\t10:%15.1f\t11:%15.1f\t12:%15.1f\t15:%15.1f\n",
			V[0],V[1],V[2],V[3],V[4],V[5],V[6],V[7],V[8],V[9],V[10],V[11],V[12]);
}

void navFlowUkfInit(const struct quat_position_control_UKF_params* params,
				const struct sensor_combined_s* sensors,
				const float inclination) {
    memset((void *)&navFlowUkfData, 0, sizeof(navFlowUkfData));

	int states = (int)params->ukf_states;
	navFlowUkfData.numberOfStates = states;
	navFlowUkfData.numberOfProcessNoiseVariables = states - 1;
	if(states >= 17) navFlowUkfData.calculateAltitude = true;
	else navFlowUkfData.calculateAltitude = false;
	if(states >= 15) navFlowUkfData.calculatePosition = true;
	else navFlowUkfData.calculatePosition = false;

    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];

    navFlowUkfData.v0a[0] = 0.0f;
    navFlowUkfData.v0a[1] = 0.0f;
    navFlowUkfData.v0a[2] = -1.0f;

    // calculate mag vector based on inclination
    mag[0] = cosf(inclination * DEG_TO_RAD);
    mag[1] = 0.0f;
    mag[2] = -sinf(inclination * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    navFlowUkfData.v0m[0] = mag[0] * cosf(COMPASS_DECLINATION * DEG_TO_RAD) - mag[1] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navFlowUkfData.v0m[1] = mag[1] * cosf(COMPASS_DECLINATION  * DEG_TO_RAD) + mag[0] * sinf(COMPASS_DECLINATION  * DEG_TO_RAD);
    navFlowUkfData.v0m[2] = mag[2];

    navFlowUkfData.kf = srcdkfInit(states, SIM_M, navFlowUkfData.numberOfProcessNoiseVariables, navFlowUkfData.numberOfProcessNoiseVariables - 6, SIM_N, navFlowUkfTimeUpdate);
    //navFlowUkfData.kf = srcdkfInit(17, SIM_M, 16, SIM_N, navFlowUkfTimeUpdate);

    navFlowUkfData.x = srcdkfGetState(navFlowUkfData.kf);
    navFlowUkfData.sonarAltOffset = 0.0f;
    navFlowUkfData.pressAltOffset = 0.0f;

    // State variance
    Q[0] = params->ukf_acc_bias_q;
    Q[1] = params->ukf_acc_bias_q;
    Q[2] = params->ukf_acc_bias_q;
    Q[3] = params->ukf_gyo_bias_q;
    Q[4] = params->ukf_gyo_bias_q;
    Q[5] = params->ukf_gyo_bias_q;
    Q[6] = params->ukf_vel_q;
    Q[7] = params->ukf_vel_q;
    Q[8] = params->ukf_vel_alt_q;
    Q[9] = params->ukf_quat_q;
    Q[10] = params->ukf_quat_q;
    Q[11] = params->ukf_quat_q;
    Q[12] = params->ukf_quat_q;
    Q[13] = params->ukf_pres_alt_q;
    //only for position
    Q[14] = params->ukf_pos_q;
    Q[15] = params->ukf_pos_q;
    Q[16] = params->ukf_pos_alt_q;

    // Process noise
    // This reflects the noise variables used in navFlowUkfTimeUpdate
    V[0] = params->ukf_acc_bias_v;
    V[1] = params->ukf_acc_bias_v;
    V[2] = params->ukf_acc_bias_v;
    V[3] = params->ukf_gyo_bias_v;
    V[4] = params->ukf_gyo_bias_v;
    V[5] = params->ukf_gyo_bias_v;
    V[6] = params->ukf_rate_v;
    V[7] = params->ukf_rate_v;
    V[8] = params->ukf_rate_v;
    V[9] = params->ukf_vel_v;
    V[10] = params->ukf_vel_v;
    V[11] = params->ukf_alt_vel_v;
    V[12] = params->ukf_pres_alt_v;
    //only for position
    V[13] = params->ukf_pos_v;
    V[14] = params->ukf_pos_v;
    V[15] = params->ukf_alt_pos_v;

    srcdkfSetVariance(navFlowUkfData.kf, Q, V, 0, 0);

    navFlowUkfInitState(sensors);

}
