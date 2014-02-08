
#ifndef _nav_flow_ukf_h
#define _nav_flow_ukf_h

#include <stdbool.h>
#include <quat/utils/srcdkf.h>
#include <quat/utils/quat_pos_control_params.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <quat/utils/quat_pos_control_params.h>

#define UKF_HIST		20

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    float r1, r2;
    float posX[UKF_HIST];
    float posY[UKF_HIST];
    float posD[UKF_HIST];
    float velX[UKF_HIST];
    float velY[UKF_HIST];
    float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    //float yawCos, yawSin;
    float mat3x3[3*3];
    float *x;			// states
    float sonarAltOffset;
    float pressAltOffset;
    int numberOfStates;
    int numberOfProcessNoiseVariables;
    bool calculatePosition;
    bool calculateAltitude;
} navFlowUkfStruct_t;

extern navFlowUkfStruct_t navFlowUkfData;

#define SIM_S                   17		// states
#define SIM_M                   4		// max measurements
#define SIM_V                   16		// process noise
#define SIM_N                   4		// max observation noise

#define UKF_GYO_AVG_NUM		20

#define UKF_FLOW_ACC_BIAS_X		navFlowUkfData.x[0]
#define UKF_FLOW_ACC_BIAS_Y		navFlowUkfData.x[1]
#define UKF_FLOW_ACC_BIAS_Z		navFlowUkfData.x[2]
#define UKF_FLOW_GYO_BIAS_X		navFlowUkfData.x[3]
#define UKF_FLOW_GYO_BIAS_Y		navFlowUkfData.x[4]
#define UKF_FLOW_GYO_BIAS_Z		navFlowUkfData.x[5]
#define UKF_FLOW_VELX		navFlowUkfData.x[6] //NED frame
#define UKF_FLOW_VELY		navFlowUkfData.x[7] //NED frame
#define UKF_FLOW_VELD		navFlowUkfData.x[8] //NED frame
#define UKF_FLOW_Q1			navFlowUkfData.x[9]
#define UKF_FLOW_Q2			navFlowUkfData.x[10]
#define UKF_FLOW_Q3			navFlowUkfData.x[11]
#define UKF_FLOW_Q4			navFlowUkfData.x[12]
#define UKF_FLOW_PRES_ALT		navFlowUkfData.x[13]
#define UKF_FLOW_POSX		navFlowUkfData.x[14] //NED frame
#define UKF_FLOW_POSY		navFlowUkfData.x[15] //NED frame
#define UKF_FLOW_POSD		navFlowUkfData.x[16] //NED frame
#define UKF_FLOW_CALCULATES_POSITION navFlowUkfData.calculatePosition
#define UKF_FLOW_CALCULATES_ALTITUDE navFlowUkfData.calculateAltitude

#define UKF_FLOW_P0			101325.0f		    // standard static pressure at sea level


bool navFlowIsArmed(const struct vehicle_control_mode_s *control_mode);
void navFlowLogVariance(void);
extern void navFlowUkfInit(	const struct quat_position_control_UKF_params* params,
						const struct sensor_combined_s* sensors);
extern float navFlowUkfInertialUpdate(const struct sensor_combined_s* raw, bool updateBias);
extern void navFlowDoPresUpdate(float pres,
		 const struct vehicle_control_mode_s *control_mode,
		 const struct quat_position_control_UKF_params* params);
extern float navFlowDoAccUpdate(float accX, float accY, float accZ,
		 const struct vehicle_control_mode_s *control_mode,
		 const struct quat_position_control_UKF_params* params);
extern void navFlowDoMagUpdate(float magX, float magY, float magZ,
		 const struct vehicle_control_mode_s *control_mode,
		 const struct quat_position_control_UKF_params* params);
extern void navFlowUkfSonarUpdate(
		const struct filtered_bottom_flow_s* bottom_flow,
		float baroAltitude,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfFlowPosUpate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfFlowVelUpate(
		const struct filtered_bottom_flow_s* local_position,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfFlowUpate(
		const struct filtered_bottom_flow_s* bottom_flow,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfGpsPosUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfGpsVelUpate(
		const struct vehicle_gps_position_s* gps_position,
		float dt,
		const struct vehicle_control_mode_s *control_mode,
		const struct quat_position_control_UKF_params* params);
extern void navFlowUkfSetGlobalPositionTarget(double lat, double lon);
extern void navFlowUkfZeroRate(float zRate, int axis);
extern void navFlowUkfFinish(void);
extern void navFlowUkfSetSonarOffset(const float sonarDistanceToEarth, const float altitude,  const float kSonarBaro);
extern void navFlowUkfSetPressAltOffset(const float baroAltitude, const float altitude,  const float kSonarBaro);

#endif
