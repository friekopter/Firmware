/*
 * quat_pos_control.h
 *
 *  Created on: May 15, 2013
 *      Author: fludwig
 */

#ifndef QUAT_FLOW_POS_CONTROL_H_
#define QUAT_FLOW_POS_CONTROL_H_
#define RUN_SENSOR_HIST		10	    // number of timesteps to average observation sensors' data

typedef struct {
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    float presHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    float sumPres;
    int accHistIndex;
    int magHistIndex;
    int presHistIndex;
} runStruct_t;




#endif /* QUAT_FLOW_POS_CONTROL_H_ */
