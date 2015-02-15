/*
 * quat_pos_control.h
 *
 *  Created on: May 15, 2013
 *      Author: fludwig
 */

#ifndef QUAT_POS_CONTROL_H_
#define QUAT_POS_CONTROL_H_

#define RUN_SENSOR_HIST		10	    // number of timesteps to average observation sensors' data
#define RUN_ACC_MASK		1.0f//10.0f	    // allow GPS accuracy to ramp up after startup

typedef struct {
    float accMask;
    float bestHacc;
    float accHist[3][RUN_SENSOR_HIST];
    float magHist[3][RUN_SENSOR_HIST];
    float presHist[RUN_SENSOR_HIST];
    //float rangeHist[RUN_SENSOR_HIST];
    float sumAcc[3];
    float sumMag[3];
    float sumPres;
    //float sumRange;
    int accHistIndex;
    int magHistIndex;
    int presHistIndex;
    //int rangeHistIndex;
} runStruct_t;

extern runStruct_t runData;


#endif /* QUAT_POS_CONTROL_H_ */
