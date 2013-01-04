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

    Copyright � 2011  Bill Nesbitt
*/
#include "pid.h"
#include "aq.h"
#include "util.h"
#include "nav.h"
#include <stdlib.h>

pidStruct_t *pidInit(float *p, float *i, float *d, float *f, float *pMax, float *iMax, float *dMax, float *oMax, int *pTrim, int *iTrim, int *dTrim, int *fTrim) {
    pidStruct_t *pid;

    pid = (pidStruct_t *)aqCalloc(1, sizeof(pidStruct_t));

    pid->pMax = pMax;
    pid->iMax = iMax;
    pid->dMax = dMax;
    pid->oMax = oMax;
    pid->pGain = p;
    pid->iGain = i;
    pid->dGain = d;
    pid->fGain = f;
    pid->pTrim = pTrim;
    pid->iTrim = iTrim;
    pid->dTrim = dTrim;
    pid->fTrim = fTrim;

    return pid;
}


float pidUpdate(pidStruct_t *pid, float setpoint, float position) {
    float error;
    float p = *pid->pGain;
    float i = *pid->iGain;
    float d = (pid->dGain) ? *pid->dGain : 0.0f;
    float f = (pid->fGain) ? *pid->fGain : 1.0f;

    if (pid->pTrim) p += (*pid->pTrim * p * 0.002f);
    if (pid->iTrim) i += (*pid->iTrim * i * 0.002f);
    if (pid->dTrim) d += (*pid->dTrim * d * 0.002f);
    if (pid->fTrim) f += (*pid->fTrim * f * 0.002f);

    error = setpoint - position;

    // calculate the proportional term
    pid->pTerm_1 = p * error;
    if (pid->pTerm_1 > *pid->pMax)
    {
    	pid->pTerm_1 = *pid->pMax;
    }
    else if (pid->pTerm_1 < -*pid->pMax)
    {
    	pid->pTerm_1 = -*pid->pMax;
    }

    // calculate the integral state with appropriate limiting
    pid->iState += error;
    pid->iTerm_1 = i * pid->iState;
    if (pid->iTerm_1 > *pid->iMax)
    {
    	pid->iTerm_1 = *pid->iMax;//set result to max
    	pid->iState -= error;//remove last error from integral
    }
    else if (pid->iTerm_1 < -*pid->iMax)
    {
    	pid->iTerm_1 = -*pid->iMax;
    	pid->iState -= error;
    }

    // derivative if we have a value not 0 set
    if (pid->dGain)
    {
    	// uncomment this line if you want the D term to ignore set point changes
    	error = -position;
    	pid->dTerm_1 = (d * f) * (error - pid->dState);
    	pid->dState += f * (error - pid->dState);
    	if (pid->dTerm_1 > *pid->dMax)
    	{
    		pid->dTerm_1 = *pid->dMax;
    	}
    	else if (pid->dTerm_1 < -*pid->dMax)
    	{
    		pid->dTerm_1 = -*pid->dMax;
    	}
    }
    else
    {
    	pid->dTerm_1 = 0.0f;
    }

    pid->pv_1 = position;
    pid->sp_1 = setpoint;
    //Construct result from proportional, integral and derivative term
    pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;
    //Constraint result in the max range
    if (pid->co_1 > *pid->oMax)
    {
    	pid->co_1 = *pid->oMax;
    }
    else if (pid->co_1 < -*pid->oMax)
    {
    	pid->co_1 = -*pid->oMax;
    }
    return pid->co_1;
}

void pidZeroIntegral(pidStruct_t *pid, float pv, float iState) {
    pid->iState = iState;
    pid->dState = pv;
    pid->sp_1 = pv;
    pid->co_1 = 0.0f;
    pid->pv_1 = pv;
    pid->pv_2 = pv;
}
