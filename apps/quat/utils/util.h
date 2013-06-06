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
#ifndef _util_h
#define _util_h

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <systemlib/visibility.h>
#include <uORB/topics/sensor_combined.h>
__BEGIN_DECLS

#define UTIL_CCM_HEAP_SIZE	(0x2000)	//  32KB
// first order filter
typedef struct {
    float tc;
    float z1;
} utilFilter_t;

__EXPORT void *aqCalloc(size_t count, size_t size);
__EXPORT void *aqDataCalloc(uint16_t count, uint16_t size);
__EXPORT int constrainInt(int i, int lo, int hi);
__EXPORT float constrainFloat(float i, float lo, float hi);
__EXPORT void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint);
__EXPORT void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint);
__EXPORT float utilFilter(utilFilter_t *f, float signal);
__EXPORT float utilFilter3(utilFilter_t *f, float signal);
__EXPORT void utilFilterReset(utilFilter_t *f, float setpoint);
__EXPORT void utilFilterReset3(utilFilter_t *f, float setpoint);
__EXPORT void utilQuasiStatic(int n, float acc_x, float acc_y, float acc_z);

__END_DECLS

#endif //_util_h
