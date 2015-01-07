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
 
    Copyright © 2011, 2012  Bill Nesbitt
*/

#ifndef _aq_math_h
#define _aq_math_h
#include <mathlib/CMSIS/Include/arm_math.h>
#include <systemlib/visibility.h>

#define aq_sqrtf __aq_sqrtf
__BEGIN_DECLS
__EXPORT float __aq_sqrtf(float x);
__EXPORT float __aq_cosf(float x);
__EXPORT float __aq_sinf(float x);
__EXPORT float __aq_atan2f(float y, float x);
__EXPORT float __aq_atanf(float x);
__EXPORT void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols);
__EXPORT void matrixFree(arm_matrix_instance_f32 *m);
__EXPORT void matrixDump(char *name, arm_matrix_instance_f32 *m);
__EXPORT void qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R);
__EXPORT void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ);
__END_DECLS
#endif
