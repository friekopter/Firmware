/*
 * quat_constants.h
 *
 *  Created on: Mar 1, 2013
 *      Author: fludwig
 */

#ifndef QUAT_CONSTANTS_H_
#define QUAT_CONSTANTS_H_

#ifndef M_PI
#define M_PI			3.14159265f
#endif
#define COMPASS_DECLINATION		2.36888889f//East in degrees//2.2666f//Declination +Ost-West   -10.11f	// local magnetic compass correction to true north [Cove]
#define COMPASS_INCLINATION		-64.132470167481f
#define RAD_TO_DEG		(180.0f/M_PI)
#define DEG_TO_RAD		(M_PI/180.0f)

#endif /* QUAT_CONSTANTS_H_ */
