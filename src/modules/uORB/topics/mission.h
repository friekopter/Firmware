/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mission.h
 * Definition of a mission consisting of mission items.
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef TOPIC_MISSION_H_
#define TOPIC_MISSION_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

#define NUM_MISSIONS_SUPPORTED 256
#define MAX_NUM_MISSION_STORAGE_PLACES 4


/**
 * @addtogroup topics
 * @{
 */


/**
 * This topic used to notify navigator about mission changes, mission itself and new mission state
 * must be stored in dataman before publication.
 */
struct mission_s {
	int dataman_id; 										/**< active storage slot. Default 0,  There are NUM_MISSION_STORAGE_PLACES offboard storage places in the dataman, beginning with index 0.*/
	unsigned count_formission[MAX_NUM_MISSION_STORAGE_PLACES]; 	/**< count of mission items in storage slot n currently stored in the dataman */
	int current_seq; 										/**< current mission item index. Default -1, start at the one changed latest */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(offboard_mission);
ORB_DECLARE(onboard_mission);

#endif
