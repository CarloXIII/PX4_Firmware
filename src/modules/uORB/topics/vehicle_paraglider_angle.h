/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file vehicle_paraglider_angel.h
 *
 * Definition of relative angle between vehicle and paraglider topic
 */

#ifndef TOPIC_VEHICLE_PARAGLIDER_ANGLE_H_
#define TOPIC_VEHICLE_PARAGLIDER_ANGLE_H_

#include "../uORB.h"
#include <stdint.h>

/**
 * @addtogroup topics
 * @{
 */

/**
 * Relativ angle.
 */
struct vehicle_paraglider_angle_s {

	uint64_t error_count;
	uint8_t valid;					/** 1 == within sensor range, 0 = outside sensor range */
	uint64_t 	timestamp_left;		/**< Timestamp in microseconds since boot         */
	float 		raw_left;			/** Raw value from max127 adc of the left potentiometer */
	float 		angle_left;     	/**< Angle between left breakline and vehicle [rad] */
	uint64_t 	timestamp_right;	/**< Timestamp in microseconds since boot */
	float 		raw_right;			/** Raw value from max127 adc of the right potentiometer */
	float 		angle_right;     	/**< Angle between right breakline and vehicle [rad] */
	uint32_t 	angle_counter; 		/**< Number of raw adc measurements taken        */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_paraglider_angle);

#endif
