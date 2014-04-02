/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
 *           @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file vehicle_paraglider_angle.h
 * Definition of the relative angle between vehicle and paraglider uORB topic.
 */

#ifndef VEHICLE_PARAGLIDER_ANGLE_H_
#define VEHICLE_PARAGLIDER_ANGLE_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * Maximum number of used ADC Channels
 */
#define MAX127_USED_CHANNELS	2

/**
 * @addtogroup topics
 * @{
 */

/**
 * vehicle paraglider angle structure. Reads from the device must be in multiples of this
 * structure.
 */
struct vehicle_paraglider_angle_s {
	uint64_t timestamp;								/* Timestamp for each channel */
	uint64_t error_count[MAX127_USED_CHANNELS];		/* Error_count for each channel */
	uint16_t value[MAX127_USED_CHANNELS];			/* Raw Value for each channel */
	float si_units[MAX127_USED_CHANNELS];			/* si_units for each channel */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_paraglider_angle);

#endif
