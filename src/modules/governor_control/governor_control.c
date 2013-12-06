/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Thomas Gubler <thomasgubler@student.ethz.ch>
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
 * @file fixedwing_att_control_rate.c
 * Implementation of a fixed wing attitude controller.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>


#include <uORB/topics/vehicle_governor_setpoint.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/geo/geo.h>
#include <systemlib/systemlib.h>

#include "governor_control.h"



#define DT_MIN 0.0025f	//Controller should run with maximal 400Hz



struct governor_control_params {
	float governor_p;
	float governor_i;
	float governor_d;
	float max_throttle_lim;
	float min_throttle_lim;
};

struct governor_control_param_handles {
	param_t governor_p;
	param_t governor_i;
	param_t governor_d;
	param_t max_throttle_lim;
	param_t min_throttle_lim;
};



/* Internal Prototypes */
static int parameters_init(struct governor_control_param_handles *h);
static int parameters_update(const struct governor_control_param_handles *h, struct governor_control_params *p);

static int parameters_init(struct governor_control_param_handles *h)
{
	/* PID parameters */
	h->governor_p 		=	param_find("GOVERNOR_P");
	h->governor_i 		=	param_find("GOVERNOR_I");
	h->governor_d 		=	param_find("GOVERNOR_D");
	h->max_throttle_lim =	param_find("MAX_THROTTLE_LIM");
	h->min_throttle_lim =	param_find("MIN_THROTTLE_LIM");

	return OK;
}

static int parameters_update(const struct governor_control_param_handles *h, struct governor_control_params *p)
{
	param_get(h->governor_p, &(p->governor_p));
	param_get(h->governor_i, &(p->governor_i));
	param_get(h->governor_d, &(p->governor_d));
	param_get(h->max_throttle_lim, &(p->max_throttle_lim));
	param_get(h->min_throttle_lim, &(p->min_throttle_lim));

	return OK;
}

int governor_control(const struct vehicle_governor_setpoint_s *gov_sp,
				struct actuator_controls_s *actuators)
{
	static int counter = 0;
	static bool initialized = false;

	static struct governor_control_params p;
	static struct governor_control_param_handles h;

	static PID_t governor_controller;



	if (!initialized) {
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&governor_controller, p.governor_p, p.governor_i, p.governor_d, 0, p.max_throttle_lim, PID_MODE_DERIVATIV_CALC, DT_MIN); //PI Controller
		// intmax is the anti-windup value (max i-value)
		// limit is a symmetrical limiter, an asymmetric minimal limiter is not jet supported so, min_throttle_lim will be ignored at the moment
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		pid_set_parameters(&governor_controller, p.governor_p, p.governor_i, p.governor_d, 0, p.max_throttle_lim);
	}


	static uint64_t last_run = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	/* RPM */
	actuators->control[3] = pid_calculate(&governor_controller, gov_sp->rpm, XXX, 0, deltaT);

	counter++;

	return 0;
}



