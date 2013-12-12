/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: 	@author Thomas Gubler <thomasgubler@student.ethz.ch>
 *   			@author Doug Weibel <douglas.weibel@colorado.edu>
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
 * @file fixedwing_att_control.c
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
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>

#include <uORB/topics/vehicle_governor_setpoint.h>

#include <uORB/topics/debug_key_value.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
//#include <systemlib/geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <drivers/drv_rpm.h>
#include "governor_control.h"
//#include "fixedwing_att_control_att.h"

/* Prototypes */
/**
 * Deamon management function.
 */
__EXPORT int governor_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int governor_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* Main Thread */
int governor_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[governor control] started\n");

	/* declare and safely initialize all structs */
	struct vehicle_governor_setpoint_s governor_setpoint;
	memset(&governor_setpoint, 0, sizeof(governor_setpoint));
	struct rpm_report rpm_measurement;
	memset(&rpm_measurement, 0, sizeof(rpm_measurement));

	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));

	/* output structs */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));


	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	//orb_advert_t rates_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);	//werden nicht mehr gebarucht. könnte governor setpoint sein


	/* subscribe */
		// trigers loop
	int gov_sp_sub = orb_subscribe(ORB_ID(vehicle_governor_setpoint));
	int gov_sub = orb_subscribe(ORB_ID(sensor_rpm));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* Setup of loop */

	//float gyro[3] = {0.0f, 0.0f, 0.0f};
	//float speed_body[3] = {0.0f, 0.0f, 0.0f};
	struct pollfd fds = { .fd = gov_sub, .events = POLLIN };


	while (!thread_should_exit) {
		/* wait for a sensor update, check for exit condition every 500 ms */
		poll(&fds, 1, 500);

		/* Check if there is a new position measurement or  attitude setpoint */
		//bool pos_updated;
		//orb_check(global_pos_sub, &pos_updated);
		bool gov_updated;
		orb_check(gov_sub, &gov_updated);

		/* get a local copy */


		if (gov_updated)
			orb_copy(ORB_ID(sensor_rpm), gov_sub, &rpm_measurement);

			orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
			/*
			 * The PX4 architecture provides a mixer outside of the controller.
			 * The mixer is fed with a default vector of actuator controls, representing
			 * moments applied to the vehicle frame. This vector
			 * is structured as:
			 *
			 * Control Group 0 (attitude):
			 *
			 *    0  -  roll   (-1..+1)
			 *    1  -  pitch  (-1..+1)
			 *    2  -  yaw    (-1..+1)
			 *    3  -  thrust ( 0..+1)
			 *    4  -  flaps  (-1..+1)
			 *    ...
			 */
			orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);



		if (control_mode.flag_control_manual_enabled) {		//if channel 5 is switched middle pos
			//printf("[control man enabled, start pid\n");
			if (control_mode.flag_control_attitude_enabled) {	// governor mode sets rpm
				governor_control(&rpm_measurement, &manual_sp, &actuators);	//actuators.control[3] is set here

				/* pass through other channels */
				actuators.control[0] = manual_sp.roll;
				actuators.control[1] = manual_sp.pitch;
				actuators.control[2] = manual_sp.yaw;

				//printf("[actuator 0 = %f\n", manual_sp.roll);

			} else {	// manual mode passes all channels
				/* directly pass through values */
				actuators.control[0] = manual_sp.roll;
				/* positive pitch means negative actuator -> pull up */
				actuators.control[1] = manual_sp.pitch;
				actuators.control[2] = manual_sp.yaw;
				actuators.control[3] = manual_sp.throttle;
			}
		}


		/* sanity check and publish actuator outputs */
		if (isfinite(actuators.control[0]) &&
		    isfinite(actuators.control[1]) &&
		    isfinite(actuators.control[2]) &&
		    isfinite(actuators.control[3])) {
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
		} else {
			printf("actuator not finite");
		}
	}

	printf("[governor_control] exiting, engine shut down.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);



	//close(att_sub);
	close(actuator_pub);
	//close(rates_pub);

	fflush(stdout);
	exit(0);

	return 0;

}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: governor_control {start|stop|status}\n\n");
	exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int governor_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("governor_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("governor_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 governor_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tgovernor_control is running\n");

		} else {
			printf("\tgovernor_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



