/**
 * @file twist_angle_controller_main.c
 * @author Lukas Koepfli, 2014
 * Implementation of a twist angle controller to hold a load parallel under a paraglider.
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
#include <uORB/topics/vehicle_control_mode.h>
//#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_paraglider_angle.h>
//#include <uORB/topics/debug_key_value.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include "twist_angle_control.h"

/* Prototypes */
/**
 * Deamon management function.
 */
__EXPORT int twist_angle_control_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int twist_angle_control_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false;		/**< Deamon exit flag */
static bool thread_running = false;		/**< Deamon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

/* Main Thread */
int twist_angle_control_thread_main(int argc, char *argv[])
{
	/* read arguments */
	bool verbose = false;		// for talking option

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[twist_angle_control] started\n");

	/* declare and safely initialize all structs */
	struct vehicle_paraglider_angle_s angle_measurement;
	memset(&angle_measurement, 0, sizeof(angle_measurement));
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	//struct vehicle_status_s vstatus;
	//memset(&vstatus, 0, sizeof(vstatus));


	/* output structs */
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);

	/* subscribe */
	int rel_ang_sub = orb_subscribe(ORB_ID(vehicle_paraglider_angle));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	//int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));


	/* Setup of loop */
	struct pollfd fds = { .fd = rel_ang_sub, .events = POLLIN };


	while (!thread_should_exit) {
		/* wait for a sensor update, check for exit condition every 500 ms with this while-loop */
		poll(&fds, 1, 500);
		static int counter = 0;



		bool rel_ang_updated;
		orb_check(rel_ang_sub, &rel_ang_updated); /* Check if there is a new relative angle measurement */

		/* get a local copy */
		if (rel_ang_updated)
			orb_copy(ORB_ID(vehicle_paraglider_angle), rel_ang_sub, &angle_measurement);
			orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);	// Also update the setpoint from the radio control
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
			orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);	/* update the flags for operating mode */
			//orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &vstatus);	/* update vehicle status flags */


		if (control_mode.flag_control_manual_enabled) {		// todo not jet clear

			if (control_mode.flag_control_attitude_enabled) {	// todo governor mode sets rpm

				twist_angle_control(&angle_measurement, &manual_sp, &actuators);	//actuators.control[2] is set here

				/* pass through other channels */
				actuators.control[0] = manual_sp.roll;
				actuators.control[1] = manual_sp.pitch;
				actuators.control[3] = manual_sp.throttle;

				/*todo*/
				if (counter % 1000 == 0) {	// debug
					printf("Regler\n");
					printf("actuator output CH0 = %.3f\n",actuators.control[0]);
					printf("actuator output CH1 = %.3f\n",actuators.control[1]);
					printf("actuator output CH2 = %.3f\n",actuators.control[2]);
					printf("actuator output CH3 = %.3f\n",actuators.control[3]);
					printf("manual setpoint roll = %.3f\n",manual_sp.roll);
					printf("manual setpoint pitch = %.3f\n",manual_sp.pitch);
					printf("manual setpoint yaw = %.3f\n",manual_sp.yaw);
					printf("manual setpoint throttle = %.3f\n",manual_sp.throttle);
				}
				/*end todo*/

			} else {											// todo manual mode passes all channels
				/* directly pass through values */
				actuators.control[0] = manual_sp.roll;
				/* positive pitch means negative actuator -> pull up */
				actuators.control[1] = manual_sp.pitch;
				actuators.control[2] = manual_sp.yaw;
				actuators.control[3] = manual_sp.throttle;

				/*todo*/
				if (counter % 1000 == 0) {	// debug
					printf("Manual\n");
					printf("actuator output CH0 = %.3f\n",actuators.control[0]);
					printf("actuator output CH1 = %.3f\n",actuators.control[1]);
					printf("actuator output CH2 = %.3f\n",actuators.control[2]);
					printf("actuator output CH3 = %.3f\n",actuators.control[3]);
					printf("manual setpoint roll = %.3f\n",manual_sp.roll);
					printf("manual setpoint pitch = %.3f\n",manual_sp.pitch);
					printf("manual setpoint yaw = %.3f\n",manual_sp.yaw);
					printf("manual setpoint throttle = %.3f\n",manual_sp.throttle);
				}
				/*end todo*/

			} // exit: control_mode.flag_control_attitude_enabled
		} //exit: control_mode.flag_control_manual_enabled


		/* sanity check and publish actuator outputs */
		if (isfinite(actuators.control[0]) &&
		    isfinite(actuators.control[1]) &&
		    isfinite(actuators.control[2]) &&
		    isfinite(actuators.control[3])) {
			orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);
		} else {
			printf("actuator not finite");
		}
		counter++;
	} // exit: while loop

	printf("[twist_angle_control] exiting, engine shut down.\n");
	thread_running = false;

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

	close(rel_ang_sub);
	close(actuator_pub);
	close(manual_sp_sub);
	close(control_mode_sub);
	//close(vehicle_status_sub);
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

	fprintf(stderr, "usage: twist_angle_control {start|stop|status}\n\n");
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
int twist_angle_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("twist_angle_control already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("twist_angle_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 20,
					 2048,
					 twist_angle_control_thread_main,
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
			printf("\ttwist_angle_control is running\n");

		} else {
			printf("\ttwist_angle_control not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}



