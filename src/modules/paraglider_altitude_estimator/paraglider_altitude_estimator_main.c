/**
 * @file paraglider_altitude_estimatorler_main.c
 * @author Carlo Schuler, 2014
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
//#include <uORB/topics/vehicle_status.h> //not used yet
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/paraglider_altitude_estimator.h>
#include <uORB/topics/xsens_sensor_combined.h>

#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>

#include "paraglider_altitude_estimator.h"

/* Prototypes */
/**
 * Deamon management function.
 */__EXPORT int paraglider_altitude_estimator_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int paraglider_altitude_estimator_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/* Variables */
static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int deamon_task; /**< Handle of deamon task / thread */

/* Ringbuffer */
typedef struct ringbuffer{
	int length;
	int head;
	int tail;
	float *elems;
}baroBuffer_t;

static baroBuffer_t altitudeBuffer;

// RINGBUFFER FUNCTIONS
void initBuffer(baroBuffer_t *bB, int length){
	bB->length = length;
	bB->head = 0;
	bB->tail = 0;
	bB->elems = (float*)calloc(bB->length,sizeof(float));
}

void addElementToBuffer(baroBuffer_t *bB, float newElement){
	if(bB->head==bB->length){
		bB->head=0;
	}

	bB->elems[(bB->head)] = newElement;
	bB->head = bB->head + 1;
}

float getBufferValue(baroBuffer_t *bB){
	float sum = 0;
	for(int i = 0; i<(bB->length);i++){
		sum += bB->elems[i];
	}
	sum = sum/((float)(bB->length));
	return sum;
}
//

/* Main Thread */
int paraglider_altitude_estimator_thread_main(int argc, char *argv[]) {
	/* read arguments */
	bool verbose = false;		// for talking option

	initBuffer(&altitudeBuffer,100);


	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[paraglider_altitude_estimator] started\n");

	/* declare and safely initialize all structs */
	struct xsens_sensor_combined_s xsens_sensor_combined_values;
	memset(&xsens_sensor_combined_values, 0, sizeof(xsens_sensor_combined_values));

	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	//struct vehicle_status_s vstatus;
	//memset(&vstatus, 0, sizeof(vstatus));

	/* output structs*/
	struct paraglider_altitude_estimator_s paraglider_altitude_estimator_out;
	memset(&paraglider_altitude_estimator_out, 0, sizeof(paraglider_altitude_estimator_out));

	orb_advert_t paraglider_altitude_estimator_pub = orb_advertise(ORB_ID(paraglider_altitude_estimator),
			&paraglider_altitude_estimator_out);


	/* subscribe */
	int xsens_sensor_combined_sub = orb_subscribe(ORB_ID(xsens_sensor_combined));




	//int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	//int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/* Setup of loop */
	struct pollfd fds = { .fd = xsens_sensor_combined_sub, .events = POLLIN };

	while (!thread_should_exit) {
		/* wait for a sensor update, check for exit condition every 100 ms with this while-loop */
		poll(&fds, 1, 100);
		static int counter = 0;

		bool xsens_sensor_combined_updated;
		orb_check(xsens_sensor_combined_sub, &xsens_sensor_combined_updated); /* Check if there is a new relative angle measurement */

		/* get a local copy */
		if (xsens_sensor_combined_updated){
			orb_copy(ORB_ID(xsens_sensor_combined), xsens_sensor_combined_sub,
					&xsens_sensor_combined_values);
			addElementToBuffer(&altitudeBuffer,xsens_sensor_combined_values.baro_alt_meter);
		}

// .................................................................
// FUNCTION BLOCK
//..................................................................


		// Implementing Ringbuffer



		//



		paraglider_altitude_estimator_out.altitude_meter = getBufferValue(&altitudeBuffer);
		paraglider_altitude_estimator_out.timestamp = counter;
//..................................................................


		if (isfinite(paraglider_altitude_estimator_out.altitude_meter)) {
					orb_publish(ORB_ID(paraglider_altitude_estimator),paraglider_altitude_estimator_pub,&paraglider_altitude_estimator_out);
				} else {
					printf("actuator not finite");
				}

		counter++;
	} // exit: while loop

	printf("[paraglider_altitude_estimator] exiting, engine shut down.\n");
	thread_running = false;

	/* kill all outputs */


	close(xsens_sensor_combined_sub);
	//close(control_mode_sub);
	//close(vehicle_status_sub);
	fflush(stdout);
	exit(0);

	return 0;

}




/* Startup Functions */

static void usage(const char *reason) {
	if (reason)
		fprintf(stderr, "%s\n", reason);

		fprintf(stderr, "usage: paraglider_altitude_estimator {start|stop|status}\n\n");
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
int paraglider_altitude_estimator_main(int argc, char *argv[]) {
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("paraglider_altitude_estimator already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		deamon_task = task_spawn_cmd("paraglider_altitude_estimator", SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 20, 2048, paraglider_altitude_estimator_thread_main,
				(argv) ? (const char **) &argv[2] : (const char **) NULL );
		thread_running = true;
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tparaglider_altitude_estimator is running\n");

		} else {
			printf("\tparaglider_altitude_estimator not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

