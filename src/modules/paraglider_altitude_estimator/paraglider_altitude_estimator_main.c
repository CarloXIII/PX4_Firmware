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
#include <uORB/topics/sensor_combined.h>

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
	//free(bB->elems);
	bB->length = (int)(length);
	bB->head = 0;
	bB->tail = 0;
	bB->elems = (float*)calloc(bB->length,sizeof(float));
}


float getBufferValue(baroBuffer_t *bB){
	float sum = 0;
	for(int i = 0; i<(bB->length);i++){
		sum += bB->elems[i];
	}
	sum = sum/((float)(bB->length));
	return sum;
}


void addElementToBuffer(baroBuffer_t *bB, float newElement){
	if(bB->head>=bB->length){
		bB->head=0;
	}

	bB->elems[(bB->head)] = newElement;
	bB->head = bB->head + 1;
}

// twist angle control parameters
PARAM_DEFINE_INT32(PALT_BUFFER_LENGTH, 20);
PARAM_DEFINE_FLOAT(PALT_INITIAL_ALTITUDE, 0.0f);
PARAM_DEFINE_INT32(PALT_SKIP_VALUES,3);
PARAM_DEFINE_FLOAT(PALT_LOWPASS_FG, 3.0f);


struct paraglider_altitude_estimator_params {
	int palt_buffer_length;
	float palt_initial_altitude;
	int palt_skip_values;
	float palt_lowpass_fg;
};

struct paraglider_altitude_estimator_param_handles {
	param_t palt_buffer_length;
	param_t palt_initial_altitude;
	param_t palt_skip_values;
	param_t palt_lowpass_fg;

};

/* Internal Prototypes */
static int parameters_init(struct paraglider_altitude_estimator_param_handles *h);
static int parameters_update(const struct paraglider_altitude_estimator_param_handles *h,
		struct paraglider_altitude_estimator_params *p);

static int parameters_init(struct paraglider_altitude_estimator_param_handles *h) {
	/* PID parameters */
	h->palt_buffer_length = param_find("PALT_BUFFER_LENGTH");
	h->palt_initial_altitude = param_find("PALT_INITIAL_ALTITUDE");
	h->palt_skip_values = param_find("PALT_SKIP_VALUES");
	h->palt_lowpass_fg = param_find("PALT_LOWPASS_FG");

	return OK;
}

static int parameters_update(const struct paraglider_altitude_estimator_param_handles *h,
		struct paraglider_altitude_estimator_params *p) {
	param_get(h->palt_buffer_length, &(p->palt_buffer_length));
	param_get(h->palt_initial_altitude, &(p->palt_initial_altitude));
	param_get(h->palt_skip_values, &(p->palt_skip_values));
	param_get(h->palt_lowpass_fg, &(p->palt_lowpass_fg));
	return OK;
}

//

/* Main Thread */
int paraglider_altitude_estimator_thread_main(int argc, char *argv[]) {
	/* read arguments */
	bool verbose = false;		// for talking option
	bool AltitudeInitialised = false;
	float altitude = 0;

	static float output = 360;
	static float oldOutput = 0;
	uint64_t oldTimestamp = 0;

	float baroOffset = 0;
	float startAltitude = 0;

	static struct paraglider_altitude_estimator_params p;
	static struct paraglider_altitude_estimator_param_handles h;
	int bufferLengthLocal = 50;

	initBuffer(&altitudeBuffer,bufferLengthLocal);

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
	struct sensor_combined_s sensor_combined_values;
	memset(&sensor_combined_values, 0, sizeof(sensor_combined_values));


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
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));





	/* Setup of loop */
	struct pollfd fds = { .fd = xsens_sensor_combined_sub, .events = POLLIN };

	while (!thread_should_exit) {
		/* wait for a sensor update, check for exit condition every 100 ms with this while-loop */
		poll(&fds, 1, 10);
		static int counter = 0;

		/* load new parameters with lower rate */
		if (counter % 100 == 0) {
			/* update parameters from storage */
			parameters_update(&h, &p);
//			if(bufferLengthLocal != p.palt_buffer_length){
//				//free(altitudeBuffer.elems);
//				bufferLengthLocal = p.palt_buffer_length;
//				initBuffer(&altitudeBuffer,bufferLengthLocal);
//			}
			if(startAltitude != (p.palt_initial_altitude)){
				startAltitude = p.palt_initial_altitude;
				AltitudeInitialised = false;
			}
		}
// .................................................................
// FUNCTION BLOCK
//..................................................................

		bool xsens_sensor_combined_updated;
		bool sensor_combined_updated;
		orb_check(xsens_sensor_combined_sub, &xsens_sensor_combined_updated); /* Check if there is a new relative angle measurement */
		//orb_check(sensor_combined_sub, &sensor_combined_updated);

		/* get a local copy */
		if (xsens_sensor_combined_updated){
			orb_copy(ORB_ID(xsens_sensor_combined), xsens_sensor_combined_sub,
					&xsens_sensor_combined_values);
			addElementToBuffer(&altitudeBuffer,xsens_sensor_combined_values.baro_alt_meter);
//		}
//
//		if (sensor_combined_updated){
//			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub,
//					&sensor_combined_values);
//			addElementToBuffer(&altitudeBuffer,sensor_combined_values.baro_alt_meter);
//			if(!AltitudeInitialised && (counter>100)){
//				baroOffset = (getBufferValue(&altitudeBuffer))-(p.palt_initial_altitude);
//				AltitudeInitialised = true;
//				printf("Altitude Initialised (Paraglider_Altitude_Estimator");
//			}
			/////////////////////////////////////////////////////
			for(int n=0; n<altitudeBuffer.length; n++){      // Array sortieren
					for(int i=n; i<altitudeBuffer.length; i++){
						if(altitudeBuffer.elems[i]>altitudeBuffer.elems[i+1]){
							float tempAlt = altitudeBuffer.elems[i];
							altitudeBuffer.elems[i]=altitudeBuffer.elems[i+1];
							altitudeBuffer.elems[i+1]=tempAlt;
						}
					}
			}

			altitude=0;

			for(int i=(5); i<altitudeBuffer.length-(5);i++){

				altitude = altitude + altitudeBuffer.elems[i];
			}
			altitude = altitude/(float)(altitudeBuffer.length-((5)*2));

			// LOWPASS
			float dt = 0.1;
			float a = dt/((1/(2.0f*3.14f*(1.0f)))+dt);
			oldTimestamp = xsens_sensor_combined_values.timestamp;

			output = oldOutput + a*(altitude-oldOutput);
			oldOutput = output;

			//altitude_median = getBufferValue(&altitudeBuffer); //altitudeBuffer.elems[(altitudeBuffer.length)/2];


			///////////////////////////////////////////////////


		}

		paraglider_altitude_estimator_out.altitude_meter = output; //altitude;
		paraglider_altitude_estimator_out.timestamp = xsens_sensor_combined_values.timestamp;



		if (isfinite(paraglider_altitude_estimator_out.altitude_meter)) {
					orb_publish(ORB_ID(paraglider_altitude_estimator),paraglider_altitude_estimator_pub,&paraglider_altitude_estimator_out);
				} else {
					printf("actuator not finite");
				}
//..................................................................
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

