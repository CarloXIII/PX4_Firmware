/*
 * @file governor_control.c
 * @author Benedikt Imbach, 2013
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <uORB/topics/actuator_controls.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/manual_control_setpoint.h>

#include "governor_control.h"
#include <drivers/drv_rpm.h>	// for rpm_report


#define DT_MIN 0.0025f	//Controller should run with maximal 400Hz
#define MAX_RPM_SP 880	//For Scout max RPM, also used to limit controller output


// RPM control parameters
PARAM_DEFINE_FLOAT(GOVERNOR_P, 1.0);
PARAM_DEFINE_FLOAT(GOVERNOR_I, 0.0f);
PARAM_DEFINE_FLOAT(GOVERNOR_D, 0.0f);
PARAM_DEFINE_FLOAT(GOVERNOR_INT_LIM, 100.0f);	// anti windup (symetrical at the moment)


struct governor_control_params {
	float governor_p;
	float governor_i;
	float governor_d;
	float integral_limiter;
};

struct governor_control_param_handles {
	param_t governor_p;
	param_t governor_i;
	param_t governor_d;
	param_t integral_limiter;
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
	h->integral_limiter =	param_find("GOVERNOR_INT_LIM");
	return OK;
}

static int parameters_update(const struct governor_control_param_handles *h, struct governor_control_params *p)
{
	param_get(h->governor_p, &(p->governor_p));
	param_get(h->governor_i, &(p->governor_i));
	param_get(h->governor_d, &(p->governor_d));
	param_get(h->integral_limiter, &(p->integral_limiter));
	return OK;
}


// Control function:
// Arguments: rpm_measurement: RPM measurement, manual_sp: RC-input with RPM setpoint, actuators: servo output to mixers
int governor_control(const struct rpm_report *rpm_measurement, const struct manual_control_setpoint_s *manual_sp,
				struct actuator_controls_s *actuators)
{
	 /*
			 *    0  -  roll   (-1..+1)
			 *    1  -  pitch  (-1..+1)
			 *    2  -  yaw    (-1..+1)
			 *    3  -  thrust ( 0..+1)
			 *    4  -  flaps  (-1..+1)
	*/
	static int counter = 0;
	static bool initialized = false;

	static struct governor_control_params p;
	static struct governor_control_param_handles h;

	static PID_t governor_controller;	// controller opject for pid.h



	if (!initialized) {
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&governor_controller, p.governor_p, p.governor_i, p.governor_d,  p.integral_limiter, MAX_RPM_SP, PID_MODE_DERIVATIV_CALC, DT_MIN); //PI Controller
		// intmax is the anti-windup value (max i-value)
		// limit is a symmetrical limiter, an asymmetric minimal limiter is not jet supported form PID lib
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		printf("param updated: p = %f, i=%f, d=%f\n", p.governor_p, p.governor_i, p.governor_d);
		pid_set_parameters(&governor_controller, p.governor_p, p.governor_i, p.governor_d, 0, MAX_RPM_SP);
	}


	// measure time for PID-controller
	static uint64_t last_run = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();


	float reference_rpm = manual_sp->throttle * MAX_RPM_SP;		//scaling of the throttle (0..1) to rpm (0...MAX_RPM_SP)
	actuators->control[3] = pid_calculate(&governor_controller, reference_rpm, rpm_measurement->rpm, 0, deltaT) / MAX_RPM_SP;	//use PID-Controller lib pid.h

		if (counter % 100 == 0) {	// debug
			printf("actuator output (throttle, CH3) = %.3f, manual setpoint = %.3f, rpm_measurement->rpm = %f.1\n",actuators->control[3], reference_rpm, rpm_measurement->rpm);
			printf("actuator output CH0 = %.3f, actuator output CH1 = %.3f, actuator output CH2 = %.3f, actuator output CH4 = %.3f\n",actuators->control[0] ,actuators->control[1], actuators->control[2], actuators->control[4]);
		}

	counter++;
	return 0;
}



