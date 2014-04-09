/*
 * @file twist_angle_control.c
 * @author Lukas Koepfli, 2014
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
#include <systemlib/systemlib.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_paraglider_angle.h> /* use it for "vehicle_paraglider_angle_s" */

#include "twist_angle_control.h"

#define DT_MIN (0.0025f)	// Defines the Frequenz of the controller, should not higher than 400Hz
#define MAX_ANG_SP (0.24f)	// For maximum twist angle between paraglider and load (after this, the limit of the controller is reached)

// low pass filter
#define F_CUT (3.0f)
float tau = (1/(2*M_PI*F_CUT));

// twist angle control parameters
PARAM_DEFINE_FLOAT(TWISTANG_P, 21.0f);
PARAM_DEFINE_FLOAT(TWISTANG_I, 12.1f);
PARAM_DEFINE_FLOAT(TWISTANG_D, 8.69f);
PARAM_DEFINE_FLOAT(TWISTANG_INT_LIM, 0.55f);
PARAM_DEFINE_FLOAT(TWISTANG_SAT, 0.65f);
PARAM_DEFINE_FLOAT(TWISTANG_THR_SP, 0.4f);
PARAM_DEFINE_INT32(TWISTANG_FILT, 0);
/* Antireset Windup */

struct twist_angle_control_params {
	float twist_angle_p;
	float twist_angle_i;
	float twist_angle_d;
	float integral_limiter;
	float saturation;
	float thrust_sp;
	int filt;
};

struct twist_angle_control_param_handles {
	param_t twist_angle_p;
	param_t twist_angle_i;
	param_t twist_angle_d;
	param_t integral_limiter;
	param_t saturation;
	param_t thrust_sp;
	param_t filt;
};

/* Internal Prototypes */
static int parameters_init(struct twist_angle_control_param_handles *h);
static int parameters_update(const struct twist_angle_control_param_handles *h,
		struct twist_angle_control_params *p);

static int parameters_init(struct twist_angle_control_param_handles *h) {
	/* PID parameters */
	h->twist_angle_p = param_find("TWISTANG_P");
	h->twist_angle_i = param_find("TWISTANG_I");
	h->twist_angle_d = param_find("TWISTANG_D");
	h->integral_limiter = param_find("TWISTANG_INT_LIM");
	h->saturation = param_find("TWISTANG_SAT");
	h->thrust_sp = param_find("TWISTANG_THR_SP");
	h->filt = param_find("TWISTANG_FILT");
	return OK;
}

static int parameters_update(const struct twist_angle_control_param_handles *h,
		struct twist_angle_control_params *p) {
	param_get(h->twist_angle_p, &(p->twist_angle_p));
	param_get(h->twist_angle_i, &(p->twist_angle_i));
	param_get(h->twist_angle_d, &(p->twist_angle_d));
	param_get(h->integral_limiter, &(p->integral_limiter));
	param_get(h->saturation, &(p->saturation));
	param_get(h->thrust_sp, &(p->thrust_sp));
	param_get(h->filt, &(p->filt));
	return OK;
}

/*
 * Control Function:
 * Arguments:
 * 	angle_measurement: Angle measurement with the two Potentiometers left and right
 * 	manual_sp: RC-input with twist angle setpoint between paraglider and load
 *	actuators: Output to the mixers
 */

int twist_angle_control(
		const struct vehicle_paraglider_angle_s *angle_measurement,
		const struct manual_control_setpoint_s *manual_sp,
		struct actuator_controls_s *actuators) {
	/*
	 *    0  -  roll   (-1..+1)
	 *    1  -  pitch  (-1..+1)
	 *    2  -  yaw    (-1..+1)
	 *    3  -  thrust ( 0..+1)
	 */
	static int counter = 0;
	static bool initialized = false;

	static struct twist_angle_control_params p;
	static struct twist_angle_control_param_handles h;

	/* Controller object for pid.h */
	static PID_t twist_angle_controller;

	if (!initialized) {
		parameters_init(&h);
		parameters_update(&h, &p);
		pid_init(&twist_angle_controller, PID_MODE_DERIVATIV_CALC, DT_MIN);
		/* PID_MODE_DERIVATIV_CALC calculates discrete derivative from previous error
		 * val_dot in pid_calculate() will be ignored
		 * intmax is the anti-windup value (max i-value)
		 * limit is a symmetrical limiter
		 */
		initialized = true;
	}

	/* load new parameters with lower rate */
	if (counter % 100 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		pid_set_parameters(&twist_angle_controller, p.twist_angle_p,
				p.twist_angle_i, p.twist_angle_d, p.integral_limiter,
				p.saturation);
#if (DEBUG)  //just for debuging
		if (counter % 1000 == 0) {
			printf("param updated: p = %f, i=%f, d=%f\n", p.twist_angle_p,
					p.twist_angle_i, p.twist_angle_d);
		}

#endif
	}

	// measure time for PID-controller
	static uint64_t last_run = 0;
	float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	last_run = hrt_absolute_time();

	if (manual_sp->throttle > p.thrust_sp) {
		/* Calculate the relativ angle between the paraglider and load. Value of the Potentiometer right[rad] - Value of the Potentiometer left[rad] */
		float actual_twist_ang = ((angle_measurement->si_units[1])
				- (angle_measurement->si_units[0]));
		static float actual_twist_ang_filt_last = 0.0f;
		float alpha = tau/(tau+deltaT);
		float actual_twist_ang_filt = actual_twist_ang * (1-alpha) + actual_twist_ang_filt_last * alpha;
		actual_twist_ang_filt_last = actual_twist_ang_filt;

		/* Scaling of the yaw input (-1..1) to a reference twist angle (-MAX_ANG_SP...MAX_ANG_SP) */
		float reference_twist_ang = manual_sp->yaw * MAX_ANG_SP; /* generate a reference_twist_ang */

		if (!p.filt==1)
		{
		actuators->control[2] = pid_calculate(&twist_angle_controller,
				reference_twist_ang, actual_twist_ang, 0, deltaT); //use PID-Controller lib pid.h
		}
		else
		{
			actuators->control[2] = pid_calculate(&twist_angle_controller,
					reference_twist_ang, actual_twist_ang_filt, 0, deltaT); //use PID-Controller lib pid.h
		}

#if (DEBUG)  //just for debugging
		if (counter % 1000 == 0) {
			printf(
					"actuator_output (yaw, CH2) = %.3f, manual_setpoint = %.3f, actual_twist_angel = %.3f\n",
					actuators->control[2], reference_twist_ang,
					actual_twist_ang);
		}
#endif
	} else {
		actuators->control[2] = manual_sp->yaw;
	}

	counter++;
	return 0;
}

