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


#include "paraglider_altitude_estimator.h"

#define DT_MIN (0.0025f)	// Defines the Frequenz of the controller, should not higher than 400Hz
#define MAX_ANG_SP (0.24f)	// For maximum twist angle between paraglider and load (after this, the limit of the controller is reached)

// twist angle control parameters
PARAM_DEFINE_FLOAT(TWISTANG_P, 0.8f);
PARAM_DEFINE_FLOAT(TWISTANG_I, 3.4f);
PARAM_DEFINE_FLOAT(TWISTANG_D, 1.6f);
PARAM_DEFINE_FLOAT(TWISTANG_INT_LIM, 0.6f);
PARAM_DEFINE_FLOAT(TWISTANG_SAT, 0.65f);
PARAM_DEFINE_FLOAT(TWISTANG_THR_SP, 0.4f);

struct paraglider_altitude_estimator_params {
	float twist_angle_p;
	float twist_angle_i;
	float twist_angle_d;
	float integral_limiter;
	float saturation;
	float thrust_sp;
};

struct paraglider_altitude_estimator_param_handles {
	param_t twist_angle_p;
	param_t twist_angle_i;
	param_t twist_angle_d;
	param_t integral_limiter;
	param_t saturation;
	param_t thrust_sp;
};

/* Internal Prototypes */
static int parameters_init(struct paraglider_altitude_estimator_param_handles *h);
static int parameters_update(const struct paraglider_altitude_estimator_param_handles *h,
		struct paraglider_altitude_estimator_params *p);

static int parameters_init(struct paraglider_altitude_estimator_param_handles *h) {
	/* PID parameters */
	h->twist_angle_p = param_find("TWISTANG_P");
	h->twist_angle_i = param_find("TWISTANG_I");
	h->twist_angle_d = param_find("TWISTANG_D");
	h->integral_limiter = param_find("TWISTANG_INT_LIM");
	h->saturation = param_find("TWISTANG_SAT");
	h->thrust_sp = param_find("TWISTANG_THR_SP");
	return OK;
}

static int parameters_update(const struct paraglider_altitude_estimator_param_handles *h,
		struct paraglider_altitude_estimator_params *p) {
	param_get(h->twist_angle_p, &(p->twist_angle_p));
	param_get(h->twist_angle_i, &(p->twist_angle_i));
	param_get(h->twist_angle_d, &(p->twist_angle_d));
	param_get(h->integral_limiter, &(p->integral_limiter));
	param_get(h->saturation, &(p->saturation));
	param_get(h->thrust_sp, &(p->thrust_sp));
	return OK;
}

/*
 * Control Function:
 * Arguments:
 * 	angle_measurement: Angle measurement with the two Potentiometers left and right
 * 	manual_sp: RC-input with twist angle setpoint between paraglider and load
 *	actuators: Output to the mixers
 */
//
//int paraglider_altitude_estimator(
//		const struct xsens_sensor_combined_s *xsens_sensor,
//		struct paraglider_altitude_estimator_s *out){
//
//
//	static int counter = 0;
//
//	counter++;
//	return 0;
//}

