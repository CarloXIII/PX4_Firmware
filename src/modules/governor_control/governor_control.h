/****************************************************************************
 *
 *   Author: @author Benedikt Imbach <benedikt.imbach@hslu.ch>
 *
 ****************************************************************************/

/* @file Governor Control */

#ifndef GOVERNOR_CONTROL_H_
#define GOVERNOR_CONTROL_H_

#include <uORB/topics/vehicle_governor_setpoint.h>


int governor_control(const struct vehicle_governor_setpoint_s *gov_sp);

#endif /* FIXEDWING_ATT_CONTROL_ATT_H_ */
