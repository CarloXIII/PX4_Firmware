/****************************************************************************
 *
 *   Author: @author Benedikt Imbach <benedikt.imbach@hslu.ch>
 *
 ****************************************************************************/

/* @file Governor Control */

#ifndef GOVERNOR_CONTROL_H_
#define GOVERNOR_CONTROL_H_

#include <drivers/drv_rpm.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>

int governor_control(const struct rpm_report *rpm_measurement, const struct manual_control_setpoint_s *manual_sp,
				struct actuator_controls_s *actuators);

#endif /* FIXEDWING_ATT_CONTROL_ATT_H_ */
