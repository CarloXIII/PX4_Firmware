/****************************************************************************
 *
 *   Author: @author Benedikt Imbach <benedikt.imbach@hslu.ch>
 *
 ****************************************************************************/

/**
 * @file vehicle_governor_setpoint.h
 * Definition of the vehicle governor setpoint topic
 */

#ifndef TOPIC_VEHICLE_GOVERNOR_SETPOINT_H_
#define TOPIC_VEHICLE_GOVERNOR_SETPOINT_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */
struct vehicle_governor_setpoint_s
{
	uint64_t timestamp; /**< in microseconds since system start */

	float rpm;	/**< RPM		*/

}; /**< vehicle_governor_setpoint_s */

 /**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_governor_setpoint);

#endif
