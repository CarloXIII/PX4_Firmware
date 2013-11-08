/**
 * @file current_sensor.cpp
 * @author Benedikt Imbach

 *
 *Current sensor driver based on the FHS 40-PSP600 current sensor
 *in combination with the ADC AD7997 with I2C interface

 */

#ifndef _DRV_CURRENT_SENSOR_H
#define _DRV_CURRENT_SENSOR_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define CURRENT_SENSOR_DEVICE_PATH	"/dev/current_sensor"


/**
 * current sensor report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct current_sensor_report {
	uint64_t timestamp;
	uint64_t error_count;
	float value; 			/** in meters */
};

/*
 * ObjDev tag for raw range finder data.
 */
//ORB_DECLARE(sensor_range_finder);

/*
 * ioctl() definitions
 *
 * Current sensor drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */

#define _CURRENTSENSORIOCBASE		(0x7900)	//random choice
#define __CURRENTSENSORIOC(_n)		(_IOC(_CURRENTSENSORIOCBASE, _n))

#define CURRENTSENSORIOCSSCALE		_CURRENTSENSORIOC(0)
#define CURRENTSENSORIOCGSCALE		_CURRENTSENSORIOC(1)


#endif /* _DRV_AIRSPEED_H */
