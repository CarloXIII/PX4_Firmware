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
	uint16_t vin1; 			/** volts */
	uint16_t vin2; 			/** volts */
	uint16_t vin3; 			/** volts */
	uint16_t vin4; 			/** volts */
	uint16_t vin5; 			/** volts */
	uint16_t vin6; 			/** volts */
	uint16_t vin7; 			/** volts */
	uint16_t vin8; 			/** volts */
	uint16_t valid;				/** 1 == within sensor range, 0 = outside sensor range */
};

/*
 * ObjDev tag for raw range finder data.
 */
ORB_DECLARE(sensor_current_sensor);

/*
 * ioctl() definitions
 *
 * Current sensor drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */


#define _RANGEFINDERIOCBASE			(0x7900)
#define __RANGEFINDERIOC(_n)		(_IOC(_RANGEFINDERIOCBASE, _n))

/** set the minimum effective distance of the device */
#define RANGEFINDERIOCSETMINIUMDISTANCE	__RANGEFINDERIOC(1)

/** set the maximum effective distance of the device */
#define RANGEFINDERIOCSETMAXIUMDISTANCE	__RANGEFINDERIOC(2)



#endif
