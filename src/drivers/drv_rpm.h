
/**
 * @file RPM driver interface.
 */

#ifndef _DRV_RPM_H
#define _DRV_RPM_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define RPM_DEVICE_PATH	"/dev/rpm"

/**
 * range finder report structure.  Reads from the device must be in multiples of this
 * structure.
 */
struct rpm_report {
	uint64_t timestamp;
	uint32_t glitches;
	uint32_t lost_pulses;
	float rpm;
};

/*
 * ObjDev tag for raw range finder data.
 */
ORB_DECLARE(sensor_rpm);

/*
 * ioctl() definitions
 *
 * Rangefinder drivers also implement the generic sensor driver
 * interfaces from drv_sensor.h
 */

#define _RPMIOCBASE			(0x7900)
#define __RPMIOC(_n)		(_IOC(_RPMIOCBASE, _n))


#endif /* _DRV_RPM_H*/
