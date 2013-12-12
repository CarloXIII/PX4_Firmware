
/**
 * @file RPM driver interface.
 */

#ifndef _DRV_RPM_ARDUINO_H
#define _DRV_RPM_ARDUINO_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_sensor.h"
#include "drv_orb_dev.h"

#define RPM_ARDUINO_DEFAULT_UART_PORT "/dev/ttyS3"

#define RPM_ARDUINO_DEVICE_PATH	"/dev/rpm_arduino"


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



#define _RPMIOCBASE			(0x7900)
#define __RPMIOC(_n)		(_IOC(_RPMIOCBASE, _n))


#endif /* _DRV_RPM_ARDUINO_H*/
