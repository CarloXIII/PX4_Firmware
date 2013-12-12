
/** 
 * @file rpm_arduino_helper.h
 */

#ifndef RPM_ARDUINO_HELPER_H
#define RPM_ARDUINO_HELPER_H

#include <uORB/uORB.h>
//#include <uORB/topics/xsens_vehicle_gps_position.h>
//#include <uORB/topics/xsens_sensor_combined.h>

class RPM_ARDUINO_Helper
{
public:
	virtual int			configure(unsigned &baud) = 0;
	virtual int 		receive(unsigned timeout) = 0;
	int 				set_baudrate(const int &fd, unsigned baud);
	float				get_update_rate();
	float				reset_update_rates();
	float				store_update_rates();
	bool				rpm_arduino_new_data;
protected:

	float _rate_measurement;
	uint8_t _rate_count;

	uint64_t _interval_rate_start;
};

#endif /* RPM_ARDUINO_HELPER_H */
