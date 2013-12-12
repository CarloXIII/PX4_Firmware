
/* @file rpm_arduino_parser.cpp */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rpm.h>

#include "rpm_arduino_parser.h"



RPM_ARDUINO_PARSER:: RPM_ARDUINO_PARSER(const int &fd, struct rpm_report *rpm_measurement) :
_fd(fd),
//_gps_position(gps_position),
//_xsens_sensor_combined(xsens_sensor_combined),
_rpm_measurement(rpm_measurement),
_rpm_arduino_revision(0),
rpm_arduino_last_bgps(255)
{
	decode_init();
}

RPM_ARDUINO_PARSER::~RPM_ARDUINO_PARSER()
{
}

int
RPM_ARDUINO_PARSER::configure(unsigned &baudrate)
{
	/* set baudrate first */
	if (RPM_ARDUINO_Helper::set_baudrate(_fd, RPM_ARDUINO_BAUDRATE) != 0)
		return -1;

	baudrate = RPM_ARDUINO_BAUDRATE;

	return 0;
}

int
RPM_ARDUINO_PARSER::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[32];

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	int j = 0;
	ssize_t count = 0;

	while (true) {

		/* pass received bytes to the packet decoder */
		while (j < count) {
			if (parse_char(buf[j]) > 0) {
				/* return to configure during configuration or to the gps driver during normal work
				 * if a packet has arrived */
				 if (handle_message() > 0)
					return 1;
			}
			/* in case we keep trying but only get crap from GPS */
			if (time_started + timeout*1000 < hrt_absolute_time() ) {
				return -1;
			}
			j++;
		}

		/* everything is read */
		j = count = 0;

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_fd, buf, sizeof(buf));
			}
		}
	}
}

int
RPM_ARDUINO_PARSER::parse_char(uint8_t b)
{
	switch (_decode_state) {
	warnx("decoding");
		/* First, look for PRE */
		case RPM_ARDUINO_DECODE_UNINIT:
			if (b == RPM_PRE) {
				_decode_state = RPM_ARDUINO_DECODE_GOT_MESSAGE_LGTH;
				//_rx_buffer[_rx_count] = b;
				//_rx_count++;
				//warnx("PRE found");
			}
			break;
		/* Get the message */
		case RPM_ARDUINO_DECODE_GOT_MESSAGE_LGTH:
			if (_rx_count < (_rx_message_lgth)) {
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				//warnx("_rx_count: %x", _rx_count);
				//warnx("byte: %x", b);
			} else {
				//for (int i = 0; i < _rx_message_lgth+3; i++){
				//	warnx("xsens: _rx_buffer_message[%d]: %x", i, _rx_buffer[i]);
				//}
				_decode_state = RPM_ARDUINO_DECODE_GOT_CHECKSUM;
				_rx_buffer[_rx_count] = b;
				_rx_count++;

				/* compare checksum */
				_calculated_checksum = calculate_checksum(_rx_count, _rx_buffer);
				if ( (uint8_t) _calculated_checksum == 0) {
					return 1;
				} else {
					//warnx("xsens: Checksum wrong. calculated crc: %x", _calculated_checksum);
					decode_init();
					return -1;
				}
			}
			break;
		default:
			break;
	}
	return 0; //XXX ?
}

int
RPM_ARDUINO_PARSER::handle_message()
{

	int ret = 0;
	char _rx_buffer_message[_rx_message_lgth];
	memcpy(_rx_buffer_message, &(_rx_buffer[_rx_header_lgth]), _rx_message_lgth);	//braucht es vielleicht nicht


	//rpm_report *rpm_measurement;
	_rpm_measurement->rpm = 1.0f * (_rx_buffer[0] <<8 | _rx_buffer[1]);
	_rpm_measurement->timestamp = hrt_absolute_time();
	rpm_arduino_new_data = true;
	//warnx("status: %f", _rpm_measurement->rpm);

	ret = 1;

	decode_init();
	return ret;
}

void
RPM_ARDUINO_PARSER::decode_init()
{
	_decode_state = RPM_ARDUINO_DECODE_UNINIT;
	_rx_count = 0;
	_rx_message_lgth = 2;
	_rx_header_lgth = 1;
}

unsigned long
RPM_ARDUINO_PARSER::calculate_checksum(unsigned long message_lgth, unsigned char *data)
{
	unsigned long ulChecksum = 0;
	int i = 0;
	while ( message_lgth-- != 0 )
	{
		ulChecksum += (unsigned long) data[i];
		i++;
	}
	return ulChecksum;
}
