/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
 *   Author: Carlo Zgraggen <carlo.zgraggen@hslu.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* @file xsens_parser.cpp */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rpm.h>

#include "xsens_parser.h"

#define XSENS_GPS_PVT 1
#define XSENS_TEMP 1
#define XSENS_CALIBRATED_DATA 1
#define XSENS_ORIENTATION_QUATERNION 0
#define XSENS_ORIENTATION_EULER 0
#define XSENS_ORIENTATION_MATRIX 0
#define XSENS_AUXILIARY 0
#define XSENS_POSITION 0
#define XSENS_VELOCITY 0
#define XSENS_STATUS 1
#define XSENS_SAMPLE_COUNTER 1
#define XSENS_UTC_TIME 1


XSENS_PARSER::XSENS_PARSER(const int &fd, struct rpm_message * rpm_measurement :
_fd(fd),
_rpm_measurement(rpm_measurement),
_xsens_revision(0),
xsens_last_bgps(255)
{
	decode_init();
}

XSENS_PARSER::~XSENS_PARSER()
{
}

int
XSENS_PARSER::configure(unsigned &baudrate)
{
	/* set baudrate first */
	if (XSENS_Helper::set_baudrate(_fd, XSENS_BAUDRATE) != 0)
		return -1;

	baudrate = XSENS_BAUDRATE;

	return 0;
}

int
XSENS_PARSER::receive(unsigned timeout)
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
XSENS_PARSER::parse_char(uint8_t b)
{
	switch (_decode_state) {
	//warnx("decoding");
		/* First, look for PRE */
		case XSENS_DECODE_UNINIT:
			if (b == XSENS_PRE) {
				_decode_state = XSENS_DECODE_GOT_SYNC1;
				//_rx_buffer[_rx_count] = b;
				//_rx_count++;
				//warnx("PRE found");
			}
			break;
		/* Second, look for BID */
		case XSENS_DECODE_GOT_SYNC1:
			if (b == XSENS_BID) {
				_decode_state = XSENS_DECODE_GOT_SYNC2;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				//warnx("BID found");
			} else {
				/* Second start symbol was wrong, reset state machine */
				decode_init();
			}
			break;
		/* Third, look for MID */
		case XSENS_DECODE_GOT_SYNC2:
			if (b == XSENS_MID) {
				_decode_state = XSENS_DECODE_GOT_SYNC3;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				//warnx("MID found");
			} else {
				/* Third start symbol was wrong, reset state machine */
				decode_init();
			}
			break;
		/* Get the length of the message */
		case XSENS_DECODE_GOT_SYNC3:
			_decode_state = XSENS_DECODE_GOT_MESSAGE_LGTH;
			_rx_message_lgth = b;
			_rx_buffer[_rx_count] = b;
			_rx_count++;
			//warnx("LEN: %x", _rx_message_lgth);
			break;
		/* Get the message */
		case XSENS_DECODE_GOT_MESSAGE_LGTH:
			if (_rx_count < (_rx_message_lgth + 3)) { //+BID+MID+LEN, Preamble not part of the message
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				//warnx("_rx_count: %x", _rx_count);
				//warnx("byte: %x", b);
			} else {
				//for (int i = 0; i < _rx_message_lgth+3; i++){
				//	warnx("xsens: _rx_buffer_message[%d]: %x", i, _rx_buffer[i]);
				//}
				_decode_state = XSENS_DECODE_GOT_CHECKSUM;
				_rx_buffer[_rx_count] = b;
				_rx_count++;

				/* compare checksum */
				_calculated_checksum = calculate_checksum(_rx_count, _rx_buffer);
				if ( (uint8_t) _calculated_checksum == 0) {
					return 1;
				} else {
					warnx("xsens: Checksum wrong. calculated crc: %x", _calculated_checksum);
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
XSENS_PARSER::handle_message()
{

	int ret = 0;
	char _rx_buffer_message[_rx_message_lgth];
	memcpy(_rx_buffer_message, &(_rx_buffer[_rx_header_lgth]), _rx_message_lgth);



#if XSENS_CALIBRATED_DATA
	unsigned xsens_calibrated_lgth = 36;
	char _xsens_calibrated_message[xsens_calibrated_lgth];
	memcpy(_xsens_calibrated_message, &(_rx_buffer[_rx_header_lgth]), xsens_calibrated_lgth);

	swapBytes(_xsens_calibrated_message, xsens_calibrated_lgth);

	xsens_calibrated_data_t *xsens_calibrated;
	xsens_calibrated = (xsens_calibrated_data_t *) _xsens_calibrated_message;


	_xsens_sensor_combined->accelerometer_m_s2[0] = xsens_calibrated->accx;
	_xsens_sensor_combined->accelerometer_m_s2[1] = xsens_calibrated->accy;
	_xsens_sensor_combined->accelerometer_m_s2[2] = xsens_calibrated->accz;
	_xsens_sensor_combined->accelerometer_counter += 1;

	_xsens_sensor_combined->gyro_rad_s[0] = xsens_calibrated->gyrx/2;	// XXX todo: chage normalized to gauss
	_xsens_sensor_combined->gyro_rad_s[1] = xsens_calibrated->gyry/2; // XXX todo: chage normalized to gauss
	_xsens_sensor_combined->gyro_rad_s[2] = xsens_calibrated->gyrz/2; // XXX todo: chage normalized to gauss (1gauss = 100*10^-6 Tesla) -> Erdmagnetfeld ca. 50uT=0.5gauss)
	_xsens_sensor_combined->gyro_counter += 1;

	_xsens_sensor_combined->magnetometer_ga[0] = xsens_calibrated->magx;
	_xsens_sensor_combined->magnetometer_ga[1] = xsens_calibrated->magy;
	_xsens_sensor_combined->magnetometer_ga[2] = xsens_calibrated->magz;
	_xsens_sensor_combined->magnetometer_counter += 1;

	_xsens_sensor_combined->timestamp = hrt_absolute_time();

	_rx_header_lgth += xsens_calibrated_lgth;
#endif


#if XSENS_UTC_TIME
	unsigned xsens_utc_lgth = 12;
	char _xsens_utc_message[xsens_utc_lgth];
	memcpy(_xsens_utc_message, &(_rx_buffer[_rx_header_lgth]), xsens_utc_lgth);

	swapBytes(_xsens_utc_message, xsens_utc_lgth);

	xsens_utc_time_t *xsens_utc;
	xsens_utc = (xsens_utc_time_t *) _xsens_utc_message;

	/*
	warnx("status: %d", xsens_utc->status);
	warnx("year: %d", xsens_utc->year);
	warnx("month: %d", xsens_utc->month);
	warnx("day: %d", xsens_utc->day);
	warnx("hour: %d", xsens_utc->hour);
	warnx("minute: %d", xsens_utc->minute);
	warnx("second: %d", xsens_utc->seconds);
	warnx("nanoseconds: %d", xsens_utc->nsec);
	*/

	_rx_header_lgth += xsens_utc_lgth;
#endif

	ret = 1;

	decode_init();
	return ret;
}

void
XSENS_PARSER::decode_init()
{
	_decode_state = XSENS_DECODE_UNINIT;
	_rx_count = 0;
	_rx_message_lgth = 0;
	_rx_header_lgth = 1;
}

unsigned long
XSENS_PARSER::calculate_checksum(unsigned long message_lgth, unsigned char *data)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulChecksum = 0;
	int i = 0;
	while ( message_lgth-- != 0 )
	{
		ulChecksum += (unsigned long) data[i];
		i++;
	}
	return ulChecksum;
}

void
XSENS_PARSER::swapBytes(char* message, unsigned size)
{
	for(int i = 0; i < (size / 2); i++)
	{
		_messageSwapped[i] = message[size-1-i];
		_messageSwapped[size-1-i] = message[i];
		message[i] = _messageSwapped[i];
		message[size-1-i] = _messageSwapped[size-1-i];
	}
	return;
}

