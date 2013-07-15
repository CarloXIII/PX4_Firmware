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

#include "xsens_parser.h"

#define XSENS_GPS_PVT 0
#define XSENS_TEMP 1
#define XSENS_CALIBRATED_DATA 0
#define XSENS_ORIENTATION_QUATERNION 0
#define XSENS_ORIENTATION_EULER 1
#define XSENS_ORIENTATION_MATRIX 0
#define XSENS_AUXILIARY 0
#define XSENS_POSITION 1
#define XSENS_VELOCITY 1
#define XSENS_STATUS 1
#define XSENS_SAMPLE_COUNTER 1
#define XSENS_UTC_TIME 1


XSENS_PARSER::XSENS_PARSER(const int &fd, struct xsens_vehicle_gps_position_s *gps_position) :
_fd(fd),
_gps_position(gps_position),
_xsens_revision(0)
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
	warnx("decoding");
		/* First, look for PRE */
		case XSENS_DECODE_UNINIT:
			if (b == XSENS_PRE) {
				_decode_state = XSENS_DECODE_GOT_SYNC1;
				//_rx_buffer[_rx_count] = b;
				//_rx_count++;
				warnx("PRE found");
			}
			break;
		/* Second, look for BID */
		case XSENS_DECODE_GOT_SYNC1:
			if (b == XSENS_BID) {
				_decode_state = XSENS_DECODE_GOT_SYNC2;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
				warnx("BID found");
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
				warnx("MID found");
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
			warnx("LEN: %x", _rx_message_lgth);
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


#if XSENS_GPS_PVT
	unsigned xsens_gps_lgth = 44;
	char _xsens_gps_message[xsens_gps_lgth];
	memcpy(_xsens_gps_message, &(_rx_buffer[_rx_header_lgth]), xsens_gps_lgth);

	swapBytes(_xsens_gps_message, xsens_gps_lgth);

	xsens_gps_pvt_t *xsens_gps_pvt;
	xsens_gps_pvt = (xsens_gps_pvt_t *) _xsens_gps_message;

	_gps_position->lat = xsens_gps_pvt->lat;
	_gps_position->lon = xsens_gps_pvt->lon;
	_gps_position->alt = xsens_gps_pvt->alt;
	//_gps_position->satellites_visible = packet_bestpos->sat_tracked;
	//_gps_position->vel_m_s = packet_bestvel->hor_spd;
	//_gps_position->cog_rad = packet_bestvel->trk_gnd;
	_gps_position->vel_n_m_s = xsens_gps_pvt->vel_n;
	_gps_position->vel_e_m_s = xsens_gps_pvt->vel_e;
	_gps_position->vel_d_m_s = xsens_gps_pvt->vel_d;
	_gps_position->vel_ned_valid = true;
	_gps_position->timestamp_position = hrt_absolute_time();
	_gps_position->timestamp_velocity = hrt_absolute_time();
	_gps_position->fix_type = 3;



	_rx_header_lgth += xsens_gps_lgth;
#endif

#if XSENS_TEMP
	unsigned xsens_temp_lgth = 4;
	char _xsens_temp_message[xsens_temp_lgth];
	memcpy(_xsens_temp_message, &(_rx_buffer[_rx_header_lgth]), xsens_temp_lgth);

	swapBytes(_xsens_temp_message, xsens_temp_lgth);

	xsens_temp_t *xsens_temp;
	xsens_temp = (xsens_temp_t *) _xsens_temp_message;

	warnx("xsens_temp: %f", xsens_temp->temp);

	_rx_header_lgth += xsens_temp_lgth;
#endif

#if XSENS_CALIBRATED_DATA
	unsigned xsens_calibrated_lgth = 36;
	char _xsens_calibrated_message[xsens_calibrated_lgth];
	memcpy(_xsens_calibrated_message, &(_rx_buffer[_rx_header_lgth]), xsens_calibrated_lgth);

	swapBytes(_xsens_calibrated_message, xsens_calibrated_lgth);

	xsens_calibrated_data_t *xsens_calibrated;
	xsens_calibrated = (xsens_calibrated_data_t *) _xsens_calibrated_message;

	warnx("accx: %f", xsens_calibrated->accx);
	warnx("accy: %f", xsens_calibrated->accy);
	warnx("accz: %f", xsens_calibrated->accz);
	warnx("gyrx: %f", xsens_calibrated->gyrx);
	warnx("gyry: %f", xsens_calibrated->gyry);
	warnx("gyrz: %f", xsens_calibrated->gyrz);
	warnx("magx: %f", xsens_calibrated->magx);
	warnx("magy: %f", xsens_calibrated->magy);
	warnx("magz: %f", xsens_calibrated->magz);

	_rx_header_lgth += xsens_calibrated_lgth;
#endif

#if XSENS_ORIENTATION_QUATERNION
	unsigned xsens_quaternion_lgth = 16;
	char _xsens_quaternion_message[xsens_quaternion_lgth];
	memcpy(_xsens_quaternion_message, &(_rx_buffer[_rx_header_lgth]), xsens_quaternion_lgth);

	swapBytes(_xsens_quaternion_message, xsens_quaternion_lgth);

	xsens_orientation_quaternion_t *xsens_quaternion;
	xsens_quaternion = (xsens_orientation_quaternion_t *) _xsens_quaternion_message;

	_rx_header_lgth += xsens_quaternion_lgth;
#endif

#if XSENS_ORIENTATION_EULER
	unsigned xsens_euler_lgth = 12;
	char _xsens_euler_message[xsens_euler_lgth];
	memcpy(_xsens_euler_message, &(_rx_buffer[_rx_header_lgth]), xsens_euler_lgth);

	swapBytes(_xsens_euler_message, xsens_euler_lgth);

	xsens_orientation_euler_t *xsens_euler;
	xsens_euler = (xsens_orientation_euler_t *) _xsens_euler_message;

	warnx("xsens_roll: %f", xsens_euler->roll);
	warnx("xsens_pitch: %f", xsens_euler->pitch);
	warnx("xsens_yaw: %f", xsens_euler->yaw);

	_rx_header_lgth += xsens_euler_lgth;
#endif

#if XSENS_ORIENTATION_MATRIX
	unsigned xsens_matrix_lgth = 36;
	char _xsens_matrix_message[xsens_matrix_lgth];
	memcpy(_xsens_matrix_message, &(_rx_buffer[_rx_header_lgth]), xsens_matrix_lgth);

	swapBytes(_xsens_matrix_message, xsens_matrix_lgth);

	xsens_orientation_matrix_t *xsens_matrix;
	xsens_matrix = (xsens_orientation_matrix_t *) _xsens_matrix_message;

	_rx_header_lgth += xsens_matrix_lgth;
#endif

#if XSENS_AUXILIARY
	unsigned xsens_auxiliary_lgth = 4;
	char _xsens_auxiliary_message[xsens_auxiliary_lgth];
	memcpy(_xsens_auxiliary_message, &(_rx_buffer[_rx_header_lgth]), xsens_auxiliary_lgth);

	swapBytes(_xsens_auxiliary_message, xsens_auxiliary_lgth);

	xsens_auxiliary_data_t *xsens_auxiliary;
	xsens_auxiliary = (xsens_auxiliary_data_t *) _xsens_auxiliary_message;

	_rx_header_lgth += xsens_auxiliary_lgth;
#endif

#if XSENS_POSITION
	unsigned xsens_position_lgth = 12;
	char _xsens_position_message[xsens_position_lgth];
	memcpy(_xsens_position_message, &(_rx_buffer[_rx_header_lgth]), xsens_position_lgth);

	swapBytes(_xsens_position_message, xsens_position_lgth);

	xsens_position_data_t *xsens_position;
	xsens_position = (xsens_position_data_t *) _xsens_position_message;

	warnx("xsens_lat: %f", xsens_position->lat);
	warnx("xsens_lon: %f", xsens_position->lon);
	warnx("xsens_alt: %f", xsens_position->alt);

	_gps_position->lat = xsens_position->lat;
	_gps_position->lon = xsens_position->lon;
	_gps_position->alt = xsens_position->alt;
	_gps_position->timestamp_position = hrt_absolute_time();
	_gps_position->fix_type = 3;

	_rx_header_lgth += xsens_position_lgth;
#endif

#if XSENS_VELOCITY
	unsigned xsens_velocity_lgth = 12;
	char _xsens_velocity_message[xsens_velocity_lgth];
	memcpy(_xsens_velocity_message, &(_rx_buffer[_rx_header_lgth]), xsens_velocity_lgth);

	swapBytes(_xsens_velocity_message, xsens_velocity_lgth);

	xsens_velocity_data_t *xsens_velocity;
	xsens_velocity = (xsens_velocity_data_t *) _xsens_velocity_message;

	warnx("xsens_velx: %f", xsens_velocity->velx);
	warnx("xsens_vely: %f", xsens_velocity->vely);
	warnx("xsens_velz: %f", xsens_velocity->velz);

	_gps_position->vel_n_m_s = xsens_velocity->velx;
	_gps_position->vel_e_m_s = xsens_velocity->vely;
	_gps_position->vel_d_m_s = xsens_velocity->velz;
	_gps_position->vel_ned_valid = true;
	_gps_position->timestamp_velocity = hrt_absolute_time();
	_gps_position->fix_type = 3;

	_rx_header_lgth += xsens_velocity_lgth;
#endif

#if XSENS_STATUS
	unsigned xsens_status_lgth = 1;

	xsens_status_t *xsens_status;
	xsens_status->status = _rx_buffer[_rx_header_lgth];

	warnx("status: %d", xsens_status->status);

	_rx_header_lgth += xsens_status_lgth;
#endif

#if XSENS_SAMPLE_COUNTER
	unsigned xsens_sample_lgth = 2;
	char _xsens_sample_message[xsens_sample_lgth];
	memcpy(_xsens_sample_message, &(_rx_buffer[_rx_header_lgth]), xsens_sample_lgth);

	swapBytes(_xsens_sample_message, xsens_sample_lgth);

	xsens_sample_counter_t *xsens_sample;
	xsens_sample = (xsens_sample_counter_t *) _xsens_sample_message;

	_rx_header_lgth += xsens_sample_lgth;
#endif

#if XSENS_UTC_TIME
	unsigned xsens_utc_lgth = 12;
	char _xsens_utc_message[xsens_utc_lgth];
	memcpy(_xsens_utc_message, &(_rx_buffer[_rx_header_lgth]), xsens_utc_lgth);

	swapBytes(_xsens_utc_message, xsens_utc_lgth);

	xsens_utc_time_t *xsens_utc;
	xsens_utc = (xsens_utc_time_t *) _xsens_utc_message;

	warnx("status: %d", xsens_utc->status);
	warnx("year: %d", xsens_utc->year);
	warnx("month: %d", xsens_utc->month);
	warnx("day: %d", xsens_utc->day);
	warnx("hour: %d", xsens_utc->hour);
	warnx("minute: %d", xsens_utc->minute);
	warnx("second: %d", xsens_utc->seconds);
	warnx("nanoseconds: %d", xsens_utc->nsec);

	_rx_header_lgth += xsens_utc_lgth;
#endif

/*
		case NOVATEL_BESTVEL:
			packet_bestvel = (gps_novatel_bestvel_packet_t *) _rx_buffer_message;

			_gps_position->vel_m_s = packet_bestvel->hor_spd;
			_gps_position->cog_rad = packet_bestvel->trk_gnd;
			_gps_position->vel_n_m_s = packet_bestvel->hor_spd * cosf(packet_bestvel->trk_gnd * M_DEG_TO_RAD_F);
			_gps_position->vel_e_m_s = packet_bestvel->hor_spd * sinf(packet_bestvel->trk_gnd * M_DEG_TO_RAD_F);
			_gps_position->vel_d_m_s = -packet_bestvel->vert_spd;
			_gps_position->vel_ned_valid = true;
			_gps_position->timestamp_velocity = hrt_absolute_time();
			_gps_position->fix_type = 3;

			warnx("solstat: %d", packet_bestvel->solstat);
			warnx("veltype: %d", packet_bestvel->veltype);
			warnx("latency: %f", packet_bestvel->latency);
			warnx("dgps age: %f", packet_bestvel->dgps_age);
			warnx("horizontal speed: %f", packet_bestvel->hor_spd);
			warnx("flight direction: %f", packet_bestvel->trk_gnd);
			warnx("north speed: %d", _gps_position->vel_n_m_s);
			warnx("east speed: %d", _gps_position->vel_e_m_s);
			warnx("down speed: %d", _gps_position->vel_d_m_s);

			ret = 1;
			break;
		default:
			warnx("NOVATEL: Unknown message received: %d\n", _rx_message_id);
			ret = -1;
			return ret;
			break;

	}

	*/
	decode_init();
	return ret;
}

void
XSENS_PARSER::decode_init()
{
	_decode_state = XSENS_DECODE_UNINIT;
	_rx_count = 0;
	_rx_message_lgth = 0;
	_rx_header_lgth = 3;
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

