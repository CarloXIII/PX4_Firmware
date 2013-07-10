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

/* @file novatel.cpp */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "novatel.h"


NOVATEL::NOVATEL(const int &fd, struct vehicle_gps_position_s *gps_position) :
_fd(fd),
_gps_position(gps_position),
_novatel_revision(0)
{
	decode_init();
}

NOVATEL::~NOVATEL()
{
}

int
NOVATEL::configure(unsigned &baudrate)
{
	/* set baudrate first */
	if (GPS_Helper::set_baudrate(_fd, NOVATEL_BAUDRATE) != 0)
		return -1;

	baudrate = NOVATEL_BAUDRATE;

	/* Write config messages, don't wait for an answer */
	/*if (strlen(MTK_OUTPUT_5HZ) != write(_fd, MTK_OUTPUT_5HZ, strlen(MTK_OUTPUT_5HZ))) {
		warnx("mtk: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(MTK_SET_BINARY) != write(_fd, MTK_SET_BINARY, strlen(MTK_SET_BINARY))) {
		warnx("mtk: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(SBAS_ON) != write(_fd, SBAS_ON, strlen(SBAS_ON))) {
		warnx("mtk: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(WAAS_ON) != write(_fd, WAAS_ON, strlen(WAAS_ON))) {
		warnx("mtk: config write failed");
		return -1;
	}
	usleep(10000);

	if (strlen(MTK_NAVTHRES_OFF) != write(_fd, MTK_NAVTHRES_OFF, strlen(MTK_NAVTHRES_OFF))) {
		warnx("mtk: config write failed");
		return -1;
	}*/

	return 0;
}

int
NOVATEL::receive(unsigned timeout)
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
NOVATEL::parse_char(uint8_t b)
{
	switch (_decode_state) {
		/* First, look for start '#' */
		case NOVATEL_DECODE_UNINIT:
			if (b == NOVATEL_SYNC1) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC1;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			}
			break;
		/* Second, look for sync2 */
		case NOVATEL_DECODE_GOT_SYNC1:
			if (b == NOVATEL_SYNC2) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC2;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				/* Second start symbol was wrong, reset state machine */
				decode_init();
			}
			break;
		/* Third, look for sync3 */
		case NOVATEL_DECODE_GOT_SYNC2:
			if (b == NOVATEL_SYNC3) {
				_decode_state = NOVATEL_DECODE_GOT_SYNC3;
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				/* Third start symbol was wrong, reset state machine */
				decode_init();
			}
			break;
		/* Get the length of the header */
		case NOVATEL_DECODE_GOT_SYNC3:
			_decode_state = NOVATEL_DECODE_GOT_HEADER_LGTH;
			_rx_header_lgth = b;
			_rx_buffer[_rx_count] = b;
			_rx_count++;
			break;
		/* Get the rest of the header */
		case NOVATEL_DECODE_GOT_HEADER_LGTH:
			if (_rx_count < _rx_header_lgth) {
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				_decode_state = NOVATEL_DECODE_GOT_MESSAGE_LGTH;
				_rx_message_id = _rx_buffer[4] + (_rx_buffer[5] << 8);
				_rx_message_lgth = _rx_buffer[8] + (_rx_buffer[9] << 8);
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			}
			break;
		/* Get the message */
		case NOVATEL_DECODE_GOT_MESSAGE_LGTH:
			if (_rx_count < (_rx_header_lgth + _rx_message_lgth)) {
				_rx_buffer[_rx_count] = b;
				_rx_count++;
			} else {
				_decode_state = NOVATEL_DECODE_GOT_CRC1;
				_rx_crc = b;
			}
			break;
		/* Get the rest of the crc */
		case NOVATEL_DECODE_GOT_CRC1:
			_decode_state = NOVATEL_DECODE_GOT_CRC2;
			_rx_crc += b << 8;
			break;
		case NOVATEL_DECODE_GOT_CRC2:
			_decode_state = NOVATEL_DECODE_GOT_CRC3;
			_rx_crc += b << 16;
			break;
		case NOVATEL_DECODE_GOT_CRC3:
			_rx_crc += b << 24;

			/* compare checksum */
			_calculated_crc = calculate_block_crc32(_rx_count, _rx_buffer);
			if ( _calculated_crc == _rx_crc) {
				return 1;
			} else {
				warnx("novatel: Checksum wrong. calculated crc: %x, rx crc: %x", _calculated_crc, _rx_crc);
				decode_init();
				return -1;
			}
			break;
		default:
			break;
	}
	return 0; //XXX ?
}

int
NOVATEL::handle_message()
{
	int ret = 0;
	char _rx_buffer_message[_rx_message_lgth];
	memcpy(_rx_buffer_message, &(_rx_buffer[_rx_header_lgth]), _rx_message_lgth);
	gps_novatel_bestpos_packet_t *packet_bestpos;
	gps_novatel_bestvel_packet_t *packet_bestvel;

/*
	for (int i = 0; i < _rx_message_lgth; i++){
		warnx("novatel: _rx_buffer_message[%d]: %x", i, _rx_buffer_message[i]);
	}
	warnx("handle message");
*/
	switch (_rx_message_id) {
		case NOVATEL_BESTPOS:
			packet_bestpos = (gps_novatel_bestpos_packet_t *) _rx_buffer_message;

			_gps_position->lat = packet_bestpos->lat * 1e7;
			_gps_position->lon = packet_bestpos->lon * 1e7;
			_gps_position->alt = packet_bestpos->hgt * 1e3;
			_gps_position->satellites_visible = packet_bestpos->sat_tracked;
			_gps_position->timestamp_time = hrt_absolute_time();
			_gps_position->fix_type = 3;
			//_gps_position->satellite_used = packet_bestpos->sat_used;

			warnx("solstat: %d", packet_bestpos->solstat);
			warnx("postype: %d", packet_bestpos->postype);
			warnx("lat: %f", packet_bestpos->lat);
			warnx("lon: %f", packet_bestpos->lon);
			warnx("hgt: %f", packet_bestpos->hgt);
			warnx("undulation: %f", packet_bestpos->undulation);
			warnx("datum id: %d", packet_bestpos->datum_id_nr);
			warnx("sigma lat: %f", packet_bestpos->sigma_lat);
			warnx("sigma lon: %f", packet_bestpos->sigma_lon);
			warnx("sigma hgt: %f", packet_bestpos->sigma_hgt);
			warnx("base station id: %d", packet_bestpos->stn_id);
			warnx("dgps age: %f", packet_bestpos->dgps_age);
			warnx("solution age: %f", packet_bestpos->sol_age);
			warnx("sat tracked: %d", packet_bestpos->sat_tracked);
			warnx("sat used: %d", packet_bestpos->sat_used);
			warnx("#ggL1: %d", packet_bestpos->ggL1);
			warnx("#ggL1L2: %d", packet_bestpos->ggL1L2);
			warnx("extended solution status: %d", packet_bestpos->ext_sol_stat);
			warnx("signals used mask: %d", packet_bestpos->sig_mask);

			ret = 1;
			break;
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
	decode_init();
	return ret;
}

void
NOVATEL::decode_init()
{
	_decode_state = NOVATEL_DECODE_UNINIT;
	_rx_count = 0;
	_rx_header_lgth = 0;
	_rx_message_id = 0;
	_rx_message_lgth = 0;
	_rx_crc = 0;
}

unsigned long
NOVATEL::calculate_block_crc32(unsigned long message_lgth, unsigned char *data)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;
	while ( message_lgth-- != 0 )
	{
		ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
		ulTemp2 = CRC32Value( ((int) ulCRC ^ *data++ ) & 0xff );
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

unsigned long
NOVATEL::CRC32Value(int i)
{
	int j;
	unsigned long ulCRC;
	ulCRC = i;
	for ( j = 8 ; j > 0; j-- )
	{
		if ( ulCRC & 1 )
			ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
		else
			ulCRC >>= 1;
	}
	return ulCRC;
}

