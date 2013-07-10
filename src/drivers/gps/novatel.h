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

/* @file novatel.h */

#ifndef NOVATEL_H_
#define NOVATEL_H_

#include "gps_helper.h"


#define NOVATEL_SYNC1 0xAA
#define NOVATEL_SYNC2 0x44
#define NOVATEL_SYNC3 0x12
/* MessageIDs (the ones that are used) */
#define NOVATEL_BESTPOS 42
#define NOVATEL_BESTVEL 99

#define CRC32_POLYNOMIAL 0xEDB88320

/*
#define MTK_OUTPUT_5HZ		"$PMTK220,200*2C\r\n"
#define MTK_SET_BINARY		"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define SBAS_ON	        	"$PMTK313,1*2E\r\n"
#define WAAS_ON				"$PMTK301,2*2E\r\n"
#define MTK_NAVTHRES_OFF 	"$PMTK397,0*23\r\n"

#define MTK_TIMEOUT_5HZ 400*/
#define NOVATEL_BAUDRATE 38400


typedef enum {
	NOVATEL_DECODE_UNINIT = 0,
	NOVATEL_DECODE_GOT_SYNC1,
	NOVATEL_DECODE_GOT_SYNC2,
	NOVATEL_DECODE_GOT_SYNC3,
	NOVATEL_DECODE_GOT_HEADER_LGTH,
	NOVATEL_DECODE_GOT_MESSAGE_LGTH,
	NOVATEL_DECODE_GOT_CRC1,
	NOVATEL_DECODE_GOT_CRC2,
	NOVATEL_DECODE_GOT_CRC3
} novatel_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t postype; /**< Position type */
	double_t lat; /**< Latitude, deg */
	double_t lon; /**< Longitude, deg */
	double_t hgt; /**< Height above mean sea level, m */
	float_t undulation; /**< Undulation - the relationship between the geoid and the ellipsoid (m) of the chosen datum */
	uint32_t datum_id_nr; /**< Datum ID number */
	float_t sigma_lat; /**< Latitude standard deviation, XXX */
	float_t sigma_lon; /**< Longitude standard deviation, XXX */
	float_t sigma_hgt; /**< Height standard deviation, XXX */
	uint32_t stn_id; /**< Base station ID */
	float_t dgps_age; /**< Differential age, s */
	float_t sol_age; /**< Solution age, s */
	uint8_t sat_tracked; /**< Number of satellite vehicles tracked */
	uint8_t sat_used; /**< Number of satellite vehicles used in solution */
	uint8_t ggL1; /**< Number of GPS plus GLONASS L1 used in solution */
	uint8_t ggL1L2; /**< Number of GPS plus GLONASS L1 and L2 used in solution */
	uint8_t reserved_1; /**< Reserved */
	uint8_t ext_sol_stat; /**< Extended solution status */
	uint8_t reserved_2; /**< Reserved */
	uint8_t sig_mask; /**< Signals used mak - if 0, signals used in solution are unknown */
} gps_novatel_bestpos_packet_t;

typedef struct {
	uint32_t solstat; /**< Solution status */
	uint32_t veltype; /**< Velocity type */
	float_t latency; /**< A measure of the latency in the velocity time tag inseconds. It should be subtracted from the time to give improved results */
	float_t dgps_age; /**< Differential age, s */
	double_t hor_spd; /**< Horizontal speed over ground, in meters per second */
	double_t trk_gnd; /**< Actual direction of motion over ground (track over ground) with respect to True North, in degrees */
	uint8_t vert_spd; /**< Vertical speed, in meters per second, where positive values indicate increasing altitude (up) and negative values indicate decreasing altitude (down) */
	uint8_t reserved; /**< Reserved */
} gps_novatel_bestvel_packet_t;


#pragma pack(pop)

#define NOVATEL_RECV_BUFFER_SIZE 150

class NOVATEL : public GPS_Helper
{
public:
	NOVATEL(const int &fd, struct vehicle_gps_position_s *gps_position);
	~NOVATEL();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the binary NOVATEL packet
	 */
	int				parse_char(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int				handle_message(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void				decode_init(void);

	/**
	 * Calculate the CRC-32 of a block of data
	 */
	unsigned long		calculate_block_crc32(unsigned long message_lgth, unsigned char *data);
	unsigned long 		CRC32Value(int i);

	int					_fd;
	struct vehicle_gps_position_s *_gps_position;
	novatel_decode_state_t	_decode_state;
	uint8_t				_novatel_revision;
	uint8_t				_rx_header_lgth;
	unsigned			_rx_message_lgth;
	unsigned			_rx_message_id;
	uint8_t				_rx_buffer[NOVATEL_RECV_BUFFER_SIZE];
	unsigned char		_crc_buffer[NOVATEL_RECV_BUFFER_SIZE];
	unsigned			_rx_count;
	uint32_t 			_rx_crc;
	unsigned long		_calculated_crc;
};

#endif /* NOVATEL_H_ */
