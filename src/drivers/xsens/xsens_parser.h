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

/* @file xsens_parser.h */

#ifndef XSENS_PARSER_H_
#define XSENS_PARSER_H_

#include "xsens_helper.h"


#define XSENS_PRE 0xFA
#define XSENS_BID 0xFF
#define XSENS_MID 0x32

#define NOVATEL_BESTPOS 0x1
#define NOVATEL_BESTVEL 0x2

#define CRC32_POLYNOMIAL 0xEDB88320

/*
#define MTK_OUTPUT_5HZ		"$PMTK220,200*2C\r\n"
#define MTK_SET_BINARY		"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define SBAS_ON	        	"$PMTK313,1*2E\r\n"
#define WAAS_ON				"$PMTK301,2*2E\r\n"
#define MTK_NAVTHRES_OFF 	"$PMTK397,0*23\r\n"

#define MTK_TIMEOUT_5HZ 400*/
#define XSENS_BAUDRATE 38400


typedef enum {
	XSENS_DECODE_UNINIT = 0,
	XSENS_DECODE_GOT_SYNC1,
	XSENS_DECODE_GOT_SYNC2,
	XSENS_DECODE_GOT_SYNC3,
	XSENS_DECODE_GOT_HEADER_LGTH,
	XSENS_DECODE_GOT_MESSAGE_LGTH,
	XSENS_DECODE_GOT_CHECKSUM
} xsens_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	uint8_t bgps; /**< GPS status byte or GPS data age. */
	uint32_t sacc; /**< Speed Accuracy Estimate. Expected error standard deviation, cm/s */
	uint32_t vacc; /**< Vertical Accuracy Estimate. Expected error standard deviation, mm */
	uint32_t hacc; /**< Horizontal Accuracy Estimate. Expected error standard deviation, mm */
	int32_t vel_d; /**< NED down velocity, cm/s */
	int32_t vel_e; /**< NED east velocity, cm/s */
	int32_t vel_n; /**< NED north velocity, cm/s */
	int32_t alt; /**< Height above mean sea level, mm */
	int32_t lon; /**< Longitude, deg*1e-7 */
	int32_t lat; /**< Latitude, deg*1e-7 */
	uint32_t itow; /**< GPS Millisecond Time of Week, ms */
	uint8_t bprs; /**< Pressure sensor status. When the value decreases, new pressure data is available */
	uint16_t press; /**< Pressure, Pa*2 */
}xsens_gps_pvt_t;

typedef struct {
	float_t temp; /**< Internal temperatur of the sensor, °C */
}xsens_temp_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t magz; /**< XXX */
	float_t magy; /**< XXX */
	float_t magx; /**< XXX */
	float_t gyrz; /**< XXX */
	float_t gyry; /**< XXX */
	float_t gyrx; /**< XXX */
	float_t accz; /**< XXX */
	float_t accy; /**< XXX */
	float_t accx; /**< XXX */
}xsens_calibrated_data_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t q3; /**< XXX */
	float_t q2; /**< XXX */
	float_t q1; /**< XXX */
	float_t q0; /**< XXX */
}xsens_orientation_quaternion_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t yaw; /**< XXX */
	float_t pitch; /**< XXX */
	float_t roll; /**< XXX */
}xsens_orientation_euler_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t i; /**< XXX */
	float_t h; /**< XXX */
	float_t g; /**< XXX */
	float_t f; /**< XXX */
	float_t e; /**< XXX */
	float_t d; /**< XXX */
	float_t c; /**< XXX */
	float_t b; /**< XXX */
	float_t a; /**< XXX */
}xsens_orientation_matrix_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	uint16_t ain2; /**< XXX */
	uint16_t ain1; /**< XXX */
}xsens_auxiliary_data_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t alt; /**< XXX */
	float_t lon; /**< XXX */
	float_t lat; /**< XXX */
}xsens_position_data_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	float_t velz; /**< XXX */
	float_t vely; /**< XXX */
	float_t velx; /**< XXX */
}xsens_velocity_data_t;

typedef struct {
	uint8_t status; /**< XXX */
}xsens_status_t;

typedef struct {
	uint16_t sample_counter; /**< XXX */
}xsens_sample_counter_t;

typedef struct { // reverse order because of swapping the bytes (little/big endian)
	uint8_t status; /**< 0x01 = Valid Time of Week; 0x02 = Valid Week Number; 0x04 = Valid UTC (Leap Seconds already known?) */
	uint8_t seconds; /**< Seconds of minute, range 0 .. 59 */
	uint8_t minute; /**< Minute of hour, range 0 .. 59 */
	uint8_t hour; /**< Hour of day, range 0 .. 23 */
	uint8_t day; /**< Day of month, range 1 .. 31 */
	uint8_t month; /**< Month, range 1 .. 12 */
	uint16_t year; /**< Year, range 1999 .. 2099 */
	uint32_t nsec; /**< Nanoseconds of second, range 0 .. 1.000.000.000 */
}xsens_utc_time_t;

#pragma pack(pop)

#define XSENS_RECV_BUFFER_SIZE 150

class XSENS_PARSER : public XSENS_Helper
{
public:
	XSENS_PARSER(const int &fd, struct xsens_vehicle_gps_position_s *gps_position);
	~XSENS_PARSER();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the XSENS packet
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
	 * Calculate the checksum of a block of data
	 */
	unsigned long		calculate_checksum(unsigned long message_lgth, unsigned char *data);

	void				swapBytes(char* message, unsigned size);

	int					_fd;
	struct xsens_vehicle_gps_position_s *_gps_position;
	xsens_decode_state_t	_decode_state;
	uint8_t				_xsens_revision;
	uint8_t				_rx_header_lgth;
	unsigned			_rx_message_lgth;
	uint8_t				_rx_buffer[XSENS_RECV_BUFFER_SIZE];
	char				_messageSwapped[50];
	unsigned			_rx_count;
	unsigned long		_calculated_checksum;
};

#endif /* XSENS_PARSER_H_ */
