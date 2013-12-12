
/* @file rpm_arduino_parser.h */

#ifndef RPM_ARDUINO_PARSER_H_
#define RPM_ARDUINO_PARSER_H_

#include "rpm_arduino_helper.h"
#include <drivers/drv_rpm.h>

#define RPM_PRE 0x55

#define RPM_ARDUINO_BAUDRATE 57600


typedef enum {
	RPM_ARDUINO_DECODE_UNINIT = 0,
	RPM_ARDUINO_DECODE_GOT_SYNC1,
	RPM_ARDUINO_DECODE_GOT_CHECKSUM
} rpm_arduino_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

#pragma pack(pop)

#define RPM_ARDUINO_RECV_BUFFER_SIZE 10

class RPM_ARDUINO_PARSER : public RPM_ARDUINO_Helper
{
public:
	RPM_ARDUINO_PARSER(const int &fd, struct rpm_report *rpm_measurement);
	~RPM_ARDUINO_PARSER();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);
private:
	/**
	 * Parse the RPM_ARDUINO packet
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


	int					_fd;
	struct rpm_report 	*_rpm_measurement;
	rpm_arduino_decode_state_t	_decode_state;
	uint8_t				_rpm_arduino_revision;
	uint8_t 			rpm_arduino_last_bgps;

	uint8_t				_rx_header_lgth;
	unsigned			_rx_message_lgth;
	uint8_t				_rx_buffer[RPM_ARDUINO_RECV_BUFFER_SIZE];
	unsigned			_rx_count;
	unsigned long		_calculated_checksum;


};	//End class RPM_ARDUINO_PARSER

#endif /* RPM_ARDUINO_PARSER_H_ */
