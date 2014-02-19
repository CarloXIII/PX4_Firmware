############################################################################
#
# @author Benedikt Imbach, 2013
#
#
#
# rpm driver with arduino board as measurement unit over serial RS-232
#
############################################################################



MODULE_COMMAND	= rpm_arduino

SRCS		= rpm_arduino.cpp \
			rpm_arduino_helper.cpp \
			rpm_arduino_parser.cpp
