############################################################################
#
# (c) Benedikt Imbach 2013
# benedikt.imbach@hslu.ch
# PX4 project
#
############################################################################

#
# Makefile to build the current sensor driver based on the FHS 40-PSP600
# current sensor in combination with the ADC AD7997 with I2C interface
#

MODULE_COMMAND		= current_sensor
MODULE_STACKSIZE	= 2048

SRCS			= current_sensor.cpp
