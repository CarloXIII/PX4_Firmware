################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../NuttX/NxWidgets/nxwm/src/capplicationwindow.cxx \
../NuttX/NxWidgets/nxwm/src/ccalibration.cxx \
../NuttX/NxWidgets/nxwm/src/cfullscreenwindow.cxx \
../NuttX/NxWidgets/nxwm/src/chexcalculator.cxx \
../NuttX/NxWidgets/nxwm/src/ckeyboard.cxx \
../NuttX/NxWidgets/nxwm/src/cmediaplayer.cxx \
../NuttX/NxWidgets/nxwm/src/cnxconsole.cxx \
../NuttX/NxWidgets/nxwm/src/cstartwindow.cxx \
../NuttX/NxWidgets/nxwm/src/ctaskbar.cxx \
../NuttX/NxWidgets/nxwm/src/ctouchscreen.cxx \
../NuttX/NxWidgets/nxwm/src/cwindowmessenger.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_calculator.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_calibration.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_cmd.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_mediaplayer.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_minimize.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_mplayer_controls.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_nsh.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_play.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_start.cxx \
../NuttX/NxWidgets/nxwm/src/glyph_stop.cxx 

OBJS += \
./NuttX/NxWidgets/nxwm/src/capplicationwindow.o \
./NuttX/NxWidgets/nxwm/src/ccalibration.o \
./NuttX/NxWidgets/nxwm/src/cfullscreenwindow.o \
./NuttX/NxWidgets/nxwm/src/chexcalculator.o \
./NuttX/NxWidgets/nxwm/src/ckeyboard.o \
./NuttX/NxWidgets/nxwm/src/cmediaplayer.o \
./NuttX/NxWidgets/nxwm/src/cnxconsole.o \
./NuttX/NxWidgets/nxwm/src/cstartwindow.o \
./NuttX/NxWidgets/nxwm/src/ctaskbar.o \
./NuttX/NxWidgets/nxwm/src/ctouchscreen.o \
./NuttX/NxWidgets/nxwm/src/cwindowmessenger.o \
./NuttX/NxWidgets/nxwm/src/glyph_calculator.o \
./NuttX/NxWidgets/nxwm/src/glyph_calibration.o \
./NuttX/NxWidgets/nxwm/src/glyph_cmd.o \
./NuttX/NxWidgets/nxwm/src/glyph_mediaplayer.o \
./NuttX/NxWidgets/nxwm/src/glyph_minimize.o \
./NuttX/NxWidgets/nxwm/src/glyph_mplayer_controls.o \
./NuttX/NxWidgets/nxwm/src/glyph_nsh.o \
./NuttX/NxWidgets/nxwm/src/glyph_play.o \
./NuttX/NxWidgets/nxwm/src/glyph_start.o \
./NuttX/NxWidgets/nxwm/src/glyph_stop.o 

CXX_DEPS += \
./NuttX/NxWidgets/nxwm/src/capplicationwindow.d \
./NuttX/NxWidgets/nxwm/src/ccalibration.d \
./NuttX/NxWidgets/nxwm/src/cfullscreenwindow.d \
./NuttX/NxWidgets/nxwm/src/chexcalculator.d \
./NuttX/NxWidgets/nxwm/src/ckeyboard.d \
./NuttX/NxWidgets/nxwm/src/cmediaplayer.d \
./NuttX/NxWidgets/nxwm/src/cnxconsole.d \
./NuttX/NxWidgets/nxwm/src/cstartwindow.d \
./NuttX/NxWidgets/nxwm/src/ctaskbar.d \
./NuttX/NxWidgets/nxwm/src/ctouchscreen.d \
./NuttX/NxWidgets/nxwm/src/cwindowmessenger.d \
./NuttX/NxWidgets/nxwm/src/glyph_calculator.d \
./NuttX/NxWidgets/nxwm/src/glyph_calibration.d \
./NuttX/NxWidgets/nxwm/src/glyph_cmd.d \
./NuttX/NxWidgets/nxwm/src/glyph_mediaplayer.d \
./NuttX/NxWidgets/nxwm/src/glyph_minimize.d \
./NuttX/NxWidgets/nxwm/src/glyph_mplayer_controls.d \
./NuttX/NxWidgets/nxwm/src/glyph_nsh.d \
./NuttX/NxWidgets/nxwm/src/glyph_play.d \
./NuttX/NxWidgets/nxwm/src/glyph_start.d \
./NuttX/NxWidgets/nxwm/src/glyph_stop.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/NxWidgets/nxwm/src/%.o: ../NuttX/NxWidgets/nxwm/src/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


