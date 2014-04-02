################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../NuttX/apps/examples/nxflat/tests/hello++/hello++1.cpp \
../NuttX/apps/examples/nxflat/tests/hello++/hello++2.cpp \
../NuttX/apps/examples/nxflat/tests/hello++/hello++3.cpp \
../NuttX/apps/examples/nxflat/tests/hello++/hello++4.cpp 

OBJS += \
./NuttX/apps/examples/nxflat/tests/hello++/hello++1.o \
./NuttX/apps/examples/nxflat/tests/hello++/hello++2.o \
./NuttX/apps/examples/nxflat/tests/hello++/hello++3.o \
./NuttX/apps/examples/nxflat/tests/hello++/hello++4.o 

CPP_DEPS += \
./NuttX/apps/examples/nxflat/tests/hello++/hello++1.d \
./NuttX/apps/examples/nxflat/tests/hello++/hello++2.d \
./NuttX/apps/examples/nxflat/tests/hello++/hello++3.d \
./NuttX/apps/examples/nxflat/tests/hello++/hello++4.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nxflat/tests/hello++/%.o: ../NuttX/apps/examples/nxflat/tests/hello++/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


