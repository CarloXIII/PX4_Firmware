################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/mavlink/mavlink_receiver.cpp 

C_SRCS += \
../src/modules/mavlink/mavlink.c \
../src/modules/mavlink/mavlink_parameters.c \
../src/modules/mavlink/missionlib.c \
../src/modules/mavlink/orb_listener.c \
../src/modules/mavlink/waypoints.c 

OBJS += \
./src/modules/mavlink/mavlink.o \
./src/modules/mavlink/mavlink_parameters.o \
./src/modules/mavlink/mavlink_receiver.o \
./src/modules/mavlink/missionlib.o \
./src/modules/mavlink/orb_listener.o \
./src/modules/mavlink/waypoints.o 

C_DEPS += \
./src/modules/mavlink/mavlink.d \
./src/modules/mavlink/mavlink_parameters.d \
./src/modules/mavlink/missionlib.d \
./src/modules/mavlink/orb_listener.d \
./src/modules/mavlink/waypoints.d 

CPP_DEPS += \
./src/modules/mavlink/mavlink_receiver.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mavlink/%.o: ../src/modules/mavlink/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/mavlink/%.o: ../src/modules/mavlink/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


