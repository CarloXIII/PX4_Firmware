################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../mavlink/share/pyshared/pymavlink/generator/C/src_v1.0/pixhawk/pixhawk.pb.cc 

OBJS += \
./mavlink/share/pyshared/pymavlink/generator/C/src_v1.0/pixhawk/pixhawk.pb.o 

CC_DEPS += \
./mavlink/share/pyshared/pymavlink/generator/C/src_v1.0/pixhawk/pixhawk.pb.d 


# Each subdirectory must supply rules for building sources it contributes
mavlink/share/pyshared/pymavlink/generator/C/src_v1.0/pixhawk/%.o: ../mavlink/share/pyshared/pymavlink/generator/C/src_v1.0/pixhawk/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


