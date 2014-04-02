################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/mavlink_onboard/mavlink.c \
../src/modules/mavlink_onboard/mavlink_receiver.c 

OBJS += \
./src/modules/mavlink_onboard/mavlink.o \
./src/modules/mavlink_onboard/mavlink_receiver.o 

C_DEPS += \
./src/modules/mavlink_onboard/mavlink.d \
./src/modules/mavlink_onboard/mavlink_receiver.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/mavlink_onboard/%.o: ../src/modules/mavlink_onboard/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


