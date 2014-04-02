################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/sdlog3/sdlog3.c 

OBJS += \
./src/modules/sdlog3/sdlog3.o 

C_DEPS += \
./src/modules/sdlog3/sdlog3.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/sdlog3/%.o: ../src/modules/sdlog3/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


