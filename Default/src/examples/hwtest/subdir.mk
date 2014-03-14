################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/examples/hwtest/hwtest.c 

OBJS += \
./src/examples/hwtest/hwtest.o 

C_DEPS += \
./src/examples/hwtest/hwtest.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/hwtest/%.o: ../src/examples/hwtest/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


