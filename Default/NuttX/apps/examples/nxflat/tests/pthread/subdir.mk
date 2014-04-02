################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nxflat/tests/pthread/pthread.c 

OBJS += \
./NuttX/apps/examples/nxflat/tests/pthread/pthread.o 

C_DEPS += \
./NuttX/apps/examples/nxflat/tests/pthread/pthread.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nxflat/tests/pthread/%.o: ../NuttX/apps/examples/nxflat/tests/pthread/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


