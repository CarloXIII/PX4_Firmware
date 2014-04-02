################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/sims/z80sim/src/main.c 

OBJS += \
./NuttX/misc/sims/z80sim/src/main.o 

C_DEPS += \
./NuttX/misc/sims/z80sim/src/main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/sims/z80sim/src/%.o: ../NuttX/misc/sims/z80sim/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


