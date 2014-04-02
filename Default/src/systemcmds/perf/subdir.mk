################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/perf/perf.c 

OBJS += \
./src/systemcmds/perf/perf.o 

C_DEPS += \
./src/systemcmds/perf/perf.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/perf/%.o: ../src/systemcmds/perf/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


