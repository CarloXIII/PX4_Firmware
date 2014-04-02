################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/examples/fixedwing_control/main.c \
../src/examples/fixedwing_control/params.c 

OBJS += \
./src/examples/fixedwing_control/main.o \
./src/examples/fixedwing_control/params.o 

C_DEPS += \
./src/examples/fixedwing_control/main.d \
./src/examples/fixedwing_control/params.d 


# Each subdirectory must supply rules for building sources it contributes
src/examples/fixedwing_control/%.o: ../src/examples/fixedwing_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


