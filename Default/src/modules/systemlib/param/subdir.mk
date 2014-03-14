################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/systemlib/param/param.c 

OBJS += \
./src/modules/systemlib/param/param.o 

C_DEPS += \
./src/modules/systemlib/param/param.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/systemlib/param/%.o: ../src/modules/systemlib/param/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


