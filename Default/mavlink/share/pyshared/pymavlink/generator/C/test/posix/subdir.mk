################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../mavlink/share/pyshared/pymavlink/generator/C/test/posix/testmav.c 

OBJS += \
./mavlink/share/pyshared/pymavlink/generator/C/test/posix/testmav.o 

C_DEPS += \
./mavlink/share/pyshared/pymavlink/generator/C/test/posix/testmav.d 


# Each subdirectory must supply rules for building sources it contributes
mavlink/share/pyshared/pymavlink/generator/C/test/posix/%.o: ../mavlink/share/pyshared/pymavlink/generator/C/test/posix/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


