################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/systemlib/bson/tinybson.c 

OBJS += \
./src/modules/systemlib/bson/tinybson.o 

C_DEPS += \
./src/modules/systemlib/bson/tinybson.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/systemlib/bson/%.o: ../src/modules/systemlib/bson/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


