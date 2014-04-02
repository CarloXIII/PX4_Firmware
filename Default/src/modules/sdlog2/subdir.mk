################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/sdlog2/logbuffer.c \
../src/modules/sdlog2/sdlog2.c 

OBJS += \
./src/modules/sdlog2/logbuffer.o \
./src/modules/sdlog2/sdlog2.o 

C_DEPS += \
./src/modules/sdlog2/logbuffer.d \
./src/modules/sdlog2/sdlog2.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/sdlog2/%.o: ../src/modules/sdlog2/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


