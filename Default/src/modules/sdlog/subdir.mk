################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/modules/sdlog/sdlog.c \
../src/modules/sdlog/sdlog_ringbuffer.c 

OBJS += \
./src/modules/sdlog/sdlog.o \
./src/modules/sdlog/sdlog_ringbuffer.o 

C_DEPS += \
./src/modules/sdlog/sdlog.d \
./src/modules/sdlog/sdlog_ringbuffer.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/sdlog/%.o: ../src/modules/sdlog/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


