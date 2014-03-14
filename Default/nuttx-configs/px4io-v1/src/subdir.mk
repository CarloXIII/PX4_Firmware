################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../nuttx-configs/px4io-v1/src/empty.c 

OBJS += \
./nuttx-configs/px4io-v1/src/empty.o 

C_DEPS += \
./nuttx-configs/px4io-v1/src/empty.d 


# Each subdirectory must supply rules for building sources it contributes
nuttx-configs/px4io-v1/src/%.o: ../nuttx-configs/px4io-v1/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


