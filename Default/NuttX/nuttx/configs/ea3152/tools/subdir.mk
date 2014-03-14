################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/ea3152/tools/crc32.c \
../NuttX/nuttx/configs/ea3152/tools/lpchdr.c 

OBJS += \
./NuttX/nuttx/configs/ea3152/tools/crc32.o \
./NuttX/nuttx/configs/ea3152/tools/lpchdr.o 

C_DEPS += \
./NuttX/nuttx/configs/ea3152/tools/crc32.d \
./NuttX/nuttx/configs/ea3152/tools/lpchdr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/ea3152/tools/%.o: ../NuttX/nuttx/configs/ea3152/tools/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


