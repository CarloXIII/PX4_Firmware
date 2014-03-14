################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/system/i2c/i2c_bus.c \
../NuttX/apps/system/i2c/i2c_common.c \
../NuttX/apps/system/i2c/i2c_dev.c \
../NuttX/apps/system/i2c/i2c_get.c \
../NuttX/apps/system/i2c/i2c_main.c \
../NuttX/apps/system/i2c/i2c_set.c \
../NuttX/apps/system/i2c/i2c_verf.c 

OBJS += \
./NuttX/apps/system/i2c/i2c_bus.o \
./NuttX/apps/system/i2c/i2c_common.o \
./NuttX/apps/system/i2c/i2c_dev.o \
./NuttX/apps/system/i2c/i2c_get.o \
./NuttX/apps/system/i2c/i2c_main.o \
./NuttX/apps/system/i2c/i2c_set.o \
./NuttX/apps/system/i2c/i2c_verf.o 

C_DEPS += \
./NuttX/apps/system/i2c/i2c_bus.d \
./NuttX/apps/system/i2c/i2c_common.d \
./NuttX/apps/system/i2c/i2c_dev.d \
./NuttX/apps/system/i2c/i2c_get.d \
./NuttX/apps/system/i2c/i2c_main.d \
./NuttX/apps/system/i2c/i2c_set.d \
./NuttX/apps/system/i2c/i2c_verf.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/system/i2c/%.o: ../NuttX/apps/system/i2c/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


