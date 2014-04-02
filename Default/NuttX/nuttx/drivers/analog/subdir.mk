################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/analog/ad5410.c \
../NuttX/nuttx/drivers/analog/adc.c \
../NuttX/nuttx/drivers/analog/ads1255.c \
../NuttX/nuttx/drivers/analog/dac.c \
../NuttX/nuttx/drivers/analog/pga11x.c 

OBJS += \
./NuttX/nuttx/drivers/analog/ad5410.o \
./NuttX/nuttx/drivers/analog/adc.o \
./NuttX/nuttx/drivers/analog/ads1255.o \
./NuttX/nuttx/drivers/analog/dac.o \
./NuttX/nuttx/drivers/analog/pga11x.o 

C_DEPS += \
./NuttX/nuttx/drivers/analog/ad5410.d \
./NuttX/nuttx/drivers/analog/adc.d \
./NuttX/nuttx/drivers/analog/ads1255.d \
./NuttX/nuttx/drivers/analog/dac.d \
./NuttX/nuttx/drivers/analog/pga11x.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/analog/%.o: ../NuttX/nuttx/drivers/analog/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


