################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/input/ads7843e.c \
../NuttX/nuttx/drivers/input/max11802.c \
../NuttX/nuttx/drivers/input/stmpe811_adc.c \
../NuttX/nuttx/drivers/input/stmpe811_base.c \
../NuttX/nuttx/drivers/input/stmpe811_gpio.c \
../NuttX/nuttx/drivers/input/stmpe811_temp.c \
../NuttX/nuttx/drivers/input/stmpe811_tsc.c \
../NuttX/nuttx/drivers/input/tsc2007.c 

OBJS += \
./NuttX/nuttx/drivers/input/ads7843e.o \
./NuttX/nuttx/drivers/input/max11802.o \
./NuttX/nuttx/drivers/input/stmpe811_adc.o \
./NuttX/nuttx/drivers/input/stmpe811_base.o \
./NuttX/nuttx/drivers/input/stmpe811_gpio.o \
./NuttX/nuttx/drivers/input/stmpe811_temp.o \
./NuttX/nuttx/drivers/input/stmpe811_tsc.o \
./NuttX/nuttx/drivers/input/tsc2007.o 

C_DEPS += \
./NuttX/nuttx/drivers/input/ads7843e.d \
./NuttX/nuttx/drivers/input/max11802.d \
./NuttX/nuttx/drivers/input/stmpe811_adc.d \
./NuttX/nuttx/drivers/input/stmpe811_base.d \
./NuttX/nuttx/drivers/input/stmpe811_gpio.d \
./NuttX/nuttx/drivers/input/stmpe811_temp.d \
./NuttX/nuttx/drivers/input/stmpe811_tsc.d \
./NuttX/nuttx/drivers/input/tsc2007.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/input/%.o: ../NuttX/nuttx/drivers/input/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


