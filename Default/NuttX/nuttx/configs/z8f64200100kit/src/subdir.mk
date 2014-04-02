################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/z8f64200100kit/src/z8_leds.c \
../NuttX/nuttx/configs/z8f64200100kit/src/z8_lowinit.c 

OBJS += \
./NuttX/nuttx/configs/z8f64200100kit/src/z8_leds.o \
./NuttX/nuttx/configs/z8f64200100kit/src/z8_lowinit.o 

C_DEPS += \
./NuttX/nuttx/configs/z8f64200100kit/src/z8_leds.d \
./NuttX/nuttx/configs/z8f64200100kit/src/z8_lowinit.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/z8f64200100kit/src/%.o: ../NuttX/nuttx/configs/z8f64200100kit/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


