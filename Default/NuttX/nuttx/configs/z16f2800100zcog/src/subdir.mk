################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/z16f2800100zcog/src/z16f_leds.c \
../NuttX/nuttx/configs/z16f2800100zcog/src/z16f_lowinit.c 

OBJS += \
./NuttX/nuttx/configs/z16f2800100zcog/src/z16f_leds.o \
./NuttX/nuttx/configs/z16f2800100zcog/src/z16f_lowinit.o 

C_DEPS += \
./NuttX/nuttx/configs/z16f2800100zcog/src/z16f_leds.d \
./NuttX/nuttx/configs/z16f2800100zcog/src/z16f_lowinit.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/z16f2800100zcog/src/%.o: ../NuttX/nuttx/configs/z16f2800100zcog/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


