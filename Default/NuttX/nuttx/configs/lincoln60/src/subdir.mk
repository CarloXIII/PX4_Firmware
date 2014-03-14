################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/lincoln60/src/up_boot.c \
../NuttX/nuttx/configs/lincoln60/src/up_buttons.c \
../NuttX/nuttx/configs/lincoln60/src/up_leds.c \
../NuttX/nuttx/configs/lincoln60/src/up_nsh.c 

OBJS += \
./NuttX/nuttx/configs/lincoln60/src/up_boot.o \
./NuttX/nuttx/configs/lincoln60/src/up_buttons.o \
./NuttX/nuttx/configs/lincoln60/src/up_leds.o \
./NuttX/nuttx/configs/lincoln60/src/up_nsh.o 

C_DEPS += \
./NuttX/nuttx/configs/lincoln60/src/up_boot.d \
./NuttX/nuttx/configs/lincoln60/src/up_buttons.d \
./NuttX/nuttx/configs/lincoln60/src/up_leds.d \
./NuttX/nuttx/configs/lincoln60/src/up_nsh.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/lincoln60/src/%.o: ../NuttX/nuttx/configs/lincoln60/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


