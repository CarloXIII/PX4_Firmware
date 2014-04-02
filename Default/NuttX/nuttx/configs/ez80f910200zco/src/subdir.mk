################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/ez80f910200zco/src/ez80_buttons.c \
../NuttX/nuttx/configs/ez80f910200zco/src/ez80_leds.c \
../NuttX/nuttx/configs/ez80f910200zco/src/ez80_lowinit.c 

OBJS += \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_buttons.o \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_leds.o \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_lowinit.o 

C_DEPS += \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_buttons.d \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_leds.d \
./NuttX/nuttx/configs/ez80f910200zco/src/ez80_lowinit.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/ez80f910200zco/src/%.o: ../NuttX/nuttx/configs/ez80f910200zco/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


