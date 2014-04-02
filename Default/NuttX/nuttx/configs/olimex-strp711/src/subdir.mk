################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/olimex-strp711/src/up_buttons.c \
../NuttX/nuttx/configs/olimex-strp711/src/up_enc28j60.c \
../NuttX/nuttx/configs/olimex-strp711/src/up_leds.c \
../NuttX/nuttx/configs/olimex-strp711/src/up_nsh.c \
../NuttX/nuttx/configs/olimex-strp711/src/up_spi.c 

OBJS += \
./NuttX/nuttx/configs/olimex-strp711/src/up_buttons.o \
./NuttX/nuttx/configs/olimex-strp711/src/up_enc28j60.o \
./NuttX/nuttx/configs/olimex-strp711/src/up_leds.o \
./NuttX/nuttx/configs/olimex-strp711/src/up_nsh.o \
./NuttX/nuttx/configs/olimex-strp711/src/up_spi.o 

C_DEPS += \
./NuttX/nuttx/configs/olimex-strp711/src/up_buttons.d \
./NuttX/nuttx/configs/olimex-strp711/src/up_enc28j60.d \
./NuttX/nuttx/configs/olimex-strp711/src/up_leds.d \
./NuttX/nuttx/configs/olimex-strp711/src/up_nsh.d \
./NuttX/nuttx/configs/olimex-strp711/src/up_spi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/olimex-strp711/src/%.o: ../NuttX/nuttx/configs/olimex-strp711/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


