################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_boot.c \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_ethernet.c \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_leds.c \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_nsh.c \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_oled.c \
../NuttX/nuttx/configs/lm3s8962-ek/src/up_ssi.c 

OBJS += \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_boot.o \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_ethernet.o \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_leds.o \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_nsh.o \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_oled.o \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_ssi.o 

C_DEPS += \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_boot.d \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_ethernet.d \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_leds.d \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_nsh.d \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_oled.d \
./NuttX/nuttx/configs/lm3s8962-ek/src/up_ssi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/lm3s8962-ek/src/%.o: ../NuttX/nuttx/configs/lm3s8962-ek/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


