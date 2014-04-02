################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_boot.c \
../NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_lcd1602.c \
../NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_nsh.c 

OBJS += \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_boot.o \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_lcd1602.o \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_nsh.o 

C_DEPS += \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_boot.d \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_lcd1602.d \
./NuttX/nuttx/configs/pcblogic-pic32mx/src/pic32mx_nsh.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/pcblogic-pic32mx/src/%.o: ../NuttX/nuttx/configs/pcblogic-pic32mx/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


