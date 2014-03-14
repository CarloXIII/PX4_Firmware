################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/graphics/nxconsole/nx_register.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_driver.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_font.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_kbdin.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_putc.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_redraw.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_register.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_scroll.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_sem.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_unregister.c \
../NuttX/nuttx/graphics/nxconsole/nxcon_vt100.c \
../NuttX/nuttx/graphics/nxconsole/nxtk_register.c \
../NuttX/nuttx/graphics/nxconsole/nxtool_register.c 

OBJS += \
./NuttX/nuttx/graphics/nxconsole/nx_register.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_driver.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_font.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_kbdin.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_putc.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_redraw.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_register.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_scroll.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_sem.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_unregister.o \
./NuttX/nuttx/graphics/nxconsole/nxcon_vt100.o \
./NuttX/nuttx/graphics/nxconsole/nxtk_register.o \
./NuttX/nuttx/graphics/nxconsole/nxtool_register.o 

C_DEPS += \
./NuttX/nuttx/graphics/nxconsole/nx_register.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_driver.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_font.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_kbdin.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_putc.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_redraw.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_register.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_scroll.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_sem.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_unregister.d \
./NuttX/nuttx/graphics/nxconsole/nxcon_vt100.d \
./NuttX/nuttx/graphics/nxconsole/nxtk_register.d \
./NuttX/nuttx/graphics/nxconsole/nxtool_register.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/graphics/nxconsole/%.o: ../NuttX/nuttx/graphics/nxconsole/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


