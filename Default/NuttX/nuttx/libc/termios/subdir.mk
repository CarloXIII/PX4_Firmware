################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/termios/lib_cfgetspeed.c \
../NuttX/nuttx/libc/termios/lib_cfsetspeed.c \
../NuttX/nuttx/libc/termios/lib_tcflush.c \
../NuttX/nuttx/libc/termios/lib_tcgetattr.c \
../NuttX/nuttx/libc/termios/lib_tcsetattr.c 

OBJS += \
./NuttX/nuttx/libc/termios/lib_cfgetspeed.o \
./NuttX/nuttx/libc/termios/lib_cfsetspeed.o \
./NuttX/nuttx/libc/termios/lib_tcflush.o \
./NuttX/nuttx/libc/termios/lib_tcgetattr.o \
./NuttX/nuttx/libc/termios/lib_tcsetattr.o 

C_DEPS += \
./NuttX/nuttx/libc/termios/lib_cfgetspeed.d \
./NuttX/nuttx/libc/termios/lib_cfsetspeed.d \
./NuttX/nuttx/libc/termios/lib_tcflush.d \
./NuttX/nuttx/libc/termios/lib_tcgetattr.d \
./NuttX/nuttx/libc/termios/lib_tcsetattr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/termios/%.o: ../NuttX/nuttx/libc/termios/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


