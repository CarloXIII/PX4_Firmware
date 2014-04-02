################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/signal/sig_addset.c \
../NuttX/nuttx/libc/signal/sig_delset.c \
../NuttX/nuttx/libc/signal/sig_emptyset.c \
../NuttX/nuttx/libc/signal/sig_fillset.c \
../NuttX/nuttx/libc/signal/sig_ismember.c 

OBJS += \
./NuttX/nuttx/libc/signal/sig_addset.o \
./NuttX/nuttx/libc/signal/sig_delset.o \
./NuttX/nuttx/libc/signal/sig_emptyset.o \
./NuttX/nuttx/libc/signal/sig_fillset.o \
./NuttX/nuttx/libc/signal/sig_ismember.o 

C_DEPS += \
./NuttX/nuttx/libc/signal/sig_addset.d \
./NuttX/nuttx/libc/signal/sig_delset.d \
./NuttX/nuttx/libc/signal/sig_emptyset.d \
./NuttX/nuttx/libc/signal/sig_fillset.d \
./NuttX/nuttx/libc/signal/sig_ismember.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/signal/%.o: ../NuttX/nuttx/libc/signal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


