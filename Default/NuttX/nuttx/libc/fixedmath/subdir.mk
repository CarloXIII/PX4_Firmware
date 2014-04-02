################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/fixedmath/lib_b16atan2.c \
../NuttX/nuttx/libc/fixedmath/lib_b16cos.c \
../NuttX/nuttx/libc/fixedmath/lib_b16sin.c \
../NuttX/nuttx/libc/fixedmath/lib_fixedmath.c \
../NuttX/nuttx/libc/fixedmath/lib_rint.c 

OBJS += \
./NuttX/nuttx/libc/fixedmath/lib_b16atan2.o \
./NuttX/nuttx/libc/fixedmath/lib_b16cos.o \
./NuttX/nuttx/libc/fixedmath/lib_b16sin.o \
./NuttX/nuttx/libc/fixedmath/lib_fixedmath.o \
./NuttX/nuttx/libc/fixedmath/lib_rint.o 

C_DEPS += \
./NuttX/nuttx/libc/fixedmath/lib_b16atan2.d \
./NuttX/nuttx/libc/fixedmath/lib_b16cos.d \
./NuttX/nuttx/libc/fixedmath/lib_b16sin.d \
./NuttX/nuttx/libc/fixedmath/lib_fixedmath.d \
./NuttX/nuttx/libc/fixedmath/lib_rint.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/fixedmath/%.o: ../NuttX/nuttx/libc/fixedmath/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


