################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/libgen/lib_basename.c \
../NuttX/nuttx/libc/libgen/lib_dirname.c 

OBJS += \
./NuttX/nuttx/libc/libgen/lib_basename.o \
./NuttX/nuttx/libc/libgen/lib_dirname.o 

C_DEPS += \
./NuttX/nuttx/libc/libgen/lib_basename.d \
./NuttX/nuttx/libc/libgen/lib_dirname.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/libgen/%.o: ../NuttX/nuttx/libc/libgen/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


