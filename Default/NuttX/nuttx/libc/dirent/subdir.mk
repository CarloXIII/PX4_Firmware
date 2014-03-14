################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/dirent/lib_readdirr.c \
../NuttX/nuttx/libc/dirent/lib_telldir.c 

OBJS += \
./NuttX/nuttx/libc/dirent/lib_readdirr.o \
./NuttX/nuttx/libc/dirent/lib_telldir.o 

C_DEPS += \
./NuttX/nuttx/libc/dirent/lib_readdirr.d \
./NuttX/nuttx/libc/dirent/lib_telldir.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/dirent/%.o: ../NuttX/nuttx/libc/dirent/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


