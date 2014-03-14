################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/mount/mount_main.c \
../NuttX/apps/examples/mount/ramdisk.c 

OBJS += \
./NuttX/apps/examples/mount/mount_main.o \
./NuttX/apps/examples/mount/ramdisk.o 

C_DEPS += \
./NuttX/apps/examples/mount/mount_main.d \
./NuttX/apps/examples/mount/ramdisk.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/mount/%.o: ../NuttX/apps/examples/mount/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


