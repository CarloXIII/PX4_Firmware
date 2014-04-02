################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/binfs/fs_binfs.c 

OBJS += \
./NuttX/nuttx/fs/binfs/fs_binfs.o 

C_DEPS += \
./NuttX/nuttx/fs/binfs/fs_binfs.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/binfs/%.o: ../NuttX/nuttx/fs/binfs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


