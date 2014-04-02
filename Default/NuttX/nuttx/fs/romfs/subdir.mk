################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/romfs/fs_romfs.c \
../NuttX/nuttx/fs/romfs/fs_romfsutil.c 

OBJS += \
./NuttX/nuttx/fs/romfs/fs_romfs.o \
./NuttX/nuttx/fs/romfs/fs_romfsutil.o 

C_DEPS += \
./NuttX/nuttx/fs/romfs/fs_romfs.d \
./NuttX/nuttx/fs/romfs/fs_romfsutil.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/romfs/%.o: ../NuttX/nuttx/fs/romfs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


