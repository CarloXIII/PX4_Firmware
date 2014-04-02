################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/mmap/fs_mmap.c \
../NuttX/nuttx/fs/mmap/fs_munmap.c \
../NuttX/nuttx/fs/mmap/fs_rammap.c 

OBJS += \
./NuttX/nuttx/fs/mmap/fs_mmap.o \
./NuttX/nuttx/fs/mmap/fs_munmap.o \
./NuttX/nuttx/fs/mmap/fs_rammap.o 

C_DEPS += \
./NuttX/nuttx/fs/mmap/fs_mmap.d \
./NuttX/nuttx/fs/mmap/fs_munmap.d \
./NuttX/nuttx/fs/mmap/fs_rammap.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/mmap/%.o: ../NuttX/nuttx/fs/mmap/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


