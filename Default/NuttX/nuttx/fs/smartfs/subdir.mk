################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/smartfs/smartfs_mksmartfs.c \
../NuttX/nuttx/fs/smartfs/smartfs_smart.c \
../NuttX/nuttx/fs/smartfs/smartfs_utils.c 

OBJS += \
./NuttX/nuttx/fs/smartfs/smartfs_mksmartfs.o \
./NuttX/nuttx/fs/smartfs/smartfs_smart.o \
./NuttX/nuttx/fs/smartfs/smartfs_utils.o 

C_DEPS += \
./NuttX/nuttx/fs/smartfs/smartfs_mksmartfs.d \
./NuttX/nuttx/fs/smartfs/smartfs_smart.d \
./NuttX/nuttx/fs/smartfs/smartfs_utils.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/smartfs/%.o: ../NuttX/nuttx/fs/smartfs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


