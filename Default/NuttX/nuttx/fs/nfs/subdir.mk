################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/nfs/nfs_util.c \
../NuttX/nuttx/fs/nfs/nfs_vfsops.c \
../NuttX/nuttx/fs/nfs/rpc_clnt.c 

OBJS += \
./NuttX/nuttx/fs/nfs/nfs_util.o \
./NuttX/nuttx/fs/nfs/nfs_vfsops.o \
./NuttX/nuttx/fs/nfs/rpc_clnt.o 

C_DEPS += \
./NuttX/nuttx/fs/nfs/nfs_util.d \
./NuttX/nuttx/fs/nfs/nfs_vfsops.d \
./NuttX/nuttx/fs/nfs/rpc_clnt.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/nfs/%.o: ../NuttX/nuttx/fs/nfs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


