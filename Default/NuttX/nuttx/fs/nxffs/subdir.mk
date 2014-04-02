################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/nxffs/nxffs_block.c \
../NuttX/nuttx/fs/nxffs/nxffs_blockstats.c \
../NuttX/nuttx/fs/nxffs/nxffs_cache.c \
../NuttX/nuttx/fs/nxffs/nxffs_dirent.c \
../NuttX/nuttx/fs/nxffs/nxffs_dump.c \
../NuttX/nuttx/fs/nxffs/nxffs_initialize.c \
../NuttX/nuttx/fs/nxffs/nxffs_inode.c \
../NuttX/nuttx/fs/nxffs/nxffs_ioctl.c \
../NuttX/nuttx/fs/nxffs/nxffs_open.c \
../NuttX/nuttx/fs/nxffs/nxffs_pack.c \
../NuttX/nuttx/fs/nxffs/nxffs_read.c \
../NuttX/nuttx/fs/nxffs/nxffs_reformat.c \
../NuttX/nuttx/fs/nxffs/nxffs_stat.c \
../NuttX/nuttx/fs/nxffs/nxffs_unlink.c \
../NuttX/nuttx/fs/nxffs/nxffs_util.c \
../NuttX/nuttx/fs/nxffs/nxffs_write.c 

OBJS += \
./NuttX/nuttx/fs/nxffs/nxffs_block.o \
./NuttX/nuttx/fs/nxffs/nxffs_blockstats.o \
./NuttX/nuttx/fs/nxffs/nxffs_cache.o \
./NuttX/nuttx/fs/nxffs/nxffs_dirent.o \
./NuttX/nuttx/fs/nxffs/nxffs_dump.o \
./NuttX/nuttx/fs/nxffs/nxffs_initialize.o \
./NuttX/nuttx/fs/nxffs/nxffs_inode.o \
./NuttX/nuttx/fs/nxffs/nxffs_ioctl.o \
./NuttX/nuttx/fs/nxffs/nxffs_open.o \
./NuttX/nuttx/fs/nxffs/nxffs_pack.o \
./NuttX/nuttx/fs/nxffs/nxffs_read.o \
./NuttX/nuttx/fs/nxffs/nxffs_reformat.o \
./NuttX/nuttx/fs/nxffs/nxffs_stat.o \
./NuttX/nuttx/fs/nxffs/nxffs_unlink.o \
./NuttX/nuttx/fs/nxffs/nxffs_util.o \
./NuttX/nuttx/fs/nxffs/nxffs_write.o 

C_DEPS += \
./NuttX/nuttx/fs/nxffs/nxffs_block.d \
./NuttX/nuttx/fs/nxffs/nxffs_blockstats.d \
./NuttX/nuttx/fs/nxffs/nxffs_cache.d \
./NuttX/nuttx/fs/nxffs/nxffs_dirent.d \
./NuttX/nuttx/fs/nxffs/nxffs_dump.d \
./NuttX/nuttx/fs/nxffs/nxffs_initialize.d \
./NuttX/nuttx/fs/nxffs/nxffs_inode.d \
./NuttX/nuttx/fs/nxffs/nxffs_ioctl.d \
./NuttX/nuttx/fs/nxffs/nxffs_open.d \
./NuttX/nuttx/fs/nxffs/nxffs_pack.d \
./NuttX/nuttx/fs/nxffs/nxffs_read.d \
./NuttX/nuttx/fs/nxffs/nxffs_reformat.d \
./NuttX/nuttx/fs/nxffs/nxffs_stat.d \
./NuttX/nuttx/fs/nxffs/nxffs_unlink.d \
./NuttX/nuttx/fs/nxffs/nxffs_util.d \
./NuttX/nuttx/fs/nxffs/nxffs_write.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/nxffs/%.o: ../NuttX/nuttx/fs/nxffs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


