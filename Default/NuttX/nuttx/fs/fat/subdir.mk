################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/fs/fat/fs_configfat.c \
../NuttX/nuttx/fs/fat/fs_fat32.c \
../NuttX/nuttx/fs/fat/fs_fat32attrib.c \
../NuttX/nuttx/fs/fat/fs_fat32dirent.c \
../NuttX/nuttx/fs/fat/fs_fat32util.c \
../NuttX/nuttx/fs/fat/fs_mkfatfs.c \
../NuttX/nuttx/fs/fat/fs_writefat.c 

OBJS += \
./NuttX/nuttx/fs/fat/fs_configfat.o \
./NuttX/nuttx/fs/fat/fs_fat32.o \
./NuttX/nuttx/fs/fat/fs_fat32attrib.o \
./NuttX/nuttx/fs/fat/fs_fat32dirent.o \
./NuttX/nuttx/fs/fat/fs_fat32util.o \
./NuttX/nuttx/fs/fat/fs_mkfatfs.o \
./NuttX/nuttx/fs/fat/fs_writefat.o 

C_DEPS += \
./NuttX/nuttx/fs/fat/fs_configfat.d \
./NuttX/nuttx/fs/fat/fs_fat32.d \
./NuttX/nuttx/fs/fat/fs_fat32attrib.d \
./NuttX/nuttx/fs/fat/fs_fat32dirent.d \
./NuttX/nuttx/fs/fat/fs_fat32util.d \
./NuttX/nuttx/fs/fat/fs_mkfatfs.d \
./NuttX/nuttx/fs/fat/fs_writefat.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/fs/fat/%.o: ../NuttX/nuttx/fs/fat/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


