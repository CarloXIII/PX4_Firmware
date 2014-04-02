################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/misc/lib_crc32.c \
../NuttX/nuttx/libc/misc/lib_dbg.c \
../NuttX/nuttx/libc/misc/lib_dumpbuffer.c \
../NuttX/nuttx/libc/misc/lib_filesem.c \
../NuttX/nuttx/libc/misc/lib_init.c \
../NuttX/nuttx/libc/misc/lib_kbddecode.c \
../NuttX/nuttx/libc/misc/lib_kbdencode.c \
../NuttX/nuttx/libc/misc/lib_match.c \
../NuttX/nuttx/libc/misc/lib_sendfile.c \
../NuttX/nuttx/libc/misc/lib_slcddecode.c \
../NuttX/nuttx/libc/misc/lib_slcdencode.c \
../NuttX/nuttx/libc/misc/lib_streamsem.c 

OBJS += \
./NuttX/nuttx/libc/misc/lib_crc32.o \
./NuttX/nuttx/libc/misc/lib_dbg.o \
./NuttX/nuttx/libc/misc/lib_dumpbuffer.o \
./NuttX/nuttx/libc/misc/lib_filesem.o \
./NuttX/nuttx/libc/misc/lib_init.o \
./NuttX/nuttx/libc/misc/lib_kbddecode.o \
./NuttX/nuttx/libc/misc/lib_kbdencode.o \
./NuttX/nuttx/libc/misc/lib_match.o \
./NuttX/nuttx/libc/misc/lib_sendfile.o \
./NuttX/nuttx/libc/misc/lib_slcddecode.o \
./NuttX/nuttx/libc/misc/lib_slcdencode.o \
./NuttX/nuttx/libc/misc/lib_streamsem.o 

C_DEPS += \
./NuttX/nuttx/libc/misc/lib_crc32.d \
./NuttX/nuttx/libc/misc/lib_dbg.d \
./NuttX/nuttx/libc/misc/lib_dumpbuffer.d \
./NuttX/nuttx/libc/misc/lib_filesem.d \
./NuttX/nuttx/libc/misc/lib_init.d \
./NuttX/nuttx/libc/misc/lib_kbddecode.d \
./NuttX/nuttx/libc/misc/lib_kbdencode.d \
./NuttX/nuttx/libc/misc/lib_match.d \
./NuttX/nuttx/libc/misc/lib_sendfile.d \
./NuttX/nuttx/libc/misc/lib_slcddecode.d \
./NuttX/nuttx/libc/misc/lib_slcdencode.d \
./NuttX/nuttx/libc/misc/lib_streamsem.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/misc/%.o: ../NuttX/nuttx/libc/misc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


