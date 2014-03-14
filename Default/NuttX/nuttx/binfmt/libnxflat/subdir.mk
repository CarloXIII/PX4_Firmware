################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_addrenv.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_bind.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_init.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_load.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_read.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_uninit.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_unload.c \
../NuttX/nuttx/binfmt/libnxflat/libnxflat_verify.c 

OBJS += \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_addrenv.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_bind.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_init.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_load.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_read.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_uninit.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_unload.o \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_verify.o 

C_DEPS += \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_addrenv.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_bind.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_init.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_load.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_read.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_uninit.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_unload.d \
./NuttX/nuttx/binfmt/libnxflat/libnxflat_verify.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/binfmt/libnxflat/%.o: ../NuttX/nuttx/binfmt/libnxflat/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


