################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/buildroot/toolchain/nxflat/ldnxflat.c \
../NuttX/misc/buildroot/toolchain/nxflat/mknxflat.c \
../NuttX/misc/buildroot/toolchain/nxflat/readnxflat.c 

OBJS += \
./NuttX/misc/buildroot/toolchain/nxflat/ldnxflat.o \
./NuttX/misc/buildroot/toolchain/nxflat/mknxflat.o \
./NuttX/misc/buildroot/toolchain/nxflat/readnxflat.o 

C_DEPS += \
./NuttX/misc/buildroot/toolchain/nxflat/ldnxflat.d \
./NuttX/misc/buildroot/toolchain/nxflat/mknxflat.d \
./NuttX/misc/buildroot/toolchain/nxflat/readnxflat.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/buildroot/toolchain/nxflat/%.o: ../NuttX/misc/buildroot/toolchain/nxflat/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


