################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/libpas/pextension.c \
../NuttX/misc/pascal/libpas/psignextend16.c \
../NuttX/misc/pascal/libpas/pswap.c 

OBJS += \
./NuttX/misc/pascal/libpas/pextension.o \
./NuttX/misc/pascal/libpas/psignextend16.o \
./NuttX/misc/pascal/libpas/pswap.o 

C_DEPS += \
./NuttX/misc/pascal/libpas/pextension.d \
./NuttX/misc/pascal/libpas/psignextend16.d \
./NuttX/misc/pascal/libpas/pswap.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/libpas/%.o: ../NuttX/misc/pascal/libpas/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


