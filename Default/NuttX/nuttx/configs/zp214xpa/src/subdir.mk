################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/zp214xpa/src/up_spi1.c \
../NuttX/nuttx/configs/zp214xpa/src/up_ug2864ambag01.c 

OBJS += \
./NuttX/nuttx/configs/zp214xpa/src/up_spi1.o \
./NuttX/nuttx/configs/zp214xpa/src/up_ug2864ambag01.o 

C_DEPS += \
./NuttX/nuttx/configs/zp214xpa/src/up_spi1.d \
./NuttX/nuttx/configs/zp214xpa/src/up_ug2864ambag01.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/zp214xpa/src/%.o: ../NuttX/nuttx/configs/zp214xpa/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


