################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/mmcsd/mmcsd_debug.c \
../NuttX/nuttx/drivers/mmcsd/mmcsd_sdio.c \
../NuttX/nuttx/drivers/mmcsd/mmcsd_spi.c 

OBJS += \
./NuttX/nuttx/drivers/mmcsd/mmcsd_debug.o \
./NuttX/nuttx/drivers/mmcsd/mmcsd_sdio.o \
./NuttX/nuttx/drivers/mmcsd/mmcsd_spi.o 

C_DEPS += \
./NuttX/nuttx/drivers/mmcsd/mmcsd_debug.d \
./NuttX/nuttx/drivers/mmcsd/mmcsd_sdio.d \
./NuttX/nuttx/drivers/mmcsd/mmcsd_spi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/mmcsd/%.o: ../NuttX/nuttx/drivers/mmcsd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


