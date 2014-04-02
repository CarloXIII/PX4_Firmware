################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/wireless/ISM1_868MHzGFSK100kbps.c \
../NuttX/nuttx/drivers/wireless/ISM2_905MHzGFSK250kbps.c \
../NuttX/nuttx/drivers/wireless/cc1101.c \
../NuttX/nuttx/drivers/wireless/nrf24l01.c 

OBJS += \
./NuttX/nuttx/drivers/wireless/ISM1_868MHzGFSK100kbps.o \
./NuttX/nuttx/drivers/wireless/ISM2_905MHzGFSK250kbps.o \
./NuttX/nuttx/drivers/wireless/cc1101.o \
./NuttX/nuttx/drivers/wireless/nrf24l01.o 

C_DEPS += \
./NuttX/nuttx/drivers/wireless/ISM1_868MHzGFSK100kbps.d \
./NuttX/nuttx/drivers/wireless/ISM2_905MHzGFSK250kbps.d \
./NuttX/nuttx/drivers/wireless/cc1101.d \
./NuttX/nuttx/drivers/wireless/nrf24l01.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/wireless/%.o: ../NuttX/nuttx/drivers/wireless/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


