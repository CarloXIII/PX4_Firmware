################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/compal_e99/src/dummy.c \
../NuttX/nuttx/configs/compal_e99/src/ssd1783.c 

OBJS += \
./NuttX/nuttx/configs/compal_e99/src/dummy.o \
./NuttX/nuttx/configs/compal_e99/src/ssd1783.o 

C_DEPS += \
./NuttX/nuttx/configs/compal_e99/src/dummy.d \
./NuttX/nuttx/configs/compal_e99/src/ssd1783.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/compal_e99/src/%.o: ../NuttX/nuttx/configs/compal_e99/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


