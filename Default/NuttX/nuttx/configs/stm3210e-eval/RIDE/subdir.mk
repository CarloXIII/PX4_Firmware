################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/stm3210e-eval/RIDE/bigfatstub.c 

OBJS += \
./NuttX/nuttx/configs/stm3210e-eval/RIDE/bigfatstub.o 

C_DEPS += \
./NuttX/nuttx/configs/stm3210e-eval/RIDE/bigfatstub.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/stm3210e-eval/RIDE/%.o: ../NuttX/nuttx/configs/stm3210e-eval/RIDE/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


