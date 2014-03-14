################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/tools/pic32mx/mkpichex.c 

OBJS += \
./NuttX/nuttx/tools/pic32mx/mkpichex.o 

C_DEPS += \
./NuttX/nuttx/tools/pic32mx/mkpichex.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/tools/pic32mx/%.o: ../NuttX/nuttx/tools/pic32mx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


