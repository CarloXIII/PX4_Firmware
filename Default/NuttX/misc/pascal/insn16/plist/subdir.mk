################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/insn16/plist/plist.c 

OBJS += \
./NuttX/misc/pascal/insn16/plist/plist.o 

C_DEPS += \
./NuttX/misc/pascal/insn16/plist/plist.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/insn16/plist/%.o: ../NuttX/misc/pascal/insn16/plist/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


