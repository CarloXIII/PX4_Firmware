################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/builtin/builtin.c \
../NuttX/apps/builtin/builtin_list.c \
../NuttX/apps/builtin/exec_builtin.c 

OBJS += \
./NuttX/apps/builtin/builtin.o \
./NuttX/apps/builtin/builtin_list.o \
./NuttX/apps/builtin/exec_builtin.o 

C_DEPS += \
./NuttX/apps/builtin/builtin.d \
./NuttX/apps/builtin/builtin_list.d \
./NuttX/apps/builtin/exec_builtin.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/builtin/%.o: ../NuttX/apps/builtin/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


