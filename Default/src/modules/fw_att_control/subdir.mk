################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/fw_att_control/fw_att_control_main.cpp 

C_SRCS += \
../src/modules/fw_att_control/fw_att_control_params.c 

OBJS += \
./src/modules/fw_att_control/fw_att_control_main.o \
./src/modules/fw_att_control/fw_att_control_params.o 

C_DEPS += \
./src/modules/fw_att_control/fw_att_control_params.d 

CPP_DEPS += \
./src/modules/fw_att_control/fw_att_control_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fw_att_control/%.o: ../src/modules/fw_att_control/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/fw_att_control/%.o: ../src/modules/fw_att_control/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


