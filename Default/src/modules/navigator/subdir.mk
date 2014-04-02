################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/navigator/navigator_main.cpp 

C_SRCS += \
../src/modules/navigator/navigator_params.c 

OBJS += \
./src/modules/navigator/navigator_main.o \
./src/modules/navigator/navigator_params.o 

C_DEPS += \
./src/modules/navigator/navigator_params.d 

CPP_DEPS += \
./src/modules/navigator/navigator_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/navigator/%.o: ../src/modules/navigator/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/navigator/%.o: ../src/modules/navigator/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


