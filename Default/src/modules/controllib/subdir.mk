################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/controllib/blocks.cpp 

C_SRCS += \
../src/modules/controllib/test_params.c 

OBJS += \
./src/modules/controllib/blocks.o \
./src/modules/controllib/test_params.o 

C_DEPS += \
./src/modules/controllib/test_params.d 

CPP_DEPS += \
./src/modules/controllib/blocks.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/controllib/%.o: ../src/modules/controllib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/controllib/%.o: ../src/modules/controllib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


