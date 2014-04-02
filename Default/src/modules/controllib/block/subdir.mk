################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/controllib/block/Block.cpp \
../src/modules/controllib/block/BlockParam.cpp 

OBJS += \
./src/modules/controllib/block/Block.o \
./src/modules/controllib/block/BlockParam.o 

CPP_DEPS += \
./src/modules/controllib/block/Block.d \
./src/modules/controllib/block/BlockParam.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/controllib/block/%.o: ../src/modules/controllib/block/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


