################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/controllib/uorb/UOrbPublication.cpp \
../src/modules/controllib/uorb/UOrbSubscription.cpp \
../src/modules/controllib/uorb/blocks.cpp 

OBJS += \
./src/modules/controllib/uorb/UOrbPublication.o \
./src/modules/controllib/uorb/UOrbSubscription.o \
./src/modules/controllib/uorb/blocks.o 

CPP_DEPS += \
./src/modules/controllib/uorb/UOrbPublication.d \
./src/modules/controllib/uorb/UOrbSubscription.d \
./src/modules/controllib/uorb/blocks.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/controllib/uorb/%.o: ../src/modules/controllib/uorb/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


