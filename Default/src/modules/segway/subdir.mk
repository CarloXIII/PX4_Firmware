################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/segway/BlockSegwayController.cpp \
../src/modules/segway/segway_main.cpp 

C_SRCS += \
../src/modules/segway/params.c 

OBJS += \
./src/modules/segway/BlockSegwayController.o \
./src/modules/segway/params.o \
./src/modules/segway/segway_main.o 

C_DEPS += \
./src/modules/segway/params.d 

CPP_DEPS += \
./src/modules/segway/BlockSegwayController.d \
./src/modules/segway/segway_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/segway/%.o: ../src/modules/segway/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/segway/%.o: ../src/modules/segway/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


