################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/fixedwing_backside/fixedwing.cpp \
../src/modules/fixedwing_backside/fixedwing_backside_main.cpp 

C_SRCS += \
../src/modules/fixedwing_backside/params.c 

OBJS += \
./src/modules/fixedwing_backside/fixedwing.o \
./src/modules/fixedwing_backside/fixedwing_backside_main.o \
./src/modules/fixedwing_backside/params.o 

C_DEPS += \
./src/modules/fixedwing_backside/params.d 

CPP_DEPS += \
./src/modules/fixedwing_backside/fixedwing.d \
./src/modules/fixedwing_backside/fixedwing_backside_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/fixedwing_backside/%.o: ../src/modules/fixedwing_backside/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/fixedwing_backside/%.o: ../src/modules/fixedwing_backside/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


