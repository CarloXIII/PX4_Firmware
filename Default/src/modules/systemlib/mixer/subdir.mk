################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/modules/systemlib/mixer/mixer.cpp \
../src/modules/systemlib/mixer/mixer_group.cpp \
../src/modules/systemlib/mixer/mixer_multirotor.cpp \
../src/modules/systemlib/mixer/mixer_simple.cpp 

C_SRCS += \
../src/modules/systemlib/mixer/mixer_load.c 

OBJS += \
./src/modules/systemlib/mixer/mixer.o \
./src/modules/systemlib/mixer/mixer_group.o \
./src/modules/systemlib/mixer/mixer_load.o \
./src/modules/systemlib/mixer/mixer_multirotor.o \
./src/modules/systemlib/mixer/mixer_simple.o 

C_DEPS += \
./src/modules/systemlib/mixer/mixer_load.d 

CPP_DEPS += \
./src/modules/systemlib/mixer/mixer.d \
./src/modules/systemlib/mixer/mixer_group.d \
./src/modules/systemlib/mixer/mixer_multirotor.d \
./src/modules/systemlib/mixer/mixer_simple.d 


# Each subdirectory must supply rules for building sources it contributes
src/modules/systemlib/mixer/%.o: ../src/modules/systemlib/mixer/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/modules/systemlib/mixer/%.o: ../src/modules/systemlib/mixer/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


