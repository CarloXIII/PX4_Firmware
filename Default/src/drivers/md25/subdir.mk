################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/drivers/md25/BlockSysIdent.cpp \
../src/drivers/md25/md25.cpp \
../src/drivers/md25/md25_main.cpp 

OBJS += \
./src/drivers/md25/BlockSysIdent.o \
./src/drivers/md25/md25.o \
./src/drivers/md25/md25_main.o 

CPP_DEPS += \
./src/drivers/md25/BlockSysIdent.d \
./src/drivers/md25/md25.d \
./src/drivers/md25/md25_main.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/md25/%.o: ../src/drivers/md25/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


