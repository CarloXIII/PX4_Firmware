################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/mathlib/math/arm/Matrix.cpp \
../src/lib/mathlib/math/arm/Vector.cpp 

OBJS += \
./src/lib/mathlib/math/arm/Matrix.o \
./src/lib/mathlib/math/arm/Vector.o 

CPP_DEPS += \
./src/lib/mathlib/math/arm/Matrix.d \
./src/lib/mathlib/math/arm/Vector.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/mathlib/math/arm/%.o: ../src/lib/mathlib/math/arm/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


