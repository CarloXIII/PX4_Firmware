################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/lib/mathlib/math/Dcm.cpp \
../src/lib/mathlib/math/EulerAngles.cpp \
../src/lib/mathlib/math/Limits.cpp \
../src/lib/mathlib/math/Matrix.cpp \
../src/lib/mathlib/math/Quaternion.cpp \
../src/lib/mathlib/math/Vector.cpp \
../src/lib/mathlib/math/Vector2f.cpp \
../src/lib/mathlib/math/Vector3.cpp 

OBJS += \
./src/lib/mathlib/math/Dcm.o \
./src/lib/mathlib/math/EulerAngles.o \
./src/lib/mathlib/math/Limits.o \
./src/lib/mathlib/math/Matrix.o \
./src/lib/mathlib/math/Quaternion.o \
./src/lib/mathlib/math/Vector.o \
./src/lib/mathlib/math/Vector2f.o \
./src/lib/mathlib/math/Vector3.o 

CPP_DEPS += \
./src/lib/mathlib/math/Dcm.d \
./src/lib/mathlib/math/EulerAngles.d \
./src/lib/mathlib/math/Limits.d \
./src/lib/mathlib/math/Matrix.d \
./src/lib/mathlib/math/Quaternion.d \
./src/lib/mathlib/math/Vector.d \
./src/lib/mathlib/math/Vector2f.d \
./src/lib/mathlib/math/Vector3.d 


# Each subdirectory must supply rules for building sources it contributes
src/lib/mathlib/math/%.o: ../src/lib/mathlib/math/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


