################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/arch/rgmp/src/bridge.c \
../NuttX/nuttx/arch/rgmp/src/cxx.c \
../NuttX/nuttx/arch/rgmp/src/nuttx.c \
../NuttX/nuttx/arch/rgmp/src/rgmp.c 

OBJS += \
./NuttX/nuttx/arch/rgmp/src/bridge.o \
./NuttX/nuttx/arch/rgmp/src/cxx.o \
./NuttX/nuttx/arch/rgmp/src/nuttx.o \
./NuttX/nuttx/arch/rgmp/src/rgmp.o 

C_DEPS += \
./NuttX/nuttx/arch/rgmp/src/bridge.d \
./NuttX/nuttx/arch/rgmp/src/cxx.d \
./NuttX/nuttx/arch/rgmp/src/nuttx.d \
./NuttX/nuttx/arch/rgmp/src/rgmp.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/arch/rgmp/src/%.o: ../NuttX/nuttx/arch/rgmp/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


