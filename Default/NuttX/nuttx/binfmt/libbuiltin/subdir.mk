################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/binfmt/libbuiltin/libbuiltin_getname.c \
../NuttX/nuttx/binfmt/libbuiltin/libbuiltin_isavail.c 

OBJS += \
./NuttX/nuttx/binfmt/libbuiltin/libbuiltin_getname.o \
./NuttX/nuttx/binfmt/libbuiltin/libbuiltin_isavail.o 

C_DEPS += \
./NuttX/nuttx/binfmt/libbuiltin/libbuiltin_getname.d \
./NuttX/nuttx/binfmt/libbuiltin/libbuiltin_isavail.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/binfmt/libbuiltin/%.o: ../NuttX/nuttx/binfmt/libbuiltin/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


