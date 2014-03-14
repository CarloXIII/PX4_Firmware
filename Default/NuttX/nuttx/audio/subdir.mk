################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/audio/audio.c \
../NuttX/nuttx/audio/buffer.c \
../NuttX/nuttx/audio/pcm.c 

OBJS += \
./NuttX/nuttx/audio/audio.o \
./NuttX/nuttx/audio/buffer.o \
./NuttX/nuttx/audio/pcm.o 

C_DEPS += \
./NuttX/nuttx/audio/audio.d \
./NuttX/nuttx/audio/buffer.d \
./NuttX/nuttx/audio/pcm.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/audio/%.o: ../NuttX/nuttx/audio/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


