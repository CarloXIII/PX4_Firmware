################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/mqueue/mq_getattr.c \
../NuttX/nuttx/libc/mqueue/mq_setattr.c 

OBJS += \
./NuttX/nuttx/libc/mqueue/mq_getattr.o \
./NuttX/nuttx/libc/mqueue/mq_setattr.o 

C_DEPS += \
./NuttX/nuttx/libc/mqueue/mq_getattr.d \
./NuttX/nuttx/libc/mqueue/mq_setattr.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/mqueue/%.o: ../NuttX/nuttx/libc/mqueue/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


