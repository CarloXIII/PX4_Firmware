################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/semaphore/sem_getvalue.c \
../NuttX/nuttx/libc/semaphore/sem_init.c 

OBJS += \
./NuttX/nuttx/libc/semaphore/sem_getvalue.o \
./NuttX/nuttx/libc/semaphore/sem_init.o 

C_DEPS += \
./NuttX/nuttx/libc/semaphore/sem_getvalue.d \
./NuttX/nuttx/libc/semaphore/sem_init.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/semaphore/%.o: ../NuttX/nuttx/libc/semaphore/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


