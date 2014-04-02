################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/wqueue/work_cancel.c \
../NuttX/nuttx/libc/wqueue/work_queue.c \
../NuttX/nuttx/libc/wqueue/work_signal.c \
../NuttX/nuttx/libc/wqueue/work_thread.c \
../NuttX/nuttx/libc/wqueue/work_usrstart.c 

OBJS += \
./NuttX/nuttx/libc/wqueue/work_cancel.o \
./NuttX/nuttx/libc/wqueue/work_queue.o \
./NuttX/nuttx/libc/wqueue/work_signal.o \
./NuttX/nuttx/libc/wqueue/work_thread.o \
./NuttX/nuttx/libc/wqueue/work_usrstart.o 

C_DEPS += \
./NuttX/nuttx/libc/wqueue/work_cancel.d \
./NuttX/nuttx/libc/wqueue/work_queue.d \
./NuttX/nuttx/libc/wqueue/work_signal.d \
./NuttX/nuttx/libc/wqueue/work_thread.d \
./NuttX/nuttx/libc/wqueue/work_usrstart.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/wqueue/%.o: ../NuttX/nuttx/libc/wqueue/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


