################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/sched/sched_getprioritymax.c \
../NuttX/nuttx/libc/sched/sched_getprioritymin.c \
../NuttX/nuttx/libc/sched/task_startup.c 

OBJS += \
./NuttX/nuttx/libc/sched/sched_getprioritymax.o \
./NuttX/nuttx/libc/sched/sched_getprioritymin.o \
./NuttX/nuttx/libc/sched/task_startup.o 

C_DEPS += \
./NuttX/nuttx/libc/sched/sched_getprioritymax.d \
./NuttX/nuttx/libc/sched/sched_getprioritymin.d \
./NuttX/nuttx/libc/sched/task_startup.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/sched/%.o: ../NuttX/nuttx/libc/sched/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


