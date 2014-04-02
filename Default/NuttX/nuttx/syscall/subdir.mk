################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/syscall/syscall_clock_systimer.c \
../NuttX/nuttx/syscall/syscall_funclookup.c \
../NuttX/nuttx/syscall/syscall_nparms.c \
../NuttX/nuttx/syscall/syscall_stublookup.c 

OBJS += \
./NuttX/nuttx/syscall/syscall_clock_systimer.o \
./NuttX/nuttx/syscall/syscall_funclookup.o \
./NuttX/nuttx/syscall/syscall_nparms.o \
./NuttX/nuttx/syscall/syscall_stublookup.o 

C_DEPS += \
./NuttX/nuttx/syscall/syscall_clock_systimer.d \
./NuttX/nuttx/syscall/syscall_funclookup.d \
./NuttX/nuttx/syscall/syscall_nparms.d \
./NuttX/nuttx/syscall/syscall_stublookup.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/syscall/%.o: ../NuttX/nuttx/syscall/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


