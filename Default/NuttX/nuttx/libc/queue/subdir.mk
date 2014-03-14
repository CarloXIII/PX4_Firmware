################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/queue/dq_addafter.c \
../NuttX/nuttx/libc/queue/dq_addbefore.c \
../NuttX/nuttx/libc/queue/dq_addfirst.c \
../NuttX/nuttx/libc/queue/dq_addlast.c \
../NuttX/nuttx/libc/queue/dq_rem.c \
../NuttX/nuttx/libc/queue/dq_remfirst.c \
../NuttX/nuttx/libc/queue/dq_remlast.c \
../NuttX/nuttx/libc/queue/sq_addafter.c \
../NuttX/nuttx/libc/queue/sq_addfirst.c \
../NuttX/nuttx/libc/queue/sq_addlast.c \
../NuttX/nuttx/libc/queue/sq_rem.c \
../NuttX/nuttx/libc/queue/sq_remafter.c \
../NuttX/nuttx/libc/queue/sq_remfirst.c \
../NuttX/nuttx/libc/queue/sq_remlast.c 

OBJS += \
./NuttX/nuttx/libc/queue/dq_addafter.o \
./NuttX/nuttx/libc/queue/dq_addbefore.o \
./NuttX/nuttx/libc/queue/dq_addfirst.o \
./NuttX/nuttx/libc/queue/dq_addlast.o \
./NuttX/nuttx/libc/queue/dq_rem.o \
./NuttX/nuttx/libc/queue/dq_remfirst.o \
./NuttX/nuttx/libc/queue/dq_remlast.o \
./NuttX/nuttx/libc/queue/sq_addafter.o \
./NuttX/nuttx/libc/queue/sq_addfirst.o \
./NuttX/nuttx/libc/queue/sq_addlast.o \
./NuttX/nuttx/libc/queue/sq_rem.o \
./NuttX/nuttx/libc/queue/sq_remafter.o \
./NuttX/nuttx/libc/queue/sq_remfirst.o \
./NuttX/nuttx/libc/queue/sq_remlast.o 

C_DEPS += \
./NuttX/nuttx/libc/queue/dq_addafter.d \
./NuttX/nuttx/libc/queue/dq_addbefore.d \
./NuttX/nuttx/libc/queue/dq_addfirst.d \
./NuttX/nuttx/libc/queue/dq_addlast.d \
./NuttX/nuttx/libc/queue/dq_rem.d \
./NuttX/nuttx/libc/queue/dq_remfirst.d \
./NuttX/nuttx/libc/queue/dq_remlast.d \
./NuttX/nuttx/libc/queue/sq_addafter.d \
./NuttX/nuttx/libc/queue/sq_addfirst.d \
./NuttX/nuttx/libc/queue/sq_addlast.d \
./NuttX/nuttx/libc/queue/sq_rem.d \
./NuttX/nuttx/libc/queue/sq_remafter.d \
./NuttX/nuttx/libc/queue/sq_remfirst.d \
./NuttX/nuttx/libc/queue/sq_remlast.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/queue/%.o: ../NuttX/nuttx/libc/queue/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


