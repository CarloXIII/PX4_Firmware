################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/ostest/barrier.c \
../NuttX/apps/examples/ostest/cancel.c \
../NuttX/apps/examples/ostest/cond.c \
../NuttX/apps/examples/ostest/dev_null.c \
../NuttX/apps/examples/ostest/fpu.c \
../NuttX/apps/examples/ostest/mqueue.c \
../NuttX/apps/examples/ostest/mutex.c \
../NuttX/apps/examples/ostest/ostest_main.c \
../NuttX/apps/examples/ostest/posixtimer.c \
../NuttX/apps/examples/ostest/prioinherit.c \
../NuttX/apps/examples/ostest/restart.c \
../NuttX/apps/examples/ostest/rmutex.c \
../NuttX/apps/examples/ostest/roundrobin.c \
../NuttX/apps/examples/ostest/sem.c \
../NuttX/apps/examples/ostest/sighand.c \
../NuttX/apps/examples/ostest/timedmqueue.c \
../NuttX/apps/examples/ostest/timedwait.c \
../NuttX/apps/examples/ostest/vfork.c \
../NuttX/apps/examples/ostest/waitpid.c 

OBJS += \
./NuttX/apps/examples/ostest/barrier.o \
./NuttX/apps/examples/ostest/cancel.o \
./NuttX/apps/examples/ostest/cond.o \
./NuttX/apps/examples/ostest/dev_null.o \
./NuttX/apps/examples/ostest/fpu.o \
./NuttX/apps/examples/ostest/mqueue.o \
./NuttX/apps/examples/ostest/mutex.o \
./NuttX/apps/examples/ostest/ostest_main.o \
./NuttX/apps/examples/ostest/posixtimer.o \
./NuttX/apps/examples/ostest/prioinherit.o \
./NuttX/apps/examples/ostest/restart.o \
./NuttX/apps/examples/ostest/rmutex.o \
./NuttX/apps/examples/ostest/roundrobin.o \
./NuttX/apps/examples/ostest/sem.o \
./NuttX/apps/examples/ostest/sighand.o \
./NuttX/apps/examples/ostest/timedmqueue.o \
./NuttX/apps/examples/ostest/timedwait.o \
./NuttX/apps/examples/ostest/vfork.o \
./NuttX/apps/examples/ostest/waitpid.o 

C_DEPS += \
./NuttX/apps/examples/ostest/barrier.d \
./NuttX/apps/examples/ostest/cancel.d \
./NuttX/apps/examples/ostest/cond.d \
./NuttX/apps/examples/ostest/dev_null.d \
./NuttX/apps/examples/ostest/fpu.d \
./NuttX/apps/examples/ostest/mqueue.d \
./NuttX/apps/examples/ostest/mutex.d \
./NuttX/apps/examples/ostest/ostest_main.d \
./NuttX/apps/examples/ostest/posixtimer.d \
./NuttX/apps/examples/ostest/prioinherit.d \
./NuttX/apps/examples/ostest/restart.d \
./NuttX/apps/examples/ostest/rmutex.d \
./NuttX/apps/examples/ostest/roundrobin.d \
./NuttX/apps/examples/ostest/sem.d \
./NuttX/apps/examples/ostest/sighand.d \
./NuttX/apps/examples/ostest/timedmqueue.d \
./NuttX/apps/examples/ostest/timedwait.d \
./NuttX/apps/examples/ostest/vfork.d \
./NuttX/apps/examples/ostest/waitpid.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/ostest/%.o: ../NuttX/apps/examples/ostest/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


