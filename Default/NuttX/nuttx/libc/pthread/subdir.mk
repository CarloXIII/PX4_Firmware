################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/pthread/pthread_attrdestroy.c \
../NuttX/nuttx/libc/pthread/pthread_attrgetinheritsched.c \
../NuttX/nuttx/libc/pthread/pthread_attrgetschedparam.c \
../NuttX/nuttx/libc/pthread/pthread_attrgetschedpolicy.c \
../NuttX/nuttx/libc/pthread/pthread_attrgetstacksize.c \
../NuttX/nuttx/libc/pthread/pthread_attrinit.c \
../NuttX/nuttx/libc/pthread/pthread_attrsetinheritsched.c \
../NuttX/nuttx/libc/pthread/pthread_attrsetschedparam.c \
../NuttX/nuttx/libc/pthread/pthread_attrsetschedpolicy.c \
../NuttX/nuttx/libc/pthread/pthread_attrsetstacksize.c \
../NuttX/nuttx/libc/pthread/pthread_barrierattrdestroy.c \
../NuttX/nuttx/libc/pthread/pthread_barrierattrgetpshared.c \
../NuttX/nuttx/libc/pthread/pthread_barrierattrinit.c \
../NuttX/nuttx/libc/pthread/pthread_barrierattrsetpshared.c \
../NuttX/nuttx/libc/pthread/pthread_condattrdestroy.c \
../NuttX/nuttx/libc/pthread/pthread_condattrinit.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrdestroy.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrgetpshared.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrgettype.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrinit.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrsetpshared.c \
../NuttX/nuttx/libc/pthread/pthread_mutexattrsettype.c \
../NuttX/nuttx/libc/pthread/pthread_startup.c 

OBJS += \
./NuttX/nuttx/libc/pthread/pthread_attrdestroy.o \
./NuttX/nuttx/libc/pthread/pthread_attrgetinheritsched.o \
./NuttX/nuttx/libc/pthread/pthread_attrgetschedparam.o \
./NuttX/nuttx/libc/pthread/pthread_attrgetschedpolicy.o \
./NuttX/nuttx/libc/pthread/pthread_attrgetstacksize.o \
./NuttX/nuttx/libc/pthread/pthread_attrinit.o \
./NuttX/nuttx/libc/pthread/pthread_attrsetinheritsched.o \
./NuttX/nuttx/libc/pthread/pthread_attrsetschedparam.o \
./NuttX/nuttx/libc/pthread/pthread_attrsetschedpolicy.o \
./NuttX/nuttx/libc/pthread/pthread_attrsetstacksize.o \
./NuttX/nuttx/libc/pthread/pthread_barrierattrdestroy.o \
./NuttX/nuttx/libc/pthread/pthread_barrierattrgetpshared.o \
./NuttX/nuttx/libc/pthread/pthread_barrierattrinit.o \
./NuttX/nuttx/libc/pthread/pthread_barrierattrsetpshared.o \
./NuttX/nuttx/libc/pthread/pthread_condattrdestroy.o \
./NuttX/nuttx/libc/pthread/pthread_condattrinit.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrdestroy.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrgetpshared.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrgettype.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrinit.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrsetpshared.o \
./NuttX/nuttx/libc/pthread/pthread_mutexattrsettype.o \
./NuttX/nuttx/libc/pthread/pthread_startup.o 

C_DEPS += \
./NuttX/nuttx/libc/pthread/pthread_attrdestroy.d \
./NuttX/nuttx/libc/pthread/pthread_attrgetinheritsched.d \
./NuttX/nuttx/libc/pthread/pthread_attrgetschedparam.d \
./NuttX/nuttx/libc/pthread/pthread_attrgetschedpolicy.d \
./NuttX/nuttx/libc/pthread/pthread_attrgetstacksize.d \
./NuttX/nuttx/libc/pthread/pthread_attrinit.d \
./NuttX/nuttx/libc/pthread/pthread_attrsetinheritsched.d \
./NuttX/nuttx/libc/pthread/pthread_attrsetschedparam.d \
./NuttX/nuttx/libc/pthread/pthread_attrsetschedpolicy.d \
./NuttX/nuttx/libc/pthread/pthread_attrsetstacksize.d \
./NuttX/nuttx/libc/pthread/pthread_barrierattrdestroy.d \
./NuttX/nuttx/libc/pthread/pthread_barrierattrgetpshared.d \
./NuttX/nuttx/libc/pthread/pthread_barrierattrinit.d \
./NuttX/nuttx/libc/pthread/pthread_barrierattrsetpshared.d \
./NuttX/nuttx/libc/pthread/pthread_condattrdestroy.d \
./NuttX/nuttx/libc/pthread/pthread_condattrinit.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrdestroy.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrgetpshared.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrgettype.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrinit.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrsetpshared.d \
./NuttX/nuttx/libc/pthread/pthread_mutexattrsettype.d \
./NuttX/nuttx/libc/pthread/pthread_startup.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/pthread/%.o: ../NuttX/nuttx/libc/pthread/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


