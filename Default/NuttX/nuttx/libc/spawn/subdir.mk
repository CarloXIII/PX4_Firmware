################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/spawn/lib_psa_dump.c \
../NuttX/nuttx/libc/spawn/lib_psa_getflags.c \
../NuttX/nuttx/libc/spawn/lib_psa_getschedparam.c \
../NuttX/nuttx/libc/spawn/lib_psa_getschedpolicy.c \
../NuttX/nuttx/libc/spawn/lib_psa_getsigmask.c \
../NuttX/nuttx/libc/spawn/lib_psa_getstacksize.c \
../NuttX/nuttx/libc/spawn/lib_psa_init.c \
../NuttX/nuttx/libc/spawn/lib_psa_setflags.c \
../NuttX/nuttx/libc/spawn/lib_psa_setschedparam.c \
../NuttX/nuttx/libc/spawn/lib_psa_setschedpolicy.c \
../NuttX/nuttx/libc/spawn/lib_psa_setsigmask.c \
../NuttX/nuttx/libc/spawn/lib_psa_setstacksize.c \
../NuttX/nuttx/libc/spawn/lib_psfa_addaction.c \
../NuttX/nuttx/libc/spawn/lib_psfa_addclose.c \
../NuttX/nuttx/libc/spawn/lib_psfa_adddup2.c \
../NuttX/nuttx/libc/spawn/lib_psfa_addopen.c \
../NuttX/nuttx/libc/spawn/lib_psfa_destroy.c \
../NuttX/nuttx/libc/spawn/lib_psfa_dump.c \
../NuttX/nuttx/libc/spawn/lib_psfa_init.c 

OBJS += \
./NuttX/nuttx/libc/spawn/lib_psa_dump.o \
./NuttX/nuttx/libc/spawn/lib_psa_getflags.o \
./NuttX/nuttx/libc/spawn/lib_psa_getschedparam.o \
./NuttX/nuttx/libc/spawn/lib_psa_getschedpolicy.o \
./NuttX/nuttx/libc/spawn/lib_psa_getsigmask.o \
./NuttX/nuttx/libc/spawn/lib_psa_getstacksize.o \
./NuttX/nuttx/libc/spawn/lib_psa_init.o \
./NuttX/nuttx/libc/spawn/lib_psa_setflags.o \
./NuttX/nuttx/libc/spawn/lib_psa_setschedparam.o \
./NuttX/nuttx/libc/spawn/lib_psa_setschedpolicy.o \
./NuttX/nuttx/libc/spawn/lib_psa_setsigmask.o \
./NuttX/nuttx/libc/spawn/lib_psa_setstacksize.o \
./NuttX/nuttx/libc/spawn/lib_psfa_addaction.o \
./NuttX/nuttx/libc/spawn/lib_psfa_addclose.o \
./NuttX/nuttx/libc/spawn/lib_psfa_adddup2.o \
./NuttX/nuttx/libc/spawn/lib_psfa_addopen.o \
./NuttX/nuttx/libc/spawn/lib_psfa_destroy.o \
./NuttX/nuttx/libc/spawn/lib_psfa_dump.o \
./NuttX/nuttx/libc/spawn/lib_psfa_init.o 

C_DEPS += \
./NuttX/nuttx/libc/spawn/lib_psa_dump.d \
./NuttX/nuttx/libc/spawn/lib_psa_getflags.d \
./NuttX/nuttx/libc/spawn/lib_psa_getschedparam.d \
./NuttX/nuttx/libc/spawn/lib_psa_getschedpolicy.d \
./NuttX/nuttx/libc/spawn/lib_psa_getsigmask.d \
./NuttX/nuttx/libc/spawn/lib_psa_getstacksize.d \
./NuttX/nuttx/libc/spawn/lib_psa_init.d \
./NuttX/nuttx/libc/spawn/lib_psa_setflags.d \
./NuttX/nuttx/libc/spawn/lib_psa_setschedparam.d \
./NuttX/nuttx/libc/spawn/lib_psa_setschedpolicy.d \
./NuttX/nuttx/libc/spawn/lib_psa_setsigmask.d \
./NuttX/nuttx/libc/spawn/lib_psa_setstacksize.d \
./NuttX/nuttx/libc/spawn/lib_psfa_addaction.d \
./NuttX/nuttx/libc/spawn/lib_psfa_addclose.d \
./NuttX/nuttx/libc/spawn/lib_psfa_adddup2.d \
./NuttX/nuttx/libc/spawn/lib_psfa_addopen.d \
./NuttX/nuttx/libc/spawn/lib_psfa_destroy.d \
./NuttX/nuttx/libc/spawn/lib_psfa_dump.d \
./NuttX/nuttx/libc/spawn/lib_psfa_init.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/spawn/%.o: ../NuttX/nuttx/libc/spawn/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


