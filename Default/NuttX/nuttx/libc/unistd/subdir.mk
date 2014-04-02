################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/unistd/lib_chdir.c \
../NuttX/nuttx/libc/unistd/lib_execl.c \
../NuttX/nuttx/libc/unistd/lib_execsymtab.c \
../NuttX/nuttx/libc/unistd/lib_execv.c \
../NuttX/nuttx/libc/unistd/lib_getcwd.c \
../NuttX/nuttx/libc/unistd/lib_getopt.c \
../NuttX/nuttx/libc/unistd/lib_getoptargp.c \
../NuttX/nuttx/libc/unistd/lib_getoptindp.c \
../NuttX/nuttx/libc/unistd/lib_getoptoptp.c 

OBJS += \
./NuttX/nuttx/libc/unistd/lib_chdir.o \
./NuttX/nuttx/libc/unistd/lib_execl.o \
./NuttX/nuttx/libc/unistd/lib_execsymtab.o \
./NuttX/nuttx/libc/unistd/lib_execv.o \
./NuttX/nuttx/libc/unistd/lib_getcwd.o \
./NuttX/nuttx/libc/unistd/lib_getopt.o \
./NuttX/nuttx/libc/unistd/lib_getoptargp.o \
./NuttX/nuttx/libc/unistd/lib_getoptindp.o \
./NuttX/nuttx/libc/unistd/lib_getoptoptp.o 

C_DEPS += \
./NuttX/nuttx/libc/unistd/lib_chdir.d \
./NuttX/nuttx/libc/unistd/lib_execl.d \
./NuttX/nuttx/libc/unistd/lib_execsymtab.d \
./NuttX/nuttx/libc/unistd/lib_execv.d \
./NuttX/nuttx/libc/unistd/lib_getcwd.d \
./NuttX/nuttx/libc/unistd/lib_getopt.d \
./NuttX/nuttx/libc/unistd/lib_getoptargp.d \
./NuttX/nuttx/libc/unistd/lib_getoptindp.d \
./NuttX/nuttx/libc/unistd/lib_getoptoptp.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/unistd/%.o: ../NuttX/nuttx/libc/unistd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


