################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/stdlib/lib_abort.c \
../NuttX/nuttx/libc/stdlib/lib_abs.c \
../NuttX/nuttx/libc/stdlib/lib_imaxabs.c \
../NuttX/nuttx/libc/stdlib/lib_itoa.c \
../NuttX/nuttx/libc/stdlib/lib_labs.c \
../NuttX/nuttx/libc/stdlib/lib_llabs.c \
../NuttX/nuttx/libc/stdlib/lib_qsort.c \
../NuttX/nuttx/libc/stdlib/lib_rand.c 

OBJS += \
./NuttX/nuttx/libc/stdlib/lib_abort.o \
./NuttX/nuttx/libc/stdlib/lib_abs.o \
./NuttX/nuttx/libc/stdlib/lib_imaxabs.o \
./NuttX/nuttx/libc/stdlib/lib_itoa.o \
./NuttX/nuttx/libc/stdlib/lib_labs.o \
./NuttX/nuttx/libc/stdlib/lib_llabs.o \
./NuttX/nuttx/libc/stdlib/lib_qsort.o \
./NuttX/nuttx/libc/stdlib/lib_rand.o 

C_DEPS += \
./NuttX/nuttx/libc/stdlib/lib_abort.d \
./NuttX/nuttx/libc/stdlib/lib_abs.d \
./NuttX/nuttx/libc/stdlib/lib_imaxabs.d \
./NuttX/nuttx/libc/stdlib/lib_itoa.d \
./NuttX/nuttx/libc/stdlib/lib_labs.d \
./NuttX/nuttx/libc/stdlib/lib_llabs.d \
./NuttX/nuttx/libc/stdlib/lib_qsort.d \
./NuttX/nuttx/libc/stdlib/lib_rand.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/stdlib/%.o: ../NuttX/nuttx/libc/stdlib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


