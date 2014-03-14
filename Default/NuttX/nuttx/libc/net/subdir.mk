################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/libc/net/lib_etherntoa.c \
../NuttX/nuttx/libc/net/lib_htonl.c \
../NuttX/nuttx/libc/net/lib_htons.c \
../NuttX/nuttx/libc/net/lib_inetaddr.c \
../NuttX/nuttx/libc/net/lib_inetntoa.c \
../NuttX/nuttx/libc/net/lib_inetntop.c \
../NuttX/nuttx/libc/net/lib_inetpton.c 

OBJS += \
./NuttX/nuttx/libc/net/lib_etherntoa.o \
./NuttX/nuttx/libc/net/lib_htonl.o \
./NuttX/nuttx/libc/net/lib_htons.o \
./NuttX/nuttx/libc/net/lib_inetaddr.o \
./NuttX/nuttx/libc/net/lib_inetntoa.o \
./NuttX/nuttx/libc/net/lib_inetntop.o \
./NuttX/nuttx/libc/net/lib_inetpton.o 

C_DEPS += \
./NuttX/nuttx/libc/net/lib_etherntoa.d \
./NuttX/nuttx/libc/net/lib_htonl.d \
./NuttX/nuttx/libc/net/lib_htons.d \
./NuttX/nuttx/libc/net/lib_inetaddr.d \
./NuttX/nuttx/libc/net/lib_inetntoa.d \
./NuttX/nuttx/libc/net/lib_inetntop.d \
./NuttX/nuttx/libc/net/lib_inetpton.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/libc/net/%.o: ../NuttX/nuttx/libc/net/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


