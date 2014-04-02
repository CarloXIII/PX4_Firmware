################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/tools/osmocon/crc16.c \
../NuttX/misc/tools/osmocon/msgb.c \
../NuttX/misc/tools/osmocon/osmocon.c \
../NuttX/misc/tools/osmocon/osmoload.c \
../NuttX/misc/tools/osmocon/panic.c \
../NuttX/misc/tools/osmocon/rbtree.c \
../NuttX/misc/tools/osmocon/select.c \
../NuttX/misc/tools/osmocon/sercomm.c \
../NuttX/misc/tools/osmocon/serial.c \
../NuttX/misc/tools/osmocon/talloc.c \
../NuttX/misc/tools/osmocon/timer.c \
../NuttX/misc/tools/osmocon/tpu_debug.c 

OBJS += \
./NuttX/misc/tools/osmocon/crc16.o \
./NuttX/misc/tools/osmocon/msgb.o \
./NuttX/misc/tools/osmocon/osmocon.o \
./NuttX/misc/tools/osmocon/osmoload.o \
./NuttX/misc/tools/osmocon/panic.o \
./NuttX/misc/tools/osmocon/rbtree.o \
./NuttX/misc/tools/osmocon/select.o \
./NuttX/misc/tools/osmocon/sercomm.o \
./NuttX/misc/tools/osmocon/serial.o \
./NuttX/misc/tools/osmocon/talloc.o \
./NuttX/misc/tools/osmocon/timer.o \
./NuttX/misc/tools/osmocon/tpu_debug.o 

C_DEPS += \
./NuttX/misc/tools/osmocon/crc16.d \
./NuttX/misc/tools/osmocon/msgb.d \
./NuttX/misc/tools/osmocon/osmocon.d \
./NuttX/misc/tools/osmocon/osmoload.d \
./NuttX/misc/tools/osmocon/panic.d \
./NuttX/misc/tools/osmocon/rbtree.d \
./NuttX/misc/tools/osmocon/select.d \
./NuttX/misc/tools/osmocon/sercomm.d \
./NuttX/misc/tools/osmocon/serial.d \
./NuttX/misc/tools/osmocon/talloc.d \
./NuttX/misc/tools/osmocon/timer.d \
./NuttX/misc/tools/osmocon/tpu_debug.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/tools/osmocon/%.o: ../NuttX/misc/tools/osmocon/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


