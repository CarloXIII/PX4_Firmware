################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/poll/host.c \
../NuttX/apps/examples/poll/net_listener.c \
../NuttX/apps/examples/poll/net_reader.c \
../NuttX/apps/examples/poll/poll_listener.c \
../NuttX/apps/examples/poll/poll_main.c \
../NuttX/apps/examples/poll/select_listener.c 

OBJS += \
./NuttX/apps/examples/poll/host.o \
./NuttX/apps/examples/poll/net_listener.o \
./NuttX/apps/examples/poll/net_reader.o \
./NuttX/apps/examples/poll/poll_listener.o \
./NuttX/apps/examples/poll/poll_main.o \
./NuttX/apps/examples/poll/select_listener.o 

C_DEPS += \
./NuttX/apps/examples/poll/host.d \
./NuttX/apps/examples/poll/net_listener.d \
./NuttX/apps/examples/poll/net_reader.d \
./NuttX/apps/examples/poll/poll_listener.d \
./NuttX/apps/examples/poll/poll_main.d \
./NuttX/apps/examples/poll/select_listener.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/poll/%.o: ../NuttX/apps/examples/poll/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


