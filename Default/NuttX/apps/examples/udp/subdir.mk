################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/udp/host.c \
../NuttX/apps/examples/udp/target.c \
../NuttX/apps/examples/udp/udp-client.c \
../NuttX/apps/examples/udp/udp-server.c 

OBJS += \
./NuttX/apps/examples/udp/host.o \
./NuttX/apps/examples/udp/target.o \
./NuttX/apps/examples/udp/udp-client.o \
./NuttX/apps/examples/udp/udp-server.o 

C_DEPS += \
./NuttX/apps/examples/udp/host.d \
./NuttX/apps/examples/udp/target.d \
./NuttX/apps/examples/udp/udp-client.d \
./NuttX/apps/examples/udp/udp-server.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/udp/%.o: ../NuttX/apps/examples/udp/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


