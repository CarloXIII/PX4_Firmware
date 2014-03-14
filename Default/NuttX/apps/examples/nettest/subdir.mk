################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nettest/host.c \
../NuttX/apps/examples/nettest/nettest.c \
../NuttX/apps/examples/nettest/nettest_client.c \
../NuttX/apps/examples/nettest/nettest_server.c 

OBJS += \
./NuttX/apps/examples/nettest/host.o \
./NuttX/apps/examples/nettest/nettest.o \
./NuttX/apps/examples/nettest/nettest_client.o \
./NuttX/apps/examples/nettest/nettest_server.o 

C_DEPS += \
./NuttX/apps/examples/nettest/host.d \
./NuttX/apps/examples/nettest/nettest.d \
./NuttX/apps/examples/nettest/nettest_client.d \
./NuttX/apps/examples/nettest/nettest_server.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nettest/%.o: ../NuttX/apps/examples/nettest/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


