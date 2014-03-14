################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/tftpc/tftpc_get.c \
../NuttX/apps/netutils/tftpc/tftpc_packets.c \
../NuttX/apps/netutils/tftpc/tftpc_put.c 

OBJS += \
./NuttX/apps/netutils/tftpc/tftpc_get.o \
./NuttX/apps/netutils/tftpc/tftpc_packets.o \
./NuttX/apps/netutils/tftpc/tftpc_put.o 

C_DEPS += \
./NuttX/apps/netutils/tftpc/tftpc_get.d \
./NuttX/apps/netutils/tftpc/tftpc_packets.d \
./NuttX/apps/netutils/tftpc/tftpc_put.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/tftpc/%.o: ../NuttX/apps/netutils/tftpc/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


