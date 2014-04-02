################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nxconsole/nxcon_main.c \
../NuttX/apps/examples/nxconsole/nxcon_server.c \
../NuttX/apps/examples/nxconsole/nxcon_toolbar.c \
../NuttX/apps/examples/nxconsole/nxcon_wndo.c 

OBJS += \
./NuttX/apps/examples/nxconsole/nxcon_main.o \
./NuttX/apps/examples/nxconsole/nxcon_server.o \
./NuttX/apps/examples/nxconsole/nxcon_toolbar.o \
./NuttX/apps/examples/nxconsole/nxcon_wndo.o 

C_DEPS += \
./NuttX/apps/examples/nxconsole/nxcon_main.d \
./NuttX/apps/examples/nxconsole/nxcon_server.d \
./NuttX/apps/examples/nxconsole/nxcon_toolbar.d \
./NuttX/apps/examples/nxconsole/nxcon_wndo.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nxconsole/%.o: ../NuttX/apps/examples/nxconsole/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


