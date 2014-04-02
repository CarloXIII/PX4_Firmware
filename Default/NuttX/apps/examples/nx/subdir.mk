################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nx/nx_events.c \
../NuttX/apps/examples/nx/nx_kbdin.c \
../NuttX/apps/examples/nx/nx_main.c \
../NuttX/apps/examples/nx/nx_server.c 

OBJS += \
./NuttX/apps/examples/nx/nx_events.o \
./NuttX/apps/examples/nx/nx_kbdin.o \
./NuttX/apps/examples/nx/nx_main.o \
./NuttX/apps/examples/nx/nx_server.o 

C_DEPS += \
./NuttX/apps/examples/nx/nx_events.d \
./NuttX/apps/examples/nx/nx_kbdin.d \
./NuttX/apps/examples/nx/nx_main.d \
./NuttX/apps/examples/nx/nx_server.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nx/%.o: ../NuttX/apps/examples/nx/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


