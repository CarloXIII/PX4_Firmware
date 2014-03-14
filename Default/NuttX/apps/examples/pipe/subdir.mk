################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/pipe/interlock_test.c \
../NuttX/apps/examples/pipe/pipe_main.c \
../NuttX/apps/examples/pipe/redirect_test.c \
../NuttX/apps/examples/pipe/transfer_test.c 

OBJS += \
./NuttX/apps/examples/pipe/interlock_test.o \
./NuttX/apps/examples/pipe/pipe_main.o \
./NuttX/apps/examples/pipe/redirect_test.o \
./NuttX/apps/examples/pipe/transfer_test.o 

C_DEPS += \
./NuttX/apps/examples/pipe/interlock_test.d \
./NuttX/apps/examples/pipe/pipe_main.d \
./NuttX/apps/examples/pipe/redirect_test.d \
./NuttX/apps/examples/pipe/transfer_test.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/pipe/%.o: ../NuttX/apps/examples/pipe/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


