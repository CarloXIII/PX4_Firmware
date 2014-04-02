################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/pipes/fifo.c \
../NuttX/nuttx/drivers/pipes/pipe.c \
../NuttX/nuttx/drivers/pipes/pipe_common.c 

OBJS += \
./NuttX/nuttx/drivers/pipes/fifo.o \
./NuttX/nuttx/drivers/pipes/pipe.o \
./NuttX/nuttx/drivers/pipes/pipe_common.o 

C_DEPS += \
./NuttX/nuttx/drivers/pipes/fifo.d \
./NuttX/nuttx/drivers/pipes/pipe.d \
./NuttX/nuttx/drivers/pipes/pipe_common.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/pipes/%.o: ../NuttX/nuttx/drivers/pipes/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


