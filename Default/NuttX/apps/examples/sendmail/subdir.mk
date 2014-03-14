################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/sendmail/host.c \
../NuttX/apps/examples/sendmail/target.c 

OBJS += \
./NuttX/apps/examples/sendmail/host.o \
./NuttX/apps/examples/sendmail/target.o 

C_DEPS += \
./NuttX/apps/examples/sendmail/host.d \
./NuttX/apps/examples/sendmail/target.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/sendmail/%.o: ../NuttX/apps/examples/sendmail/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


