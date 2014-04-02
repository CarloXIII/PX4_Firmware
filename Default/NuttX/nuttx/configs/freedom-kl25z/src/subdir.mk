################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/freedom-kl25z/src/kl_boardinitialize.c \
../NuttX/nuttx/configs/freedom-kl25z/src/kl_led.c 

OBJS += \
./NuttX/nuttx/configs/freedom-kl25z/src/kl_boardinitialize.o \
./NuttX/nuttx/configs/freedom-kl25z/src/kl_led.o 

C_DEPS += \
./NuttX/nuttx/configs/freedom-kl25z/src/kl_boardinitialize.d \
./NuttX/nuttx/configs/freedom-kl25z/src/kl_led.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/freedom-kl25z/src/%.o: ../NuttX/nuttx/configs/freedom-kl25z/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


