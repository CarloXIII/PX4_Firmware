################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_autoleds.c \
../NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_boot.c \
../NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_nsh.c \
../NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_ssi.c 

OBJS += \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_autoleds.o \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_boot.o \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_nsh.o \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_ssi.o 

C_DEPS += \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_autoleds.d \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_boot.d \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_nsh.d \
./NuttX/nuttx/configs/lm4f120-launchpad/src/lm4f_ssi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/lm4f120-launchpad/src/%.o: ../NuttX/nuttx/configs/lm4f120-launchpad/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


