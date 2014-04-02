################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/tools/kconfig-frontends/frontends/gconf/gconf.c 

OBJS += \
./NuttX/misc/tools/kconfig-frontends/frontends/gconf/gconf.o 

C_DEPS += \
./NuttX/misc/tools/kconfig-frontends/frontends/gconf/gconf.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/tools/kconfig-frontends/frontends/gconf/%.o: ../NuttX/misc/tools/kconfig-frontends/frontends/gconf/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


