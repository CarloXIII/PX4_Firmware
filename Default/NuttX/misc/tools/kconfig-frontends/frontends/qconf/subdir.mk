################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../NuttX/misc/tools/kconfig-frontends/frontends/qconf/qconf.cc 

OBJS += \
./NuttX/misc/tools/kconfig-frontends/frontends/qconf/qconf.o 

CC_DEPS += \
./NuttX/misc/tools/kconfig-frontends/frontends/qconf/qconf.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/tools/kconfig-frontends/frontends/qconf/%.o: ../NuttX/misc/tools/kconfig-frontends/frontends/qconf/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


