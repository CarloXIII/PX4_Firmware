################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/power/battery.c \
../NuttX/nuttx/drivers/power/max1704x.c \
../NuttX/nuttx/drivers/power/pm_activity.c \
../NuttX/nuttx/drivers/power/pm_changestate.c \
../NuttX/nuttx/drivers/power/pm_checkstate.c \
../NuttX/nuttx/drivers/power/pm_initialize.c \
../NuttX/nuttx/drivers/power/pm_register.c \
../NuttX/nuttx/drivers/power/pm_update.c 

OBJS += \
./NuttX/nuttx/drivers/power/battery.o \
./NuttX/nuttx/drivers/power/max1704x.o \
./NuttX/nuttx/drivers/power/pm_activity.o \
./NuttX/nuttx/drivers/power/pm_changestate.o \
./NuttX/nuttx/drivers/power/pm_checkstate.o \
./NuttX/nuttx/drivers/power/pm_initialize.o \
./NuttX/nuttx/drivers/power/pm_register.o \
./NuttX/nuttx/drivers/power/pm_update.o 

C_DEPS += \
./NuttX/nuttx/drivers/power/battery.d \
./NuttX/nuttx/drivers/power/max1704x.d \
./NuttX/nuttx/drivers/power/pm_activity.d \
./NuttX/nuttx/drivers/power/pm_changestate.d \
./NuttX/nuttx/drivers/power/pm_checkstate.d \
./NuttX/nuttx/drivers/power/pm_initialize.d \
./NuttX/nuttx/drivers/power/pm_register.d \
./NuttX/nuttx/drivers/power/pm_update.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/power/%.o: ../NuttX/nuttx/drivers/power/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


