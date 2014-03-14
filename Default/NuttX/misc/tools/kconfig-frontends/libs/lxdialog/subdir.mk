################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/checklist.c \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/inputbox.c \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/menubox.c \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/textbox.c \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/util.c \
../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/yesno.c 

OBJS += \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/checklist.o \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/inputbox.o \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/menubox.o \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/textbox.o \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/util.o \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/yesno.o 

C_DEPS += \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/checklist.d \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/inputbox.d \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/menubox.d \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/textbox.d \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/util.d \
./NuttX/misc/tools/kconfig-frontends/libs/lxdialog/yesno.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/tools/kconfig-frontends/libs/lxdialog/%.o: ../NuttX/misc/tools/kconfig-frontends/libs/lxdialog/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


