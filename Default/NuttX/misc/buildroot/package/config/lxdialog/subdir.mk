################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/buildroot/package/config/lxdialog/checklist.c \
../NuttX/misc/buildroot/package/config/lxdialog/inputbox.c \
../NuttX/misc/buildroot/package/config/lxdialog/menubox.c \
../NuttX/misc/buildroot/package/config/lxdialog/msgbox.c \
../NuttX/misc/buildroot/package/config/lxdialog/textbox.c \
../NuttX/misc/buildroot/package/config/lxdialog/util.c \
../NuttX/misc/buildroot/package/config/lxdialog/yesno.c 

OBJS += \
./NuttX/misc/buildroot/package/config/lxdialog/checklist.o \
./NuttX/misc/buildroot/package/config/lxdialog/inputbox.o \
./NuttX/misc/buildroot/package/config/lxdialog/menubox.o \
./NuttX/misc/buildroot/package/config/lxdialog/msgbox.o \
./NuttX/misc/buildroot/package/config/lxdialog/textbox.o \
./NuttX/misc/buildroot/package/config/lxdialog/util.o \
./NuttX/misc/buildroot/package/config/lxdialog/yesno.o 

C_DEPS += \
./NuttX/misc/buildroot/package/config/lxdialog/checklist.d \
./NuttX/misc/buildroot/package/config/lxdialog/inputbox.d \
./NuttX/misc/buildroot/package/config/lxdialog/menubox.d \
./NuttX/misc/buildroot/package/config/lxdialog/msgbox.d \
./NuttX/misc/buildroot/package/config/lxdialog/textbox.d \
./NuttX/misc/buildroot/package/config/lxdialog/util.d \
./NuttX/misc/buildroot/package/config/lxdialog/yesno.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/buildroot/package/config/lxdialog/%.o: ../NuttX/misc/buildroot/package/config/lxdialog/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


