################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/usbhost/hid_parser.c \
../NuttX/nuttx/drivers/usbhost/usbhost_enumerate.c \
../NuttX/nuttx/drivers/usbhost/usbhost_findclass.c \
../NuttX/nuttx/drivers/usbhost/usbhost_hidkbd.c \
../NuttX/nuttx/drivers/usbhost/usbhost_registerclass.c \
../NuttX/nuttx/drivers/usbhost/usbhost_registry.c \
../NuttX/nuttx/drivers/usbhost/usbhost_skeleton.c \
../NuttX/nuttx/drivers/usbhost/usbhost_storage.c 

OBJS += \
./NuttX/nuttx/drivers/usbhost/hid_parser.o \
./NuttX/nuttx/drivers/usbhost/usbhost_enumerate.o \
./NuttX/nuttx/drivers/usbhost/usbhost_findclass.o \
./NuttX/nuttx/drivers/usbhost/usbhost_hidkbd.o \
./NuttX/nuttx/drivers/usbhost/usbhost_registerclass.o \
./NuttX/nuttx/drivers/usbhost/usbhost_registry.o \
./NuttX/nuttx/drivers/usbhost/usbhost_skeleton.o \
./NuttX/nuttx/drivers/usbhost/usbhost_storage.o 

C_DEPS += \
./NuttX/nuttx/drivers/usbhost/hid_parser.d \
./NuttX/nuttx/drivers/usbhost/usbhost_enumerate.d \
./NuttX/nuttx/drivers/usbhost/usbhost_findclass.d \
./NuttX/nuttx/drivers/usbhost/usbhost_hidkbd.d \
./NuttX/nuttx/drivers/usbhost/usbhost_registerclass.d \
./NuttX/nuttx/drivers/usbhost/usbhost_registry.d \
./NuttX/nuttx/drivers/usbhost/usbhost_skeleton.d \
./NuttX/nuttx/drivers/usbhost/usbhost_storage.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/usbhost/%.o: ../NuttX/nuttx/drivers/usbhost/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


