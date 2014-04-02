################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/usbdev/cdcacm.c \
../NuttX/nuttx/drivers/usbdev/cdcacm_desc.c \
../NuttX/nuttx/drivers/usbdev/composite.c \
../NuttX/nuttx/drivers/usbdev/composite_desc.c \
../NuttX/nuttx/drivers/usbdev/pl2303.c \
../NuttX/nuttx/drivers/usbdev/usbdev_trace.c \
../NuttX/nuttx/drivers/usbdev/usbdev_trprintf.c \
../NuttX/nuttx/drivers/usbdev/usbmsc.c \
../NuttX/nuttx/drivers/usbdev/usbmsc_desc.c \
../NuttX/nuttx/drivers/usbdev/usbmsc_scsi.c 

OBJS += \
./NuttX/nuttx/drivers/usbdev/cdcacm.o \
./NuttX/nuttx/drivers/usbdev/cdcacm_desc.o \
./NuttX/nuttx/drivers/usbdev/composite.o \
./NuttX/nuttx/drivers/usbdev/composite_desc.o \
./NuttX/nuttx/drivers/usbdev/pl2303.o \
./NuttX/nuttx/drivers/usbdev/usbdev_trace.o \
./NuttX/nuttx/drivers/usbdev/usbdev_trprintf.o \
./NuttX/nuttx/drivers/usbdev/usbmsc.o \
./NuttX/nuttx/drivers/usbdev/usbmsc_desc.o \
./NuttX/nuttx/drivers/usbdev/usbmsc_scsi.o 

C_DEPS += \
./NuttX/nuttx/drivers/usbdev/cdcacm.d \
./NuttX/nuttx/drivers/usbdev/cdcacm_desc.d \
./NuttX/nuttx/drivers/usbdev/composite.d \
./NuttX/nuttx/drivers/usbdev/composite_desc.d \
./NuttX/nuttx/drivers/usbdev/pl2303.d \
./NuttX/nuttx/drivers/usbdev/usbdev_trace.d \
./NuttX/nuttx/drivers/usbdev/usbdev_trprintf.d \
./NuttX/nuttx/drivers/usbdev/usbmsc.d \
./NuttX/nuttx/drivers/usbdev/usbmsc_desc.d \
./NuttX/nuttx/drivers/usbdev/usbmsc_scsi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/usbdev/%.o: ../NuttX/nuttx/drivers/usbdev/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


