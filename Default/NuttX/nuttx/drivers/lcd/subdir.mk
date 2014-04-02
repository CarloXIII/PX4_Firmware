################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/lcd/mio283qt2.c \
../NuttX/nuttx/drivers/lcd/nokia6100.c \
../NuttX/nuttx/drivers/lcd/p14201.c \
../NuttX/nuttx/drivers/lcd/skeleton.c \
../NuttX/nuttx/drivers/lcd/ssd1289.c \
../NuttX/nuttx/drivers/lcd/st7567.c \
../NuttX/nuttx/drivers/lcd/ug-2864ambag01.c \
../NuttX/nuttx/drivers/lcd/ug-2864hsweg01.c \
../NuttX/nuttx/drivers/lcd/ug-9664hswag01.c 

OBJS += \
./NuttX/nuttx/drivers/lcd/mio283qt2.o \
./NuttX/nuttx/drivers/lcd/nokia6100.o \
./NuttX/nuttx/drivers/lcd/p14201.o \
./NuttX/nuttx/drivers/lcd/skeleton.o \
./NuttX/nuttx/drivers/lcd/ssd1289.o \
./NuttX/nuttx/drivers/lcd/st7567.o \
./NuttX/nuttx/drivers/lcd/ug-2864ambag01.o \
./NuttX/nuttx/drivers/lcd/ug-2864hsweg01.o \
./NuttX/nuttx/drivers/lcd/ug-9664hswag01.o 

C_DEPS += \
./NuttX/nuttx/drivers/lcd/mio283qt2.d \
./NuttX/nuttx/drivers/lcd/nokia6100.d \
./NuttX/nuttx/drivers/lcd/p14201.d \
./NuttX/nuttx/drivers/lcd/skeleton.d \
./NuttX/nuttx/drivers/lcd/ssd1289.d \
./NuttX/nuttx/drivers/lcd/st7567.d \
./NuttX/nuttx/drivers/lcd/ug-2864ambag01.d \
./NuttX/nuttx/drivers/lcd/ug-2864hsweg01.d \
./NuttX/nuttx/drivers/lcd/ug-9664hswag01.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/lcd/%.o: ../NuttX/nuttx/drivers/lcd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


