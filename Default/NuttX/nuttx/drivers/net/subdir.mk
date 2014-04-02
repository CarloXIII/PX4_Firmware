################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/drivers/net/cs89x0.c \
../NuttX/nuttx/drivers/net/dm90x0.c \
../NuttX/nuttx/drivers/net/e1000.c \
../NuttX/nuttx/drivers/net/enc28j60.c \
../NuttX/nuttx/drivers/net/skeleton.c \
../NuttX/nuttx/drivers/net/slip.c \
../NuttX/nuttx/drivers/net/vnet.c 

OBJS += \
./NuttX/nuttx/drivers/net/cs89x0.o \
./NuttX/nuttx/drivers/net/dm90x0.o \
./NuttX/nuttx/drivers/net/e1000.o \
./NuttX/nuttx/drivers/net/enc28j60.o \
./NuttX/nuttx/drivers/net/skeleton.o \
./NuttX/nuttx/drivers/net/slip.o \
./NuttX/nuttx/drivers/net/vnet.o 

C_DEPS += \
./NuttX/nuttx/drivers/net/cs89x0.d \
./NuttX/nuttx/drivers/net/dm90x0.d \
./NuttX/nuttx/drivers/net/e1000.d \
./NuttX/nuttx/drivers/net/enc28j60.d \
./NuttX/nuttx/drivers/net/skeleton.d \
./NuttX/nuttx/drivers/net/slip.d \
./NuttX/nuttx/drivers/net/vnet.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/drivers/net/%.o: ../NuttX/nuttx/drivers/net/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


