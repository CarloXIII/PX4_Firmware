################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nxtext/nxtext_bkgd.c \
../NuttX/apps/examples/nxtext/nxtext_main.c \
../NuttX/apps/examples/nxtext/nxtext_popup.c \
../NuttX/apps/examples/nxtext/nxtext_putc.c \
../NuttX/apps/examples/nxtext/nxtext_server.c 

OBJS += \
./NuttX/apps/examples/nxtext/nxtext_bkgd.o \
./NuttX/apps/examples/nxtext/nxtext_main.o \
./NuttX/apps/examples/nxtext/nxtext_popup.o \
./NuttX/apps/examples/nxtext/nxtext_putc.o \
./NuttX/apps/examples/nxtext/nxtext_server.o 

C_DEPS += \
./NuttX/apps/examples/nxtext/nxtext_bkgd.d \
./NuttX/apps/examples/nxtext/nxtext_main.d \
./NuttX/apps/examples/nxtext/nxtext_popup.d \
./NuttX/apps/examples/nxtext/nxtext_putc.d \
./NuttX/apps/examples/nxtext/nxtext_server.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nxtext/%.o: ../NuttX/apps/examples/nxtext/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


