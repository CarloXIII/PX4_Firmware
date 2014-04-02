################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/examples/nximage/nximage_bitmap.c \
../NuttX/apps/examples/nximage/nximage_bkgd.c \
../NuttX/apps/examples/nximage/nximage_main.c 

OBJS += \
./NuttX/apps/examples/nximage/nximage_bitmap.o \
./NuttX/apps/examples/nximage/nximage_bkgd.o \
./NuttX/apps/examples/nximage/nximage_main.o 

C_DEPS += \
./NuttX/apps/examples/nximage/nximage_bitmap.d \
./NuttX/apps/examples/nximage/nximage_bkgd.d \
./NuttX/apps/examples/nximage/nximage_main.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/examples/nximage/%.o: ../NuttX/apps/examples/nximage/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


