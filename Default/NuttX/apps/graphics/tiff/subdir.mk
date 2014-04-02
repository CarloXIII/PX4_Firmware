################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/graphics/tiff/tiff_addstrip.c \
../NuttX/apps/graphics/tiff/tiff_finalize.c \
../NuttX/apps/graphics/tiff/tiff_initialize.c \
../NuttX/apps/graphics/tiff/tiff_utils.c 

OBJS += \
./NuttX/apps/graphics/tiff/tiff_addstrip.o \
./NuttX/apps/graphics/tiff/tiff_finalize.o \
./NuttX/apps/graphics/tiff/tiff_initialize.o \
./NuttX/apps/graphics/tiff/tiff_utils.o 

C_DEPS += \
./NuttX/apps/graphics/tiff/tiff_addstrip.d \
./NuttX/apps/graphics/tiff/tiff_finalize.d \
./NuttX/apps/graphics/tiff/tiff_initialize.d \
./NuttX/apps/graphics/tiff/tiff_utils.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/graphics/tiff/%.o: ../NuttX/apps/graphics/tiff/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


