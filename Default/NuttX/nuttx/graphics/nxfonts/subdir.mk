################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/graphics/nxfonts/nxfonts_bitmaps.c \
../NuttX/nuttx/graphics/nxfonts/nxfonts_convert.c \
../NuttX/nuttx/graphics/nxfonts/nxfonts_getfont.c 

OBJS += \
./NuttX/nuttx/graphics/nxfonts/nxfonts_bitmaps.o \
./NuttX/nuttx/graphics/nxfonts/nxfonts_convert.o \
./NuttX/nuttx/graphics/nxfonts/nxfonts_getfont.o 

C_DEPS += \
./NuttX/nuttx/graphics/nxfonts/nxfonts_bitmaps.d \
./NuttX/nuttx/graphics/nxfonts/nxfonts_convert.d \
./NuttX/nuttx/graphics/nxfonts/nxfonts_getfont.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/graphics/nxfonts/%.o: ../NuttX/nuttx/graphics/nxfonts/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


