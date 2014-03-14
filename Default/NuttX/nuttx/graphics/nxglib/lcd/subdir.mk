################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_copyrectangle.c \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_fillrectangle.c \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_filltrapezoid.c \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_getrectangle.c \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_moverectangle.c \
../NuttX/nuttx/graphics/nxglib/lcd/nxglib_setpixel.c 

OBJS += \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_copyrectangle.o \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_fillrectangle.o \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_filltrapezoid.o \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_getrectangle.o \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_moverectangle.o \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_setpixel.o 

C_DEPS += \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_copyrectangle.d \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_fillrectangle.d \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_filltrapezoid.d \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_getrectangle.d \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_moverectangle.d \
./NuttX/nuttx/graphics/nxglib/lcd/nxglib_setpixel.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/graphics/nxglib/lcd/%.o: ../NuttX/nuttx/graphics/nxglib/lcd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


