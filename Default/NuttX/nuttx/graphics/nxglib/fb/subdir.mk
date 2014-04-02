################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_copyrectangle.c \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_fillrectangle.c \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_filltrapezoid.c \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_getrectangle.c \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_moverectangle.c \
../NuttX/nuttx/graphics/nxglib/fb/nxglib_setpixel.c 

OBJS += \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_copyrectangle.o \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_fillrectangle.o \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_filltrapezoid.o \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_getrectangle.o \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_moverectangle.o \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_setpixel.o 

C_DEPS += \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_copyrectangle.d \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_fillrectangle.d \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_filltrapezoid.d \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_getrectangle.d \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_moverectangle.d \
./NuttX/nuttx/graphics/nxglib/fb/nxglib_setpixel.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/graphics/nxglib/fb/%.o: ../NuttX/nuttx/graphics/nxglib/fb/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


