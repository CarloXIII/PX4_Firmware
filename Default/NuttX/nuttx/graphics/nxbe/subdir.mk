################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/graphics/nxbe/nxbe_bitmap.c \
../NuttX/nuttx/graphics/nxbe/nxbe_clipper.c \
../NuttX/nuttx/graphics/nxbe/nxbe_closewindow.c \
../NuttX/nuttx/graphics/nxbe/nxbe_colormap.c \
../NuttX/nuttx/graphics/nxbe/nxbe_configure.c \
../NuttX/nuttx/graphics/nxbe/nxbe_fill.c \
../NuttX/nuttx/graphics/nxbe/nxbe_filltrapezoid.c \
../NuttX/nuttx/graphics/nxbe/nxbe_getrectangle.c \
../NuttX/nuttx/graphics/nxbe/nxbe_lower.c \
../NuttX/nuttx/graphics/nxbe/nxbe_move.c \
../NuttX/nuttx/graphics/nxbe/nxbe_raise.c \
../NuttX/nuttx/graphics/nxbe/nxbe_redraw.c \
../NuttX/nuttx/graphics/nxbe/nxbe_redrawbelow.c \
../NuttX/nuttx/graphics/nxbe/nxbe_setpixel.c \
../NuttX/nuttx/graphics/nxbe/nxbe_setposition.c \
../NuttX/nuttx/graphics/nxbe/nxbe_setsize.c \
../NuttX/nuttx/graphics/nxbe/nxbe_visible.c 

OBJS += \
./NuttX/nuttx/graphics/nxbe/nxbe_bitmap.o \
./NuttX/nuttx/graphics/nxbe/nxbe_clipper.o \
./NuttX/nuttx/graphics/nxbe/nxbe_closewindow.o \
./NuttX/nuttx/graphics/nxbe/nxbe_colormap.o \
./NuttX/nuttx/graphics/nxbe/nxbe_configure.o \
./NuttX/nuttx/graphics/nxbe/nxbe_fill.o \
./NuttX/nuttx/graphics/nxbe/nxbe_filltrapezoid.o \
./NuttX/nuttx/graphics/nxbe/nxbe_getrectangle.o \
./NuttX/nuttx/graphics/nxbe/nxbe_lower.o \
./NuttX/nuttx/graphics/nxbe/nxbe_move.o \
./NuttX/nuttx/graphics/nxbe/nxbe_raise.o \
./NuttX/nuttx/graphics/nxbe/nxbe_redraw.o \
./NuttX/nuttx/graphics/nxbe/nxbe_redrawbelow.o \
./NuttX/nuttx/graphics/nxbe/nxbe_setpixel.o \
./NuttX/nuttx/graphics/nxbe/nxbe_setposition.o \
./NuttX/nuttx/graphics/nxbe/nxbe_setsize.o \
./NuttX/nuttx/graphics/nxbe/nxbe_visible.o 

C_DEPS += \
./NuttX/nuttx/graphics/nxbe/nxbe_bitmap.d \
./NuttX/nuttx/graphics/nxbe/nxbe_clipper.d \
./NuttX/nuttx/graphics/nxbe/nxbe_closewindow.d \
./NuttX/nuttx/graphics/nxbe/nxbe_colormap.d \
./NuttX/nuttx/graphics/nxbe/nxbe_configure.d \
./NuttX/nuttx/graphics/nxbe/nxbe_fill.d \
./NuttX/nuttx/graphics/nxbe/nxbe_filltrapezoid.d \
./NuttX/nuttx/graphics/nxbe/nxbe_getrectangle.d \
./NuttX/nuttx/graphics/nxbe/nxbe_lower.d \
./NuttX/nuttx/graphics/nxbe/nxbe_move.d \
./NuttX/nuttx/graphics/nxbe/nxbe_raise.d \
./NuttX/nuttx/graphics/nxbe/nxbe_redraw.d \
./NuttX/nuttx/graphics/nxbe/nxbe_redrawbelow.d \
./NuttX/nuttx/graphics/nxbe/nxbe_setpixel.d \
./NuttX/nuttx/graphics/nxbe/nxbe_setposition.d \
./NuttX/nuttx/graphics/nxbe/nxbe_setsize.d \
./NuttX/nuttx/graphics/nxbe/nxbe_visible.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/graphics/nxbe/%.o: ../NuttX/nuttx/graphics/nxbe/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


