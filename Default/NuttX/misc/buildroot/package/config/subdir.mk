################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/buildroot/package/config/conf.c \
../NuttX/misc/buildroot/package/config/confdata.c \
../NuttX/misc/buildroot/package/config/expr.c \
../NuttX/misc/buildroot/package/config/mconf.c \
../NuttX/misc/buildroot/package/config/menu.c \
../NuttX/misc/buildroot/package/config/symbol.c \
../NuttX/misc/buildroot/package/config/util.c 

OBJS += \
./NuttX/misc/buildroot/package/config/conf.o \
./NuttX/misc/buildroot/package/config/confdata.o \
./NuttX/misc/buildroot/package/config/expr.o \
./NuttX/misc/buildroot/package/config/mconf.o \
./NuttX/misc/buildroot/package/config/menu.o \
./NuttX/misc/buildroot/package/config/symbol.o \
./NuttX/misc/buildroot/package/config/util.o 

C_DEPS += \
./NuttX/misc/buildroot/package/config/conf.d \
./NuttX/misc/buildroot/package/config/confdata.d \
./NuttX/misc/buildroot/package/config/expr.d \
./NuttX/misc/buildroot/package/config/mconf.d \
./NuttX/misc/buildroot/package/config/menu.d \
./NuttX/misc/buildroot/package/config/symbol.d \
./NuttX/misc/buildroot/package/config/util.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/buildroot/package/config/%.o: ../NuttX/misc/buildroot/package/config/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


