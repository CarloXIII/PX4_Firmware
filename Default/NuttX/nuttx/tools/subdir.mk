################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/tools/b16.c \
../NuttX/nuttx/tools/bdf-converter.c \
../NuttX/nuttx/tools/cfgdefine.c \
../NuttX/nuttx/tools/cfgparser.c \
../NuttX/nuttx/tools/cmpconfig.c \
../NuttX/nuttx/tools/configure.c \
../NuttX/nuttx/tools/csvparser.c \
../NuttX/nuttx/tools/kconfig2html.c \
../NuttX/nuttx/tools/mkconfig.c \
../NuttX/nuttx/tools/mkdeps.c \
../NuttX/nuttx/tools/mksymtab.c \
../NuttX/nuttx/tools/mksyscall.c \
../NuttX/nuttx/tools/mkversion.c 

OBJS += \
./NuttX/nuttx/tools/b16.o \
./NuttX/nuttx/tools/bdf-converter.o \
./NuttX/nuttx/tools/cfgdefine.o \
./NuttX/nuttx/tools/cfgparser.o \
./NuttX/nuttx/tools/cmpconfig.o \
./NuttX/nuttx/tools/configure.o \
./NuttX/nuttx/tools/csvparser.o \
./NuttX/nuttx/tools/kconfig2html.o \
./NuttX/nuttx/tools/mkconfig.o \
./NuttX/nuttx/tools/mkdeps.o \
./NuttX/nuttx/tools/mksymtab.o \
./NuttX/nuttx/tools/mksyscall.o \
./NuttX/nuttx/tools/mkversion.o 

C_DEPS += \
./NuttX/nuttx/tools/b16.d \
./NuttX/nuttx/tools/bdf-converter.d \
./NuttX/nuttx/tools/cfgdefine.d \
./NuttX/nuttx/tools/cfgparser.d \
./NuttX/nuttx/tools/cmpconfig.d \
./NuttX/nuttx/tools/configure.d \
./NuttX/nuttx/tools/csvparser.d \
./NuttX/nuttx/tools/kconfig2html.d \
./NuttX/nuttx/tools/mkconfig.d \
./NuttX/nuttx/tools/mkdeps.d \
./NuttX/nuttx/tools/mksymtab.d \
./NuttX/nuttx/tools/mksyscall.d \
./NuttX/nuttx/tools/mkversion.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/tools/%.o: ../NuttX/nuttx/tools/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


