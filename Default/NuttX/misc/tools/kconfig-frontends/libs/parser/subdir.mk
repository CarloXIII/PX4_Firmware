################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/tools/kconfig-frontends/libs/parser/confdata.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/expr.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/lconf.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/menu.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/symbol.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/util.c \
../NuttX/misc/tools/kconfig-frontends/libs/parser/yconf.c 

OBJS += \
./NuttX/misc/tools/kconfig-frontends/libs/parser/confdata.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/expr.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/lconf.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/menu.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/symbol.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/util.o \
./NuttX/misc/tools/kconfig-frontends/libs/parser/yconf.o 

C_DEPS += \
./NuttX/misc/tools/kconfig-frontends/libs/parser/confdata.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/expr.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/lconf.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/menu.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/symbol.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/util.d \
./NuttX/misc/tools/kconfig-frontends/libs/parser/yconf.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/tools/kconfig-frontends/libs/parser/%.o: ../NuttX/misc/tools/kconfig-frontends/libs/parser/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


