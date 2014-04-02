################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/binfmt/binfmt_dumpmodule.c \
../NuttX/nuttx/binfmt/binfmt_exec.c \
../NuttX/nuttx/binfmt/binfmt_execmodule.c \
../NuttX/nuttx/binfmt/binfmt_exepath.c \
../NuttX/nuttx/binfmt/binfmt_globals.c \
../NuttX/nuttx/binfmt/binfmt_loadmodule.c \
../NuttX/nuttx/binfmt/binfmt_register.c \
../NuttX/nuttx/binfmt/binfmt_schedunload.c \
../NuttX/nuttx/binfmt/binfmt_unloadmodule.c \
../NuttX/nuttx/binfmt/binfmt_unregister.c \
../NuttX/nuttx/binfmt/builtin.c \
../NuttX/nuttx/binfmt/elf.c \
../NuttX/nuttx/binfmt/nxflat.c \
../NuttX/nuttx/binfmt/symtab_findbyname.c \
../NuttX/nuttx/binfmt/symtab_findbyvalue.c \
../NuttX/nuttx/binfmt/symtab_findorderedbyname.c \
../NuttX/nuttx/binfmt/symtab_findorderedbyvalue.c 

OBJS += \
./NuttX/nuttx/binfmt/binfmt_dumpmodule.o \
./NuttX/nuttx/binfmt/binfmt_exec.o \
./NuttX/nuttx/binfmt/binfmt_execmodule.o \
./NuttX/nuttx/binfmt/binfmt_exepath.o \
./NuttX/nuttx/binfmt/binfmt_globals.o \
./NuttX/nuttx/binfmt/binfmt_loadmodule.o \
./NuttX/nuttx/binfmt/binfmt_register.o \
./NuttX/nuttx/binfmt/binfmt_schedunload.o \
./NuttX/nuttx/binfmt/binfmt_unloadmodule.o \
./NuttX/nuttx/binfmt/binfmt_unregister.o \
./NuttX/nuttx/binfmt/builtin.o \
./NuttX/nuttx/binfmt/elf.o \
./NuttX/nuttx/binfmt/nxflat.o \
./NuttX/nuttx/binfmt/symtab_findbyname.o \
./NuttX/nuttx/binfmt/symtab_findbyvalue.o \
./NuttX/nuttx/binfmt/symtab_findorderedbyname.o \
./NuttX/nuttx/binfmt/symtab_findorderedbyvalue.o 

C_DEPS += \
./NuttX/nuttx/binfmt/binfmt_dumpmodule.d \
./NuttX/nuttx/binfmt/binfmt_exec.d \
./NuttX/nuttx/binfmt/binfmt_execmodule.d \
./NuttX/nuttx/binfmt/binfmt_exepath.d \
./NuttX/nuttx/binfmt/binfmt_globals.d \
./NuttX/nuttx/binfmt/binfmt_loadmodule.d \
./NuttX/nuttx/binfmt/binfmt_register.d \
./NuttX/nuttx/binfmt/binfmt_schedunload.d \
./NuttX/nuttx/binfmt/binfmt_unloadmodule.d \
./NuttX/nuttx/binfmt/binfmt_unregister.d \
./NuttX/nuttx/binfmt/builtin.d \
./NuttX/nuttx/binfmt/elf.d \
./NuttX/nuttx/binfmt/nxflat.d \
./NuttX/nuttx/binfmt/symtab_findbyname.d \
./NuttX/nuttx/binfmt/symtab_findbyvalue.d \
./NuttX/nuttx/binfmt/symtab_findorderedbyname.d \
./NuttX/nuttx/binfmt/symtab_findorderedbyvalue.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/binfmt/%.o: ../NuttX/nuttx/binfmt/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


