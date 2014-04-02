################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/binfmt/libelf/libelf_addrenv.c \
../NuttX/nuttx/binfmt/libelf/libelf_bind.c \
../NuttX/nuttx/binfmt/libelf/libelf_ctors.c \
../NuttX/nuttx/binfmt/libelf/libelf_dtors.c \
../NuttX/nuttx/binfmt/libelf/libelf_init.c \
../NuttX/nuttx/binfmt/libelf/libelf_iobuffer.c \
../NuttX/nuttx/binfmt/libelf/libelf_load.c \
../NuttX/nuttx/binfmt/libelf/libelf_read.c \
../NuttX/nuttx/binfmt/libelf/libelf_sections.c \
../NuttX/nuttx/binfmt/libelf/libelf_symbols.c \
../NuttX/nuttx/binfmt/libelf/libelf_uninit.c \
../NuttX/nuttx/binfmt/libelf/libelf_unload.c \
../NuttX/nuttx/binfmt/libelf/libelf_verify.c 

OBJS += \
./NuttX/nuttx/binfmt/libelf/libelf_addrenv.o \
./NuttX/nuttx/binfmt/libelf/libelf_bind.o \
./NuttX/nuttx/binfmt/libelf/libelf_ctors.o \
./NuttX/nuttx/binfmt/libelf/libelf_dtors.o \
./NuttX/nuttx/binfmt/libelf/libelf_init.o \
./NuttX/nuttx/binfmt/libelf/libelf_iobuffer.o \
./NuttX/nuttx/binfmt/libelf/libelf_load.o \
./NuttX/nuttx/binfmt/libelf/libelf_read.o \
./NuttX/nuttx/binfmt/libelf/libelf_sections.o \
./NuttX/nuttx/binfmt/libelf/libelf_symbols.o \
./NuttX/nuttx/binfmt/libelf/libelf_uninit.o \
./NuttX/nuttx/binfmt/libelf/libelf_unload.o \
./NuttX/nuttx/binfmt/libelf/libelf_verify.o 

C_DEPS += \
./NuttX/nuttx/binfmt/libelf/libelf_addrenv.d \
./NuttX/nuttx/binfmt/libelf/libelf_bind.d \
./NuttX/nuttx/binfmt/libelf/libelf_ctors.d \
./NuttX/nuttx/binfmt/libelf/libelf_dtors.d \
./NuttX/nuttx/binfmt/libelf/libelf_init.d \
./NuttX/nuttx/binfmt/libelf/libelf_iobuffer.d \
./NuttX/nuttx/binfmt/libelf/libelf_load.d \
./NuttX/nuttx/binfmt/libelf/libelf_read.d \
./NuttX/nuttx/binfmt/libelf/libelf_sections.d \
./NuttX/nuttx/binfmt/libelf/libelf_symbols.d \
./NuttX/nuttx/binfmt/libelf/libelf_uninit.d \
./NuttX/nuttx/binfmt/libelf/libelf_unload.d \
./NuttX/nuttx/binfmt/libelf/libelf_verify.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/binfmt/libelf/%.o: ../NuttX/nuttx/binfmt/libelf/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


