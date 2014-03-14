################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/pascal/pas.c \
../NuttX/misc/pascal/pascal/pblck.c \
../NuttX/misc/pascal/pascal/pcexpr.c \
../NuttX/misc/pascal/pascal/pcfunc.c \
../NuttX/misc/pascal/pascal/perr.c \
../NuttX/misc/pascal/pascal/pexpr.c \
../NuttX/misc/pascal/pascal/pffunc.c \
../NuttX/misc/pascal/pascal/pgen.c \
../NuttX/misc/pascal/pascal/pprgm.c \
../NuttX/misc/pascal/pascal/pproc.c \
../NuttX/misc/pascal/pascal/pstm.c \
../NuttX/misc/pascal/pascal/ptbl.c \
../NuttX/misc/pascal/pascal/ptkn.c \
../NuttX/misc/pascal/pascal/punit.c 

OBJS += \
./NuttX/misc/pascal/pascal/pas.o \
./NuttX/misc/pascal/pascal/pblck.o \
./NuttX/misc/pascal/pascal/pcexpr.o \
./NuttX/misc/pascal/pascal/pcfunc.o \
./NuttX/misc/pascal/pascal/perr.o \
./NuttX/misc/pascal/pascal/pexpr.o \
./NuttX/misc/pascal/pascal/pffunc.o \
./NuttX/misc/pascal/pascal/pgen.o \
./NuttX/misc/pascal/pascal/pprgm.o \
./NuttX/misc/pascal/pascal/pproc.o \
./NuttX/misc/pascal/pascal/pstm.o \
./NuttX/misc/pascal/pascal/ptbl.o \
./NuttX/misc/pascal/pascal/ptkn.o \
./NuttX/misc/pascal/pascal/punit.o 

C_DEPS += \
./NuttX/misc/pascal/pascal/pas.d \
./NuttX/misc/pascal/pascal/pblck.d \
./NuttX/misc/pascal/pascal/pcexpr.d \
./NuttX/misc/pascal/pascal/pcfunc.d \
./NuttX/misc/pascal/pascal/perr.d \
./NuttX/misc/pascal/pascal/pexpr.d \
./NuttX/misc/pascal/pascal/pffunc.d \
./NuttX/misc/pascal/pascal/pgen.d \
./NuttX/misc/pascal/pascal/pprgm.d \
./NuttX/misc/pascal/pascal/pproc.d \
./NuttX/misc/pascal/pascal/pstm.d \
./NuttX/misc/pascal/pascal/ptbl.d \
./NuttX/misc/pascal/pascal/ptkn.d \
./NuttX/misc/pascal/pascal/punit.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/pascal/%.o: ../NuttX/misc/pascal/pascal/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


