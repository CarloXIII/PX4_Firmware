################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/systemcmds/mtd/24xxxx_mtd.c \
../src/systemcmds/mtd/mtd.c 

OBJS += \
./src/systemcmds/mtd/24xxxx_mtd.o \
./src/systemcmds/mtd/mtd.o 

C_DEPS += \
./src/systemcmds/mtd/24xxxx_mtd.d \
./src/systemcmds/mtd/mtd.d 


# Each subdirectory must supply rules for building sources it contributes
src/systemcmds/mtd/%.o: ../src/systemcmds/mtd/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


