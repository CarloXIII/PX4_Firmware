################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/misc/pascal/plink/plink.c \
../NuttX/misc/pascal/plink/plreloc.c \
../NuttX/misc/pascal/plink/plsym.c 

OBJS += \
./NuttX/misc/pascal/plink/plink.o \
./NuttX/misc/pascal/plink/plreloc.o \
./NuttX/misc/pascal/plink/plsym.o 

C_DEPS += \
./NuttX/misc/pascal/plink/plink.d \
./NuttX/misc/pascal/plink/plreloc.d \
./NuttX/misc/pascal/plink/plsym.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/misc/pascal/plink/%.o: ../NuttX/misc/pascal/plink/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


