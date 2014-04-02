################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/nuttx/configs/sam4s-xplained/src/sam_autoleds.c \
../NuttX/nuttx/configs/sam4s-xplained/src/sam_boot.c \
../NuttX/nuttx/configs/sam4s-xplained/src/sam_buttons.c \
../NuttX/nuttx/configs/sam4s-xplained/src/sam_cxxinitialize.c \
../NuttX/nuttx/configs/sam4s-xplained/src/sam_userleds.c 

OBJS += \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_autoleds.o \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_boot.o \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_buttons.o \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_cxxinitialize.o \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_userleds.o 

C_DEPS += \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_autoleds.d \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_boot.d \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_buttons.d \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_cxxinitialize.d \
./NuttX/nuttx/configs/sam4s-xplained/src/sam_userleds.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/nuttx/configs/sam4s-xplained/src/%.o: ../NuttX/nuttx/configs/sam4s-xplained/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


