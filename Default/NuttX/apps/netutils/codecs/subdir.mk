################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/codecs/base64.c \
../NuttX/apps/netutils/codecs/md5.c \
../NuttX/apps/netutils/codecs/urldecode.c 

OBJS += \
./NuttX/apps/netutils/codecs/base64.o \
./NuttX/apps/netutils/codecs/md5.o \
./NuttX/apps/netutils/codecs/urldecode.o 

C_DEPS += \
./NuttX/apps/netutils/codecs/base64.d \
./NuttX/apps/netutils/codecs/md5.d \
./NuttX/apps/netutils/codecs/urldecode.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/codecs/%.o: ../NuttX/apps/netutils/codecs/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


