################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/thttpd/cgi-src/phf.c \
../NuttX/apps/netutils/thttpd/cgi-src/redirect.c \
../NuttX/apps/netutils/thttpd/cgi-src/ssi.c 

OBJS += \
./NuttX/apps/netutils/thttpd/cgi-src/phf.o \
./NuttX/apps/netutils/thttpd/cgi-src/redirect.o \
./NuttX/apps/netutils/thttpd/cgi-src/ssi.o 

C_DEPS += \
./NuttX/apps/netutils/thttpd/cgi-src/phf.d \
./NuttX/apps/netutils/thttpd/cgi-src/redirect.d \
./NuttX/apps/netutils/thttpd/cgi-src/ssi.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/thttpd/cgi-src/%.o: ../NuttX/apps/netutils/thttpd/cgi-src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


