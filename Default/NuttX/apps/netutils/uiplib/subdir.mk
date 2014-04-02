################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../NuttX/apps/netutils/uiplib/uip_gethostaddr.c \
../NuttX/apps/netutils/uiplib/uip_getifflag.c \
../NuttX/apps/netutils/uiplib/uip_getmacaddr.c \
../NuttX/apps/netutils/uiplib/uip_ipmsfilter.c \
../NuttX/apps/netutils/uiplib/uip_listenon.c \
../NuttX/apps/netutils/uiplib/uip_parsehttpurl.c \
../NuttX/apps/netutils/uiplib/uip_server.c \
../NuttX/apps/netutils/uiplib/uip_setdraddr.c \
../NuttX/apps/netutils/uiplib/uip_sethostaddr.c \
../NuttX/apps/netutils/uiplib/uip_setifflag.c \
../NuttX/apps/netutils/uiplib/uip_setmacaddr.c \
../NuttX/apps/netutils/uiplib/uip_setnetmask.c \
../NuttX/apps/netutils/uiplib/uiplib.c 

OBJS += \
./NuttX/apps/netutils/uiplib/uip_gethostaddr.o \
./NuttX/apps/netutils/uiplib/uip_getifflag.o \
./NuttX/apps/netutils/uiplib/uip_getmacaddr.o \
./NuttX/apps/netutils/uiplib/uip_ipmsfilter.o \
./NuttX/apps/netutils/uiplib/uip_listenon.o \
./NuttX/apps/netutils/uiplib/uip_parsehttpurl.o \
./NuttX/apps/netutils/uiplib/uip_server.o \
./NuttX/apps/netutils/uiplib/uip_setdraddr.o \
./NuttX/apps/netutils/uiplib/uip_sethostaddr.o \
./NuttX/apps/netutils/uiplib/uip_setifflag.o \
./NuttX/apps/netutils/uiplib/uip_setmacaddr.o \
./NuttX/apps/netutils/uiplib/uip_setnetmask.o \
./NuttX/apps/netutils/uiplib/uiplib.o 

C_DEPS += \
./NuttX/apps/netutils/uiplib/uip_gethostaddr.d \
./NuttX/apps/netutils/uiplib/uip_getifflag.d \
./NuttX/apps/netutils/uiplib/uip_getmacaddr.d \
./NuttX/apps/netutils/uiplib/uip_ipmsfilter.d \
./NuttX/apps/netutils/uiplib/uip_listenon.d \
./NuttX/apps/netutils/uiplib/uip_parsehttpurl.d \
./NuttX/apps/netutils/uiplib/uip_server.d \
./NuttX/apps/netutils/uiplib/uip_setdraddr.d \
./NuttX/apps/netutils/uiplib/uip_sethostaddr.d \
./NuttX/apps/netutils/uiplib/uip_setifflag.d \
./NuttX/apps/netutils/uiplib/uip_setmacaddr.d \
./NuttX/apps/netutils/uiplib/uip_setnetmask.d \
./NuttX/apps/netutils/uiplib/uiplib.d 


# Each subdirectory must supply rules for building sources it contributes
NuttX/apps/netutils/uiplib/%.o: ../NuttX/apps/netutils/uiplib/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O2 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


